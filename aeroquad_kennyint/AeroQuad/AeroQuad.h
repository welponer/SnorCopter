/*
  AeroQuad v2.5 Beta 1 - July 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h>
#include <math.h>
#include "WProgram.h"
#include "pins_arduino.h"

// Flight Software Version
#define VERSION 5.0

#define ISR_FRAME_COUNT        500
#define SUM_COUNT              5.0   // Sum of 5 for average
#define BACKGROUND_COUNT       5     // Number of 500 Hz frames for 100 Hz
#define I2C_ESC_COUNT          5     // Number of 500 Hz frames for 100 Hz
#define COMPASS_COUNT          10    // Number of 500 Hz frames for 50 Hz
#define PRESSURE_COUNT         10    // Number of 500 Hz frames for 50 Hz
#define RECEIVER_COUNT         10    // Number of 500 Hz frames for 50 Hz
#define ALTITUDE_COUNT         50    // Number of 500 Hz frames for 10 Hz
#define SERIAL_COM_COUNT       50    // Number of 500 Hz frames for 10 Hz
#define TIMER0_COUNT0          250   // For 8 bit system timer with prescaler = 64
#define TIMER0_COUNT1          250   // 16E6/64/500 = 500, 250 + 250 = 500

#define dt 0.01       // dt for 100 Hz loop

#define ON 1
#define OFF 0

#if ((defined AeroQuad_Mini_FFIMUV2 && defined isrSourceIsITG3200) || \
     (defined AeroQuad_Mini         && defined isrSourceIsITG3200) || \
     (defined AeroQuad_v18          && defined isrSourceIsITG3200))
  #define INITIALIZED_LED 13
  #define ARMED_LED        4
  #define RATE_LED         7 
#endif

#if ((defined AeroQuad_Mini_FFIMUV2 && !defined isrSourceIsITG3200) || \
     (defined AeroQuad_Mini         && !defined isrSourceIsITG3200) || \
     (defined AeroQuad_v18          && !defined isrSourceIsITG3200))
  #define INITIALIZED_LED 13
  #define ARMED_LED       12
  #define RATE_LED        12 
#endif

#ifdef AeroQuadMega_v2
  #define INITIALIZED_LED 13
  #define ARMED_LED        4
  #define RATE_LED        31
#endif

// Basic axis definitions
#define ROLL     0
#define PITCH    1
#define YAW      2
#define THROTTLE 3
#define MODE     4
#define AUX      5
#define AUX2     6
#define AUX3     7

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define LASTAXIS 3

// Basic PID definitions

#define ROLL_RATE_PID  0
#define PITCH_RATE_PID 1
#define YAW_RATE_PID   2

#define ROLL_ATT_PID   3
#define PITCH_ATT_PID  4
#define HEADING_PID    5

#define LAST_PID       6

float smoothHeading;

// Flight Mode
#define ACRO 0
#define STABLE 1
byte flightMode;
int minAcro; // Read in from EEPROM, defines min throttle during flips

// Heading hold
byte headingHoldConfig;
//float headingScaleFactor;
float commandedYaw = 0;
float headingHold = 0; // calculated adjustment for quad to go to heading (PID output)
float heading = 0; // measured heading from yaw gyro (process variable)
float relativeHeading = 0; // current heading the quad is set to (set point)
float setHeading = 0;
unsigned long headingTime = micros();
byte headingHoldState = OFF;

// batteryMonitor
int autoDescent = 0;

// Communication
char queryType = 'X';
byte tlmType = 0;
byte armed = OFF;
byte safetyCheck = OFF;
byte update = 0;

/**************************************************************/
/******************* Loop timing parameters *******************/
/**************************************************************/

unsigned long currentTime = 0;

// jihlein: wireless telemetry defines
/**************************************************************/
/********************** Wireless Telem Port *******************/
/**************************************************************/
#if defined WirelessTelemetry && defined AeroQuadMega_v2

  #define SERIAL_BAUD       111111
  #define SERIAL_PRINT      Serial3.print
  #define SERIAL_PRINTLN    Serial3.println
  #define SERIAL_AVAILABLE  Serial3.available
  #define SERIAL_READ       Serial3.read
  #define SERIAL_FLUSH      Serial3.flush
  #define SERIAL_BEGIN      Serial3.begin
#else
  #define SERIAL_BAUD       115200
  #define SERIAL_PRINT      Serial.print
  #define SERIAL_PRINTLN    Serial.println
  #define SERIAL_AVAILABLE  Serial.available
  #define SERIAL_READ       Serial.read
  #define SERIAL_FLUSH      Serial.flush
  #define SERIAL_BEGIN      Serial.begin
#endif

// **************************************************************
// *************************** EEPROM ***************************
// **************************************************************
// EEPROM storage addresses

typedef struct {
  float p;
  float i;
  float d;
  float windup;
} t_NVR_PID;

typedef struct {
  float XAXIS_ACCEL_BIAS_ADR;
  float XAXIS_ACCEL_SCALE_FACTOR_ADR;
  float YAXIS_ACCEL_BIAS_ADR;
  float YAXIS_ACCEL_SCALE_FACTOR_ADR;
  float ZAXIS_ACCEL_BIAS_ADR;
  float ZAXIS_ACCEL_SCALE_FACTOR_ADR;

  float XAXIS_MAG_BIAS_ADR;
  float YAXIS_MAG_BIAS_ADR;
  float ZAXIS_MAG_BIAS_ADR;

  t_NVR_PID ROLL_RATE_PID_GAIN_ADR;
  t_NVR_PID PITCH_RATE_PID_GAIN_ADR;
  t_NVR_PID YAW_RATE_PID_GAIN_ADR;
  
  t_NVR_PID ROLL_ATT_PID_GAIN_ADR;
  t_NVR_PID PITCH_ATT_PID_GAIN_ADR;
  t_NVR_PID HEADING_PID_GAIN_ADR;
  
  float WINDUPGUARD_ADR;
  float HEADINGSMOOTH_ADR;
  float HEADINGHOLD_ADR;
  float MINACRO_ADR;
} t_NVR_Data;

float nvrReadFloat(int address); // defined in DataStorage.h
void nvrWriteFloat(float value, int address); // defined in DataStorage.h
void nvrReadPID(unsigned char IDPid, unsigned int IDEeprom);
void nvrWritePID(unsigned char IDPid, unsigned int IDEeprom);

#define GET_NVR_OFFSET(param) ((int)&(((t_NVR_Data*) 0)->param))
#define readFloat(addr) nvrReadFloat(GET_NVR_OFFSET(addr))
#define writeFloat(value, addr) nvrWriteFloat(value, GET_NVR_OFFSET(addr))
#define readPID(IDPid, addr) nvrReadPID(IDPid, GET_NVR_OFFSET(addr))
#define writePID(IDPid, addr) nvrWritePID(IDPid, GET_NVR_OFFSET(addr))

// external function defined
float arctan2(float y, float x);       // defined in AQMath.h
void readEEPROM(void);                 // defined in DataStorage.h
void readPilotCommands(void);          // defined in FlightCommand.pde
void processAltitudeHold(void);        // defined in FlightControl.pde
void comma(void);                      // defined in SerialCom.pde
void fastTelemetry(void);              // defined in SerialCom.pde
void printInt(int data);               // defined in SerialCom.pde
float readFloatSerial(void);           // defined in SerialCom.pde
void readSerialCommand(void);          // defined in SerialCom.pde
void sendBinaryFloat(float);           // defined in SerialCom.pde
void sendBinaryuslong(unsigned long);  // defined in SerialCom.pde
void sendSerialTelemetry(void);        // defined in SerialCom.pde



