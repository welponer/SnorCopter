/*
  AeroQuad v3.0 - April 2011
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
#define SOFTWARE_VERSION 3.0

#define BAUD 115200
//#define BAUD 111111 // use this to be compatible with USB and XBee connections
#define LEDPIN 13
#define ON 1
#define OFF 0

#if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
  #define LED_Red 35
  #define LED_Yellow 36
  #define LED_Green 37
  #define RELE_pin 47
  #define SW1_pin 41
  #define SW2_pin 40
  #define BUZZER 9
  #define PIANO_SW1 42
  #define PIANO_SW2 43
#endif
#ifdef AeroQuadMega_v2  
  #define LED2PIN 4
  #define LED3PIN 31
#else
  #define LED2PIN 12
  #define LED3PIN 12
#endif

// PID Variables
struct PIDdata {
  float P, I, D;
  float lastPosition;
  // AKA experiments with PID
  float previousPIDTime;
  float integratedError;
  float windupGuard; // Thinking about having individual wind up guards for each PID
} PID[10];
// This struct above declares the variable PID[] to hold each of the PID values for various functions
// The following constants are declared in AeroQuad.h
// ROLL = 0, PITCH = 1, YAW = 2 (used for Arcobatic Mode, gyros only)
// ROLLLEVEL = 3, PITCHLEVEL = 4, LEVELGYROROLL = 6, LEVELGYROPITCH = 7 (used for Stable Mode, accels + gyros)
// HEADING = 5 (used for heading hold)
// ALTITUDE = 8 (used for altitude hold)
// ZDAMPENING = 9 (used in altitude hold to dampen vertical accelerations)
float windupGuard; // Read in from EEPROM

// PID types
#define NOTYPE 0
#define TYPEPI 1

// Smoothing filter parameters
#define GYRO 0
#define ACCEL 1
#define FINDZERO 49
float smoothHeading;

// Sensor pin assignments
#define PITCHACCELPIN 0
#define ROLLACCELPIN 1
#define ZACCELPIN 2
#define PITCHRATEPIN 3
#define ROLLRATEPIN 4
#define YAWRATEPIN 5

// Flight Mode
#define ACRO 0
#define STABLE 1
byte flightMode;
unsigned long frameCounter = 0; // main loop executive frame counter
int minAcro; // Read in from EEPROM, defines min throttle during flips


// Analog Reference Value
// This value provided from Configurator
// Use a DMM to measure the voltage between AREF and GND
// Enter the measured voltage below to define your value for aref
// If you don't have a DMM use the following:
// AeroQuad Shield v1.7, aref = 3.0
// AeroQuad Shield v1.6 or below, aref = 2.8
float aref; // Read in from EEPROM

// Auto level setup
float levelAdjust[2] = {0.0,0.0};
//int levelAdjust[2] = {0,0};
  // Scale to convert 1000-2000 PWM to +/- 45 degrees
//float mLevelTransmitter = 0.09;
//float bLevelTransmitter = -135;

#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
  float CHR_RollAngle;
  float CHR_PitchAngle;
#endif

// Heading hold
byte headingHoldConfig;
//float headingScaleFactor;
float commandedYaw = 0;
float headingHold = 0; // calculated adjustment for quad to go to heading (PID output)
float heading = 0; // measured heading from yaw gyro (process variable)
float relativeHeading = 0; // current heading the quad is set to (set point)
//float absoluteHeading = 0;;
float setHeading = 0;
unsigned long headingTime = micros();
byte headingHoldState = OFF;

// batteryMonitor & Altitude Hold
int throttle = 1000;
int batteyMonitorThrottleCorrection = 0;
#if defined (BattMonitor)
  int batteryMonitorStartThrottle = 0;
  unsigned long batteryMonitorStartTime = 0;
  #define BATTERY_MONITOR_THROTTLE_TARGET 1100
  #define BATTERY_MONITOR_GOIN_DOWN_TIME 60000  // 1 minutes
  #if defined BattMonitorAutoDescent
    int batteryMonitorAlarmCounter = 0;
    #define BATTERY_MONITOR_MAX_ALARM_COUNT 20
  #endif
#endif

// Altitude Hold
#ifdef AltitudeHold
  #define ALTPANIC 2 // special state that allows immediate turn off of Altitude hold if large throttle changesa are made at the TX
  #define ALTBUMP 90 // amount of stick movement to cause an altitude bump (up or down)
  #define PANICSTICK_MOVEMENT 250 // 80 if althold on and throttle commanded to move by a gross amount, set PANIC

  float altitudeToHoldTarget = 0.0;
  int altitudeHoldThrottle = 1000;
  boolean isStoreAltitudeNeeded = false;
  boolean altitudeHoldState = OFF;  // ON, OFF or ALTPANIC
#endif
//int altitudeHoldThrottleCorrection = 0;
int minThrottleAdjust = -50;
int maxThrottleAdjust = 50;



//// Receiver variables
int delta;

// Flight angle variables
float timeConstant;

// ESC Calibration
byte calibrateESC = 0;
int testCommand = 1000;

// Communication
char queryType = 'X';
byte tlmType = 0;
byte armed = OFF;
byte safetyCheck = OFF;
byte update = 0;
HardwareSerial *binaryPort;

/**************************************************************/
/******************* Loop timing parameters *******************/
/**************************************************************/
#define RECEIVERLOOPTIME 100000  // 100ms, 10Hz
#define COMPASSLOOPTIME 103000   // 103ms, ~10Hz
#define ALTITUDELOOPTIME 50000   // 50ms x 2, 10Hz (alternates between temperature and pressure measurements)
#define BATTERYLOOPTIME 100000   // 100ms, 10Hz
#define CAMERALOOPTIME 20000     // 20ms, 50Hz
#define FASTTELEMETRYTIME 10000  // 10ms, 100Hz
#define TELEMETRYLOOPTIME 100000 // 100ms, 10Hz for slower computers/cables (more rough Configurator values)

float G_Dt = 0.002;
// Offset starting times so that events don't happen at the same time
// main loop times
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;
// sub loop times
unsigned long oneHZpreviousTime = 0;
unsigned long tenHZpreviousTime = 0;
unsigned long twentyFiveHZpreviousTime = 0;
unsigned long fiftyHZpreviousTime = 0;
unsigned long hundredHZpreviousTime = 0;

#ifdef CameraControl
  unsigned long cameraTime = 10000;
#endif
unsigned long fastTelemetryTime = 0;
//unsigned long telemetryTime = 50000; // make telemetry output 50ms offset from receiver check

// jihlein: wireless telemetry defines
/**************************************************************/
/********************** Wireless Telem Port *******************/
/**************************************************************/
#if defined WirelessTelemetry && (defined(AeroQuadMega_v1)     || \
                                  defined(AeroQuadMega_v2)     || \
                                  defined(AeroQuadMega_Wii)    || \
                                  defined(ArduCopter)          || \
                                  defined(AeroQuadMega_CHR6DM) || \
                                  defined(APM_OP_CHR6DM))
  #define SERIAL_PRINT      Serial3.print
  #define SERIAL_PRINTLN    Serial3.println
  #define SERIAL_AVAILABLE  Serial3.available
  #define SERIAL_READ       Serial3.read
  #define SERIAL_FLUSH      Serial3.flush
  #define SERIAL_BEGIN      Serial3.begin
#else
  #define SERIAL_PRINT      Serial.print
  #define SERIAL_PRINTLN    Serial.println
  #define SERIAL_AVAILABLE  Serial.available
  #define SERIAL_READ       Serial.read
  #define SERIAL_FLUSH      Serial.flush
  #define SERIAL_BEGIN      Serial.begin
#endif

/**************************************************************/
/********************** Debug Parameters **********************/
/**************************************************************/
// Enable/disable control loops for debug
//#define DEBUG
byte fastTransfer = OFF; // Used for troubleshooting
byte testSignal = LOW;

// **************************************************************
// *************************** EEPROM ***************************
// **************************************************************
// EEPROM storage addresses


typedef struct {
  float p;
  float i;
  float d;
} t_NVR_PID;

typedef struct {
  float slope;
  float offset;
  float smooth_factor;
} t_NVR_Receiver;

typedef struct {    
  t_NVR_PID ROLL_PID_GAIN_ADR;
  t_NVR_PID LEVELROLL_PID_GAIN_ADR;
  t_NVR_PID YAW_PID_GAIN_ADR;
  t_NVR_PID PITCH_PID_GAIN_ADR;
  t_NVR_PID LEVELPITCH_PID_GAIN_ADR;
  t_NVR_PID HEADING_PID_GAIN_ADR;
  t_NVR_PID LEVEL_GYRO_ROLL_PID_GAIN_ADR;
  t_NVR_PID LEVEL_GYRO_PITCH_PID_GAIN_ADR;
  t_NVR_PID ALTITUDE_PID_GAIN_ADR;
  t_NVR_PID ZDAMP_PID_GAIN_ADR;
  t_NVR_Receiver RECEIVER_DATA[LASTCHANNEL];
  
  float SOFTWARE_VERSION_ADR;
  float WINDUPGUARD_ADR;
  float XMITFACTOR_ADR;
  float GYROSMOOTH_ADR;
  float ACCSMOOTH_ADR;
  float ACCEL_XAXIS_ZERO_ADR;
  float ACCEL_YAXIS_ZERO_ADR;
  float ACCEL_ZAXIS_ZERO_ADR;
  float FILTERTERM_ADR;
  float HEADINGSMOOTH_ADR;
  float AREF_ADR;
  float FLIGHTMODE_ADR;
  float HEADINGHOLD_ADR;
  float MINACRO_ADR;
  float ACCEL_1G_ADR;
//  float ALTITUDE_PGAIN_ADR;
  float ALTITUDE_MAX_THROTTLE_ADR;
  float ALTITUDE_MIN_THROTTLE_ADR;
  float ALTITUDE_SMOOTH_ADR;
//  float ZDAMP_PGAIN_ADR;
  float ALTITUDE_WINDUP_ADR;
  float SERVOMINPITCH_ADR;
  float SERVOMINROLL_ADR;
  float GYRO_ROLL_ZERO_ADR;
  float GYRO_PITCH_ZERO_ADR;
  float GYRO_YAW_ZERO_ADR;
  // Accel Calibration
  float XAXIS_ACCEL_BIAS_ADR;
  float XAXIS_ACCEL_SCALE_FACTOR_ADR;
  float YAXIS_ACCEL_BIAS_ADR;
  float YAXIS_ACCEL_SCALE_FACTOR_ADR;
  float ZAXIS_ACCEL_BIAS_ADR;
  float ZAXIS_ACCEL_SCALE_FACTOR_ADR;
  // Mag Calibration
  float XAXIS_MAG_BIAS_ADR;
  float YAXIS_MAG_BIAS_ADR;
  float ZAXIS_MAG_BIAS_ADR;
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

// defined in DataStorage.h
void readEEPROM(); 
void initSensorsZeroFromEEPROM();
void storeSensorsZeroToEEPROM();
void initReceiverFromEEPROM();
//////////////////////////////////////////////////////

// defined in FlightCommand.pde
void readPilotCommands(); 
//////////////////////////////////////////////////////

// defined in FlightControl.pde Flight control needs
int motorAxisCommandRoll = 0;
int motorAxisCommandPitch = 0;
int motorAxisCommandYaw = 0;

#if defined quadXConfig || defined quadPlusConfig || defined triConfig || defined quadY4Config
  int motorMaxCommand[4] = {0,0,0,0};
  int motorMinCommand[4] = {0,0,0,0};
  int motorConfiguratorCommand[4] = {0,0,0,0};
#elif defined hexXConfig || defined hexPlusConfig || defined hexY6Config
  int motorMaxCommand[6] = {0,0,0,0,0,0};
  int motorMinCommand[6] = {0,0,0,0,0,0};
  int motorConfiguratorCommand[6] = {0,0,0,0,0,0};
#elif defined (octoX8Congig) || defined (octoXCongig) || defined (octoPlusCongig) 
  int motorMaxCommand[8] = {0,0,0,0,0,0,0,0};
  int motorMinCommand[8] = {0,0,0,0,0,0,0,0};
  int motorConfiguratorCommand[8] = {0,0,0,0,0,0,0,0};
#endif



void calculateFlightError();
void processHeading();
void processAltitudeHold();
void processCalibrateESC();
void processFlightControl();
void processAltitudeHold();
//////////////////////////////////////////////////////

//defined in SerialCom.pde
void readSerialCommand();
void sendSerialTelemetry();
void printInt(int data);
float readFloatSerial();
void sendBinaryFloat(float);
void sendBinaryuslong(unsigned long);
void fastTelemetry();
void comma();
//////////////////////////////////////////////////////

#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
  float findMode(float *data, int arraySize); // defined in Sensors.pde
#else
  int findMode(int *data, int arraySize); // defined in Sensors.pde
#endif

// FUNCTION: return the number of bytes currently free in RAM      
//extern int  __bss_end; // used by freemem 
//extern int  *__brkval; // used by freemem
//int freemem(){
//    int free_memory;
//    if((int)__brkval == 0)
//        free_memory = ((int)&free_memory) - ((int)&__bss_end);
//    else
//        free_memory = ((int)&free_memory) - ((int)__brkval);
//    return free_memory;
//}

