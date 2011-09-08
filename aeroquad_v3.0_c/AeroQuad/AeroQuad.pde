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

/****************************************************************************
   Before flight, select the different user options for your AeroQuad below
   If you need additional assitance go to http://AeroQuad.com/forum
*****************************************************************************/

/****************************************************************************
 ************************* Hardware Configuration ***************************
 ****************************************************************************/
// Select which hardware you wish to use with the AeroQuad Flight Software

//#define AeroQuad_v1         // Arduino 2009 with AeroQuad Shield v1.7 and below
//#define AeroQuad_v1_IDG     // Arduino 2009 with AeroQuad Shield v1.7 and below using IDG yaw gyro
#define AeroQuad_v18        // Arduino 2009 with AeroQuad Shield v1.8 or greater
//#define AeroQuad_Mini       // Arduino Pro Mini with AeroQuad Mini Shield v1.0
//#define AeroQuad_Wii        // Arduino 2009 with Wii Sensors and AeroQuad Shield v1.x
//#define AeroQuad_Paris_v3   // Define along with either AeroQuad_Wii to include specific changes for MultiWiiCopter Paris v3.0 board
//#define AeroQuadMega_v1     // Arduino Mega with AeroQuad Shield v1.7 and below
//#define AeroQuadMega_v2     // Arduino Mega with AeroQuad Shield v2.x
//#define AeroQuadMega_Wii    // Arduino Mega with Wii Sensors and AeroQuad Shield v2.x
//#define ArduCopter          // ArduPilot Mega (APM) with Oilpan Sensor Board
//#define AeroQuadMega_CHR6DM // Clean Arduino Mega with CHR6DM as IMU/heading ref.
//#define APM_OP_CHR6DM       // ArduPilot Mega with CHR6DM as IMU/heading ref., Oilpan for barometer (just uncomment AltitudeHold for baro), and voltage divider

/****************************************************************************
 *********************** Define Flight Configuration ************************
 ****************************************************************************/
// Use only one of the following definitions
#define quadXConfig
//#define quadPlusConfig
//#define hexPlusConfig
//#define hexXConfig      // not flight tested, take real care
//#define triConfig
//#define quadY4Config
//#define hexY6Config
//#define octoX8Congig
//#define octoPlusCongig  // not yet implemented
//#define octoXCongig

//
// *******************************************************************************************************************************
// Optional Sensors
// Warning:  If you enable HeadingHold or AltitudeHold and do not have the correct sensors connected, the flight software may hang
// *******************************************************************************************************************************
// You must define one of the next 3 attitude stabilization modes or the software will not build
// *******************************************************************************************************************************
//#define HeadingMagHold // Enables HMC5843 Magnetometer, gets automatically selected if CHR6DM is defined
//#define AltitudeHold // Enables BMP085 Barometer (experimental, use at your own risk)
//#define BattMonitor //define your personal specs in BatteryMonitor.h! Full documentation with schematic there
//#define RateModeOnly // Use this if you only have a gyro sensor, this will disable any attitude modes.
//#define RemotePCReceiver // EXPERIMENTAL Use PC as transmitter via serial communicator with XBEE

//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// You must define *only* one of the following 2 flightAngle calculations
// If you only want DCM, then don't define either of the below
// Use FlightAngleARG if you do not have a magnetometer, use DCM if you have a magnetometer installed
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#define FlightAngleMARG // Experimental!  Fly at your own risk! Use this if you have a magnetometer installed and enabled HeadingMagHold above
#define FlightAngleARG // Use this if you do not have a magnetometer installed
//#define WirelessTelemetry  // Enables Wireless telemetry on Serial3  // Wireless telemetry enable
//#define BinaryWrite // Enables fast binary transfer of flight data to Configurator
//#define BinaryWritePID // Enables fast binary transfer of attitude PID data
//#define OpenlogBinaryWrite // Enables fast binary transfer to serial1 and openlog hardware

//
// *******************************************************************************************************************************
// Define how many channels that are connected from your R/C receiver
// Please note that the flight software currently only supports 6 channels, additional channels will be supported in the future
// Additionally 8 receiver channels are only available when not using the Arduino Uno
// *******************************************************************************************************************************
#define LASTCHANNEL 6
//#define LASTCHANNEL 8

//
// *******************************************************************************************************************************
// Camera Stabilization
// Servo output goes to D11(pitch), D12(roll), D13(yaw) on AeroQuad v1.8 shield
// If using v2.0 Shield place jumper between:
// D12 to D33 for roll, connect servo to SERVO1
// D11 to D34 for pitch, connect servo to SERVO2
// D13 to D35 for yaw, connect servo to SERVO3
// Please note that you will need to have battery connected to power on servos with v2.0 shield
// *******************************************************************************************************************************
//#define CameraControl

// On screen display implementation using MAX7456 chip. See OSD.h for more info and configuration.
//#define MAX7456_OSD


#define YAW_DIRECTION 1 // if you want to reverse the yaw correction direction
//#define YAW_DIRECTION -1


/****************************************************************************
 ********************* End of User Definition Section ***********************
 ****************************************************************************/
// Checks to make sure we have the right combinations defined
#if defined(FlightAngleMARG) && !defined(HeadingMagHold)
  #undef FlightAngleMARG
#endif
#if defined(HeadingMagHold) && defined(FlightAngleMARG) && defined(FlightAngleARG)
  #undef FlightAngleARG
#endif
#if defined(MAX7456_OSD) && !defined(AeroQuadMega_v2) && !defined(AeroQuadMega_Wii) && !defined(AeroQuadMega_CHR6DM)
  #undef MAX7456_OSD
#endif  

//#define DEBUG_LOOP


/**
 * Kenny todo.
 * @todo : UNIT TEST
 *
 * @TODO : REMOVE DRIFT CORRECTION TEST FROM AGR WHEN ALAN AND JOHN HAVE FIX IT!
 */
 
#include <EEPROM.h>
#include <Wire.h>
#include "AeroQuad.h"
#include <Axis.h>
#include "PID.h"
#include <AQMath.h>


//********************************************************
//********************************************************
//********* PLATFORM SPECIFIC SECTION ********************
//********************************************************
//********************************************************
#ifdef AeroQuad_v1
  // Gyroscope declaration
  #include <Gyroscope_IDG_IDZ500.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL500.h>
  
  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM
  
  // unsuported in v1
  #undef AltitudeHold
  #undef HeadingMagHold
  #undef BattMonitor
  
  /**
   * Put AeroQuad_v1 specific intialization need here
   */
  void initPlatform() {
    setGyroAref(aref);
    setAccelAref(aref);
  }
  
  #define DELAY_BETWEEN_READING 2000
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if ((currentTime - lastSampleTime) > DELAY_BETWEEN_READING) {
      lastSampleTime = currentTime;
      measureGyro();
      measureAccel();
      for (int i = 0; i < LASTAXIS;i++) {
        accelSample[i] += meterPerSec[i];
        gyroSample[i] += gyroRate[i];
      }
      sampleCount++;
    }
  }
#endif

#ifdef AeroQuad_v1_IDG
  // Gyroscope declaration
  #include <Gyroscope_IDG_IDZ500.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL500.h>
  
  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM  
  
  // unsuported in v1
  #undef AltitudeHold
  #undef HeadingMagHold
  #undef BattMonitor
  
  /**
   * Put AeroQuad_v1_IDG specific intialization need here
   */
  void initPlatform() {
    setGyroAref(aref);
    setAccelAref(aref);
  }
  
  #define DELAY_BETWEEN_READING 2000
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if ((currentTime - lastSampleTime) > DELAY_BETWEEN_READING) {
      lastSampleTime = currentTime;
      measureGyro();
      measureAccel();
      for (int i = 0; i < LASTAXIS;i++) {
        accelSample[i] += meterPerSec[i];
        gyroSample[i] += gyroRate[i];
      }
      sampleCount++;
    }
  }
#endif

#ifdef AeroQuad_v18
  #include <Device_I2C.h>
  
  // Gyroscope declaration
  #include <Gyroscope_ITG3200.h>
  
  // Accelerometer declaraion
  #include <Accelerometer_BMA180.h>
  
  // Receiver declaration
  #define RECEIVER_328P
  
  // Motor declaration
  #define MOTOR_PWM_Timer
  
  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif
  
  // Altitude declaration
  #ifdef AltitudeHold
    #define BMP085
  #endif
  
  // Battery Monitor declaration
  #ifdef BattMonitor
    #define BATTERY_MONITOR_AQ
  #endif
  
  /**
   * Put AeroQuad_v18 specific intialization need here
   */
  void initPlatform() {

    pinMode(LED2PIN, OUTPUT);
    digitalWrite(LED2PIN, LOW);
    pinMode(LED3PIN, OUTPUT);
    digitalWrite(LED3PIN, LOW);
    
    Wire.begin();
    TWBR = 12;
  }
  
  #define DELAY_BETWEEN_READING 1000
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if ((currentTime - lastSampleTime) > DELAY_BETWEEN_READING) {
      lastSampleTime = currentTime;
      measureGyro();
      measureAccel();
      for (int i = 0; i < LASTAXIS;i++) {
        accelSample[i] += meterPerSec[i];
        gyroSample[i] += gyroRate[i];
      }
      sampleCount++;
    }
  }
#endif

#ifdef AeroQuad_Mini
  #include <Device_I2C.h>
  
  // Gyroscope declaration
  #include <Gyroscope_ITG3200.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL345.h>
  
  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM
  
  // Battery Monitor declaration
  #ifdef BattMonitor
    #define BATTERY_MONITOR_AQ
  #endif
  
  // unsuported in mini
  #undef AltitudeHold
  #undef HeadingMagHold
  
  /**
   * Put AeroQuad_Mini specific intialization need here
   */
  void initPlatform() {
    gyroAddress = ITG3200_ADDRESS-1;
    
    pinMode(LED2PIN, OUTPUT);
    digitalWrite(LED2PIN, LOW);
    pinMode(LED3PIN, OUTPUT);
    digitalWrite(LED3PIN, LOW);

    Wire.begin();
    TWBR = 12;
  }
  
  #define DELAY_BETWEEN_READING 1000
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if ((currentTime - lastSampleTime) > DELAY_BETWEEN_READING) {
      lastSampleTime = currentTime;
      measureGyro();
      measureAccel();
      for (int i = 0; i < LASTAXIS;i++) {
        accelSample[i] += meterPerSec[i];
        gyroSample[i] += gyroRate[i];
      }
      sampleCount++;
    }
  }
#endif

#ifdef AeroQuadMega_v1
  // Special thanks to Wilafau for fixes for this setup
  // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11466&viewfull=1#post11466
  // Gyroscope declaration
  #include <Gyroscope_IDG_IDZ500.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL500.h>

  // Reveiver declaration
  #define RECEIVER_MEGA
  
  // Motor declaration
  #define MOTOR_PWM  

  // unsuported on mega v1  
  #undef AltitudeHold
  #undef HeadingMagHold
  #undef BattMonitor
  
  /**
   * Put AeroQuadMega_v1 specific intialization need here
   */
  void initPlatform() {
    setGyroAref(aref);
    setAccelAref(aref);
  }
  
  #define DELAY_BETWEEN_READING 2000
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if ((currentTime - lastSampleTime) > DELAY_BETWEEN_READING) {
      lastSampleTime = currentTime;
      measureGyro();
      measureAccel();
      for (int i = 0; i < LASTAXIS;i++) {
        accelSample[i] += meterPerSec[i];
        gyroSample[i] += gyroRate[i];
      }
      sampleCount++;
    }
  }
#endif

#ifdef AeroQuadMega_v2
  #include <Device_I2C.h>
  
  // Gyroscope declaration
  #include <Gyroscope_ITG3200.h>
  
  // Accelerometer declaration
  #include <Accelerometer_BMA180.h>

  // Receiver Declaration
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM_Timer
  
  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif

  // Altitude declaration
  #ifdef AltitudeHold
    #define BMP085
  #endif

  // Battery Monitor declaration
  #ifdef BattMonitor
    #define BATTERY_MONITOR_AQ
  #endif


  /**
   * Put AeroQuadMega_v2 specific intialization need here
   */
  void initPlatform() {
    
    pinMode(LED2PIN, OUTPUT);
    digitalWrite(LED2PIN, LOW);
    pinMode(LED3PIN, OUTPUT);
    digitalWrite(LED3PIN, LOW);

    // pins set to INPUT for camera stabilization so won't interfere with new camera class
    pinMode(33, INPUT); // disable SERVO 1, jumper D12 for roll
    pinMode(34, INPUT); // disable SERVO 2, jumper D11 for pitch
    pinMode(35, INPUT); // disable SERVO 3, jumper D13 for yaw
    pinMode(43, OUTPUT); // LED 1
    pinMode(44, OUTPUT); // LED 2
    pinMode(45, OUTPUT); // LED 3
    pinMode(46, OUTPUT); // LED 4
    digitalWrite(43, HIGH); // LED 1 on
    digitalWrite(44, HIGH); // LED 2 on
    digitalWrite(45, HIGH); // LED 3 on
    digitalWrite(46, HIGH); // LED 4 on  

    Wire.begin();
    TWBR = 12;
  }
  
  #define DELAY_BETWEEN_READING 1000
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if ((currentTime - lastSampleTime) > DELAY_BETWEEN_READING) {
      lastSampleTime = currentTime;
      measureGyro();
      measureAccel();
      for (int i = 0; i < LASTAXIS;i++) {
        accelSample[i] += meterPerSec[i];
        gyroSample[i] += gyroRate[i];
      }
      sampleCount++;
    }
  }
#endif

#ifdef ArduCopter
  #include <APM_ADC.h>
  #include <APM_RC.h>
  #include <Device_I2C.h>

  // Gyroscope declaration 
  #include <Gyroscope_APM.h>
  
  // Accelerometer Declaration
  #include <Accelerometer_APM.h>

  // Receiver Declaration
  #define RECEIVER_APM
  
  // Motor Declaration
  #define MOTOR_APM
  
  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif
 
  // Altitude declaration
  #ifdef AltitudeHold
    #define BMP085
  #endif
  
  // Battery monitor declaration
  #ifdef BattMonitor
    #define BATTERY_MONITOR_APM
  #endif
  
  /**
   * Put ArduCopter specific intialization need here
   */
  void initPlatform() {
    pinMode(LED_Red, OUTPUT);
    pinMode(LED_Yellow, OUTPUT);
    pinMode(LED_Green, OUTPUT);

    initializeADC();
    initRC();
    
    Wire.begin();
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      lastSampleTime = currentTime;
      measureGyro();
      measureAccel();
      for (int i = 0; i < LASTAXIS;i++) {
        accelSample[i] += meterPerSec[i];
        gyroSample[i] += gyroRate[i];
      }
      sampleCount++;
    }
  }
#endif

#ifdef AeroQuad_Wii
  #include <Device_I2C.h>
  
  // Platform Wii declaration
  #include <Platform_Wii.h>
  Platform_Wii platformWii;
  
  // Gyroscope declaration
  #include <Gyroscope_Wii.h>

  // Accelerometer declaration
  #include <Accelerometer_WII.h>
  
  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM  
  
  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif
  
  // Altitude declaration
  #ifdef AltitudeHold
    #define BMP085
  #endif
  
  // Battery monitor declaration
  #ifdef BattMonitor
    #define BATTERY_MONITOR_AQ
  #endif
  
  /**
   * Put AeroQuad_Wii specific intialization need here
   */
  void initPlatform() {
     Wire.begin();
     
     #if defined(AeroQuad_Paris_v3)
       platformWii.initialize(true);
     #else
       platformWii.initialize();
     #endif  
     
     gyroPlatformWii = &platformWii;
     accelPlatformWii = &platformWii;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      platformWii.measure();
      lastSampleTime = currentTime;
      measureGyro();
      measureAccel();
      for (int i = 0; i < LASTAXIS;i++) {
        accelSample[i] += meterPerSec[i];
        gyroSample[i] += gyroRate[i];
      }
      sampleCount++;
    }
  }
#endif

#ifdef AeroQuadMega_Wii
  #include <Device_I2C.h>
  
  // Platform Wii declaration
  #include <Platform_Wii.h>
  Platform_Wii platformWii;
  
  // Gyroscope declaration
  #include <Gyroscope_Wii.h>

  // Accelerometer declaration
  #include <Accelerometer_WII.h>

  // Receiver declaration
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM
  
  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif
  
  // Altitude declaration
  #ifdef AltitudeHold
    #define BMP085
  #endif
  
  // Battery monitor declaration
  #ifdef BattMonitor
    #define BATTERY_MONITOR_AQ
  #endif

  /**
   * Put AeroQuadMega_Wii specific intialization need here
   */
  void initPlatform() {
    Wire.begin();
    
    platformWii.initialize();
    gyroPlatformWii = &platformWii;
    accelPlatformWii = &platformWii;    
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      platformWii.measure();
      lastSampleTime = currentTime;
      measureGyro();
      measureAccel();
      for (int i = 0; i < LASTAXIS;i++) {
        accelSample[i] += meterPerSec[i];
        gyroSample[i] += gyroRate[i];
      }
      sampleCount++;
    }
  }
#endif

#ifdef AeroQuadMega_CHR6DM
  #include <Device_I2C.h>
  #include <Platform_CHR6DM.h>
  CHR6DM chr6dm;
  
  // Gyroscope declaration
  #include <Gyroscope_CHR6DM.h>

  // Accelerometer declaration
  #include <Accelerometer_CHR6DM.h>

  // Receiver declaration
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM
  
  // Kinematics declaration
  #include "Kinematics_CHR6DM.h"
  
  // Compas declaration
  #define HeadingMagHold
  #define COMPASS_CHR6DM
  #include <Magnetometer_CHR6DM.h>

  // Altitude declaration
  #ifdef AltitudeHold
    #define BMP085
  #endif
  
  // Battery monitor declaration
  #ifdef BattMonitor
    #define BATTERY_MONITOR_APM
  #endif
  
  /**
   * Put AeroQuadMega_CHR6DM specific intialization need here
   */
  void initPlatform() {
    Serial1.begin(BAUD);
    PORTD = B00000100;

    Wire.begin();
    
    chr6dm.resetToFactory();
    chr6dm.setListenMode();
    chr6dm.setActiveChannels(CHANNEL_ALL_MASK);
    chr6dm.requestPacket();
    
    gyroChr6dm = &chr6dm;
    accelChr6dm = &chr6dm;
    kinematicsChr6dm = &chr6dm;
//    tempKinematics.setGyroscope(&gyroSpecific);
    compassChr6dm = &chr6dm;
  }
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      chr6dm.read();
      lastSampleTime = currentTime;
      measureGyro();
      measureAccel();
      for (int i = 0; i < LASTAXIS;i++) {
        accelSample[i] += meterPerSec[i];
        gyroSample[i] += gyroRate[i];
      }
      sampleCount++;
    }
  }
#endif

#ifdef APM_OP_CHR6DM
  #include <Device_I2C.h>
  #include <Platform_CHR6DM.h>
  CHR6DM chr6dm;
  
  // Gyroscope declaration
  #include <Gyroscope_CHR6DM.h>
  
  // Accelerometer declaration
  #include <Accelerometer_CHR6DM.h>

  // Receiver declaration
  #define RECEIVER_APM

  // Motor declaration
  #define MOTOR_APM
  
  // Kinematics declaration
  #include "Kinematics_CHR6DM.h"
  
  // Compas declaration
  #define HeadingMagHold
  #define COMPASS_CHR6DM
  #include <Magnetometer_CHR6DM.h>

  // Altitude declaration
  #ifdef AltitudeHold
    #define BMP085
  #endif
  
  // Battery monitor declaration
  #ifdef BattMonitor
    #define BATTERY_MONITOR_APM
  #endif
  
  /**
   * Put APM_OP_CHR6DM specific intialization need here
   */
  void initPlatform() {
    pinMode(LED_Red, OUTPUT);
    pinMode(LED_Yellow, OUTPUT);
    pinMode(LED_Green, OUTPUT);

    Serial1.begin(BAUD);
    PORTD = B00000100;
    
    Wire.begin();

    chr6dm.resetToFactory();
    chr6dm.setListenMode();
    chr6dm.setActiveChannels(CHANNEL_ALL_MASK);
    chr6dm.requestPacket();
    
    gyroChr6dm = &chr6dm;
    accelChr6dm = &chr6dm;
    kinematicsChr6dm = &chr6dm;
//    tempKinematics.setGyroscope(&gyroSpecific);
    compassChr6dm = &chr6dm;
  }
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      chr6dm.read();
      lastSampleTime = currentTime;
      measureGyro();
      measureAccel();
      for (int i = 0; i < LASTAXIS;i++) {
        accelSample[i] += meterPerSec[i];
        gyroSample[i] += gyroRate[i];
      }
      sampleCount++;
    }
  }sureAccel();
  }
#endif

//********************************************************
//********************************************************
//********* HARDWARE GENERALIZATION SECTION **************
//********************************************************
//********************************************************

//********************************************************
//****************** KINEMATICS DECLARATION **************
//********************************************************
#include "Kinematics.h"
#if defined (AeroQuadMega_CHR6DM) || defined (APM_OP_CHR6DM)
  // CHR6DM have it's own kinematics, so, initialize in it's scope
#elif defined FlightAngleARG
  #include "Kinematics_ARG.h"
#elif defined FlightAngleMARG
  #include "Kinematics_MARG.h"
#else
  #include "Kinematics_DCM.h"
#endif

//********************************************************
//******************** RECEIVER DECLARATION **************
//********************************************************
#if defined (AeroQuad_Mini) && (defined (hexPlusConfig) || defined (hexXConfig) || defined (hexY6Config))
  #include <Receiver_PPM.h>
#elif defined RemotePCReceiver
  #include <Receiver_RemotePC.h>
#elif defined RECEIVER_328P
  #include <Receiver_328p.h>
#elif defined RECEIVER_MEGA
  #include <Receiver_MEGA.h>
#elif defined RECEIVER_APM
  #include <Receiver_APM.h>
#endif


//********************************************************
//********************** MOTORS DECLARATION **************
//********************************************************
//#if defined triConfig 
//  #include <Motors_PWM_Tri.h>
#if defined MOTOR_PWM
  #include <Motors_PWM.h>
#elif defined MOTOR_PWM_Timer
  #include <Motors_PWM_Timer.h>
#elif defined MOTOR_APM
  #include <Motors_APM.h>
#elif defined MOTOR_I2C
  #include <Motors_I2C.h>
#endif

//********************************************************
//******* HEADING HOLD MAGNETOMETER DECLARATION **********
//********************************************************
#if defined (HMC5843)
  #include <Magnetometer_HMC5843.h>
#elif defined (COMPASS_CHR6DM)
#endif

//********************************************************
//******* ALTITUDE HOLD BAROMETER DECLARATION ************
//********************************************************
#if defined (BMP085)
  #include <BarometricSensor_BMP085.h>
#endif

//********************************************************
//*************** BATTERY MONITOR DECLARATION ************
//********************************************************
#if defined (BATTERY_MONITOR_AQ)
  #include <BatteryMonitor_AQ.h>
#elif defined (BATTERY_MONITOR_APM)
  #include <BatteryMonitor_APM.h>
#endif  

//********************************************************
//************** CAMERA CONTROL DECLARATION **************
//********************************************************
// used only on mega for now
#ifdef CameraControl
  #if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
    #include <CameraStabilizer_Aeroquad.h>
  #else
    #undef CameraControl
  #endif
#endif


//********************************************************
//******** FLIGHT CONFIGURATION DECLARATION **************
//********************************************************
#if defined quadXConfig
  #include "FlightControlQuadXMode.h"
#elif defined quadPlusConfig
  #include "FlightControlQuadPlusMode.h"
#elif defined hexPlusConfig  
  #include "FlightControlHexPlusMode.h"
#elif defined hexXConfig
  #include "FlightControlHexXMode.h"
#elif defined triConfig
  #include "FlightControlTriMode.h"
#elif defined quadY4Config
  #include "FlightControlQuadY4.h"
#elif defined hexY6Config
  #include "FlightControlHexY6.h"  
#elif defined octoX8Congig
  #include "FlightControlOctoX8.h"
#elif defined octoXCongig
  #include "FlightControlOctoX.h"
#elif defined octoPlusCongig
  #include "FlightControlOctoPlus.h"
#endif

//********************************************************
//****************** OSD DEVICE DECLARATION **************
//********************************************************
#ifdef MAX7456_OSD
  #include "MAX7456.h"
#endif


// Include this last as it contains objects from above declarations
#include "DataStorage.h"

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
  SERIAL_BEGIN(BAUD);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);

#ifdef DEBUG_LOOP
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(12, LOW);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
  digitalWrite(9, LOW);
  digitalWrite(8, LOW);
#endif    

  // Read user values from EEPROM
  readEEPROM(); // defined in DataStorage.h
  
  initPlatform();
  
  // Configure motors
  #if defined(quadXConfig) || defined(quadPlusConfig) || defined(quadY4Config)
     initializeMotors(); 
  #elif defined(hexPlusConfig) || defined(hexXConfig) || defined (hexY6Config)
     initializeMotors(SIX_Motors); 
  #elif defined (octoX8Congig) || defined (octoXCongig) || defined (octoXPlusCongig)
     initializeMotors(HEIGHT_Motors); 
  #endif


  // Setup receiver pins for pin change interrupts
  if (receiverLoop == ON) {
    initializeReceiver(LASTCHANNEL);
    initReceiverFromEEPROM();
  }
       
  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  initSensorsZeroFromEEPROM();
  initializeGyro(); // defined in Gyro.h
  initializeAccel(); // defined in Accel.h
  
  // Calibrate sensors
  calibrateGyro(); // defined in Gyro.h
  zeroIntegralError();
  levelAdjust[ROLL] = 0;
  levelAdjust[PITCH] = 0;
  
  // Flight angle estimation
  #ifdef HeadingMagHold
    initializeMagnetometer();
    //setHeading = compass->getHeading();
    initializeKinematics(getHdgXY(XAXIS), getHdgXY(YAXIS));
  #else
    initializeKinematics(1.0, 0.0);  // with no compass, DCM matrix initalizes to a heading of 0 degrees
  #endif
  // Integral Limit for attitude mode
  // This overrides default set in readEEPROM()
  // Set for 1/2 max attitude command (+/-0.75 radians)
  // Rate integral not used for now
  PID[LEVELROLL].windupGuard = 0.375;
  PID[LEVELPITCH].windupGuard = 0.375;

  // Optional Sensors
  #ifdef AltitudeHold
    initializeBaro();
  #endif
  
  // Battery Monitor
  #ifdef BattMonitor
    initializeBatteryMonitor();
  #endif
  
  // Camera stabilization setup
  #if defined (CameraControl)
    initializeCameraStabilization();
    setmCameraRoll(11.11); // Need to figure out nice way to reverse servos
    setCenterRoll(1500); // Need to figure out nice way to set center position
    setmCameraPitch(11.11);
    setCenterPitch(1300);
  #endif
  
  #if defined(MAX7456_OSD)
    initializeOSD();
  #endif


  #if defined(BinaryWrite) || defined(BinaryWritePID)
    #ifdef OpenlogBinaryWrite
      binaryPort = &Serial1;
      binaryPort->begin(115200);
      delay(1000);
    #else
     binaryPort = &Serial;
    #endif 
  #endif
  
  // AKA use a new low pass filter called a Lag Filter uncomment only if using DCM LAG filters
  //  setupFilters(accel.accelOneG);

  previousTime = micros();
  digitalWrite(LEDPIN, HIGH);
  safetyCheck = 0;
}

/*******************************************************************
  // tasks (microseconds of interval)
  ReadGyro        readGyro      (   5000); // 200hz
  ReadAccel       readAccel     (   5000); // 200hz
  RunDCM          runDCM        (  10000); // 100hz
  FlightControls  flightControls(  10000); // 100hz
  ReadReceiver    readReceiver  (  20000); //  50hz
  ReadBaro        readBaro      (  40000); //  25hz
  ReadCompass     readCompass   ( 100000); //  10Hz
  ProcessTelem    processTelem  ( 100000); //  10Hz
  ReadBattery     readBattery   ( 100000); //  10Hz
  
  Task *tasks[] = {&readGyro, &readAccel, &runDCM, &flightControls,   \
                   &readReceiver, &readBaro, &readCompass,            \
                   &processTelem, &readBattery};
                   
  TaskScheduler sched(tasks, NUM_TASKS(tasks));
  
  sched.run();
*******************************************************************/
void loop () {
  currentTime = micros();
  deltaTime = currentTime - previousTime;
  
  if (sensorLoop == ON) {
    measureCriticalSensors();
  }
  // Main scheduler loop set for 100hz
  if (deltaTime >= 10000) {

    #ifdef DEBUG_LOOP
      testSignal ^= HIGH;
      digitalWrite(LEDPIN, testSignal);
    #endif

    frameCounter++;
    
    // ================================================================
    // 100hz task loop
    // ================================================================
    if (frameCounter %   1 == 0) {  //  100 Hz tasks
      #ifdef DEBUG_LOOP
        digitalWrite(11, HIGH);
      #endif
      
      G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
      hundredHZpreviousTime = currentTime;
      
      if (sensorLoop == ON) {
        // ****************** Calculate Absolute Angle *****************
        #if defined HeadingMagHold && defined FlightAngleMARG
          calculateKinematics(gyroSample[XAXIS]/sampleCount,                  
                              gyroSample[YAXIS]/sampleCount,                      
                              gyroSample[ZAXIS]/sampleCount,                        
                              accelSample[XAXIS]/sampleCount,                  
                              accelSample[YAXIS]/sampleCount,                  
                              accelSample[ZAXIS]/sampleCount,                  
                              getMagnetometerRawData(XAXIS),                      
                              getMagnetometerRawData(YAXIS),                     
                              getMagnetometerRawData(ZAXIS),
                              G_Dt);
        #endif
      
        #if defined FlightAngleARG
          calculateKinematics(gyroSample[XAXIS]/sampleCount,                  
                              gyroSample[YAXIS]/sampleCount,                      
                              gyroSample[ZAXIS]/sampleCount,                        
                              accelSample[XAXIS]/sampleCount,                  
                              accelSample[YAXIS]/sampleCount,                  
                              accelSample[ZAXIS]/sampleCount,                  
                              0.0,                                            
                              0.0,                                            
                              0.0,
                              G_Dt);
        #endif

        #if defined HeadingMagHold && !defined FlightAngleMARG && !defined FlightAngleARG
          calculateKinematics(gyroSample[XAXIS]/sampleCount,                  
                              gyroSample[YAXIS]/sampleCount,                      
                              gyroSample[ZAXIS]/sampleCount,                        
                              accelSample[XAXIS]/sampleCount,                  
                              accelSample[YAXIS]/sampleCount,                  
                              accelSample[ZAXIS]/sampleCount,                   
                              accelOneG,                              
                              getHdgXY(XAXIS),                        
                              getHdgXY(YAXIS),
                              G_Dt);
        #endif
        
        #if !defined HeadingMagHold && !defined FlightAngleMARG && !defined FlightAngleARG
          calculateKinematics(gyroSample[XAXIS]/sampleCount,                  
                              gyroSample[YAXIS]/sampleCount,                      
                              gyroSample[ZAXIS]/sampleCount,                        
                              accelSample[XAXIS]/sampleCount,                  
                              accelSample[YAXIS]/sampleCount,                  
                              accelSample[ZAXIS]/sampleCount,                   
                              accel->getOneG(),                               
                              0.0,                                             
                              0.0,
                              G_Dt);
        #endif
        for (int i = 0; i < LASTAXIS;i++) {
          accelSample[i] = 0.0;
          gyroSample[i] = 0.0;
        }
        sampleCount = 0.0;
      }
      
      // Combines external pilot commands and measured sensor data to generate motor commands
      if (controlLoop == ON) {
        processFlightControl();
      } 

      #ifdef BinaryWrite
        if (fastTransfer == ON) {
          // write out fastTelemetry to Configurator or openLog
          fastTelemetry();
        }
      #endif      
      
      #ifdef DEBUG_LOOP
        digitalWrite(11, LOW);
      #endif
    }

    // ================================================================
    // 50hz task loop
    // ================================================================
    if (frameCounter %   2 == 0) {  //  50 Hz tasks
      #ifdef DEBUG_LOOP
        digitalWrite(10, HIGH);
      #endif
      
      G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
      fiftyHZpreviousTime = currentTime;
      
      // Reads external pilot commands and performs functions based on stick configuration
      if (receiverLoop == ON) { 
        readPilotCommands(); // defined in FlightCommand.pde
      }
      
      #ifdef AltitudeHold
        if (sensorLoop == ON) {
          measureBaro(); // defined in altitude.h
        }
      #endif

      #ifdef DEBUG_LOOP
        digitalWrite(10, LOW);
      #endif
    }

    // ================================================================
    // 25hz task loop
    // ================================================================
    if (frameCounter %   4 == 0) {  //  25 Hz tasks
      #ifdef DEBUG_LOOP    
        digitalWrite(9, HIGH);
      #endif
      
      G_Dt = (currentTime - twentyFiveHZpreviousTime) / 1000000.0;
      twentyFiveHZpreviousTime = currentTime;
      
      if (sensorLoop == ON) {
        #if defined(AltitudeHold)
          measureBaro(); // defined in altitude.h
        #endif
      }
      
      #ifdef DEBUG_LOOP
        digitalWrite(9, LOW);
      #endif
    }
    
    // ================================================================
    // 10hz task loop
    // ================================================================
    if (frameCounter %  10 == 0) {  //   10 Hz tasks
      #ifdef DEBUG_LOOP
        digitalWrite(8, HIGH);
      #endif
      
      G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
      tenHZpreviousTime = currentTime;

      if (sensorLoop == ON) {
        #if defined(HeadingMagHold)
          measureMagnetometer(kinematicsAngle[ROLL], kinematicsAngle[PITCH]);
        #endif
        #if defined(BattMonitor)
          measureBatteryVoltage(armed);
        #endif
        processAltitudeHold();
      }
      // Listen for configuration commands and reports telemetry
      if (telemetryLoop == ON) {
        readSerialCommand(); // defined in SerialCom.pde
        sendSerialTelemetry(); // defined in SerialCom.pde
      }
      
      #ifdef MAX7456_OSD
        updateOSD();
      #endif

      
      #ifdef DEBUG_LOOP
        digitalWrite(8, LOW);
      #endif
    }

    previousTime = currentTime;
  }
  if (frameCounter >= 100) {
      frameCounter = 0;
  }
}


