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
//#define AeroQuad_v18        // Arduino 2009 with AeroQuad Shield v1.8 or greater
//#define AeroQuad_Mini       // Arduino Pro Mini with AeroQuad Mini Shield v1.0
//#define AeroQuad_Wii        // Arduino 2009 with Wii Sensors and AeroQuad Shield v1.x
//#define AeroQuad_Paris_v3   // Define along with either AeroQuad_Wii to include specific changes for MultiWiiCopter Paris v3.0 board
//#define AeroQuadMega_v1     // Arduino Mega with AeroQuad Shield v1.7 and below
//#define AeroQuadMega_v2     // Arduino Mega with AeroQuad Shield v2.x
//#define AeroQuadMega_Wii    // Arduino Mega with Wii Sensors and AeroQuad Shield v2.x
//#define ArduCopter          // ArduPilot Mega (APM) with Oilpan Sensor Board
//#define AeroQuadMega_CHR6DM // Clean Arduino Mega with CHR6DM as IMU/heading ref.
//#define APM_OP_CHR6DM       // ArduPilot Mega with CHR6DM as IMU/heading ref., Oilpan for barometer (just uncomment AltitudeHold for baro), and voltage divider
//#define ArduCopter_AQ       //  
#define MapleCopter_CSG

/****************************************************************************
 *********************** Define Flight Configuration ************************
 ****************************************************************************/
// Use only one of the following definitions
#define quadXConfig
//#define quadPlusConfig
//#define hexPlusConfig
//#define hexXConfig
//#define triConfig
//#define quadY4Config
//#define hexY6Config
//#define octoX8Congig

//
// *******************************************************************************************************************************
// Optional Sensors
// Warning:  If you enable HeadingHold or AltitudeHold and do not have the correct sensors connected, the flight software may hang
// *******************************************************************************************************************************
// You must define one of the next 3 attitude stabilization modes or the software will not build
// *******************************************************************************************************************************
#define HeadingMagHold // Enables HMC5843 Magnetometer, gets automatically selected if CHR6DM is defined
//#define AltitudeHold // Enables BMP085 Barometer (experimental, use at your own risk)
#define BattMonitor //define your personal specs in BatteryMonitor.h! Full documentation with schematic there
//#define RateModeOnly // Use this if you only have a gyro sensor, this will disable any attitude modes.
//#define RemotePCReceiver // EXPERIMENTAL Use PC as transmitter via serial communicator with XBEE

//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// You must define *only* one of the following 2 flightAngle calculations
// If you only want DCM, then don't define either of the below
// Use FlightAngleARG if you do not have a magnetometer, use DCM if you have a magnetometer installed
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define FlightAngleMARG // Experimental!  Fly at your own risk! Use this if you have a magnetometer installed and enabled HeadingMagHold above
//#define FlightAngleARG // Use this if you do not have a magnetometer installed
//#define WirelessTelemetry  // Enables Wireless telemetry on Serial3  // Wireless telemetry enable
//#define BinaryWrite // Enables fast binary transfer of flight data to Configurator
//#define BinaryWritePID // Enables fast binary transfer of attitude PID data
//#define OpenlogBinaryWrite // Enables fast binary transfer to serial1 and openlog hardware

// High speed sampled gyro & accel sensor
//#define SENSOR_SAMPLED

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


//#define STATUSMONITOR

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
 * @todo : adapt Alan led class or use it, standardize led processing. Fix dave bug for WII
 *
 * @TODO : REMOVE DRIFT CORRECTION TEST FROM AGR WHEN ALAN AND JOHN HAVE FIX IT!
 */
 
#include "AeroQuad.h"
#include <EEPROM.h>
#include <Wire.h>
#include <Axis.h>
#include "PID.h"
#include <AQMath.h>
#include "filter.h"




//********************************************************
//********************************************************
//********* PLATFORM SPECIFIC SECTION ********************
//********************************************************
//********************************************************

#ifdef MapleCopter_CSG   // STM32
  #define Serial SerialUSB
  #include <Device_I2C.h>
  
  // EEPROM emulation
  #define EEPROM_USES_16BIT_WORDS

  // Gyroscope declaration
  #include <Gyroscope.h>
  #include <Gyroscope_ITG3200.h>
  Gyroscope_ITG3200 gyroSpecific(true);
  Gyroscope *gyro = &gyroSpecific;
  
  // Accelerometer declaration
  #include <Accelerometer.h>
  #include <Accelerometer_BMA180.h>
  Accelerometer_BMA180 accelSpecific(true);
  Accelerometer *accel = &accelSpecific;

  // Receiver Declaration
  #include <Receiver.h>
  #include <Receiver_MapleR5.h>
  Receiver_MapleR5 receiverSpecific;
  Receiver *receiver = &receiverSpecific;
  
  // Motor Declaration
  #include <Motors.h>
  #include <Motors_MapleR5.h>
  Motors_PWM_MapleR5 motorsSpecific(6);
  Motors *motors = &motorsSpecific;
  
  // Magnetometer heading hold declaration
  #define HeadingMagHold
  #include <Magnetometer_HMC5883L.h>
  Magnetometer_HMC5883L compassSpecific;
  Compass *compass = &compassSpecific;

  #include <Kinematics.h>
  #include <Kinematics_MARG.h>
  Kinematics_MARG tempKinematics;
  Kinematics *kinematics = &tempKinematics;
  
  // Altitude declaration
  #ifdef AltitudeHold
    #define BMP085
  #endif

  #undef BattMonitor
  // Battery monitor declaration
  #ifdef BattMonitor
    #include <BatterySensor.h>
    BatterySensor batteryMonitorSpecific;
    BatterySensor* batteryMonitor = &batteryMonitorSpecific;
  #endif

  #include "Copter.h"
  Copter copterSpecific;
  Copter* copter = &copterSpecific; 

  FlightControlX4 flightControlSpecific(motors);
  FlightControl* flight = &flightControlSpecific;


  #include "DataStorage.h"
  Storage storageSpecific;
  Storage* storage = &storageSpecific;

  // Put platform specific intialization need here
  void Copter::initPlatform() {
    Wire.begin( 0, PORTI2C2, I2C_FAST_MODE);
  }
  
  // Measure critical sensors
  void measureCriticalSensors() {
    gyro->measure();
    accel->measure();
  }

#endif


#ifdef ArduCopter_AQ
  #include <APM_RC.h>
  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope.h>
  #include <Gyroscope_ITG3200.h>
  Gyroscope_ITG3200 gyroSpecific;
  Gyroscope *gyro = &gyroSpecific;
  
  // Accelerometer declaration
  #include <Accelerometer.h>
  #include <Accelerometer_BMA180.h>
  Accelerometer_BMA180 accelSpecific;
  Accelerometer *accel = &accelSpecific;

  // Receiver Declaration
  #include <Receiver.h>
  #include <Receiver_APM.h>
  Receiver_APM receiverSpecific;
  Receiver *receiver = &receiverSpecific;
  
  // Motor Declaration
  #include <Motors.h>
  #include <Motors_APM.h>
  Motors_APM motorsSpecific;
  Motors *motors = &motorsSpecific;
  
  // heading mag hold declaration
  #define HeadingMagHold
  #include <Magnetometer_HMC5843.h>
  Magnetometer_HMC5843 compassSpecific;
  Compass *compass = &compassSpecific;

  #include <Kinematics.h>
  #include <Kinematics_MARG.h>
  Kinematics_MARG tempKinematics;
  Kinematics *kinematics = &tempKinematics;

  #undef AltitudeHold 
  // Altitude declaration
  #ifdef AltitudeHold
    #define BMP085
  #endif
  
  // MultiCopter declaration  
  #include "Copter.h"
  Copter copterSpecific;
  Copter* copter = &copterSpecific; 

  FlightControlX4 flightControlSpecific;
  FlightControl* flight = &flightControlSpecific;

  // Battery monitor declaration
  #ifdef BattMonitor
    #include <BatteryMonitor.h>
    #include <BatteryMonitor_APM2.h>
    BatteryMonitor_APM2 batteryMonitorSpecific;
    BatteryMonitor* batteryMonitor = &batteryMonitorSpecific;
  #endif
  


  #include "DataStorage.h"
  Storage storageSpecific;
  Storage* storage = &storageSpecific;

  /**
   * Put ArduCopter specific intialization need here
   */
  void Copter::initPlatform() {
    initRC();
    Wire.begin();
    TWBR = 12;
  }
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    gyro->measure();
    accel->measure();
  }

#endif

#ifdef AeroQuad_v1
  // Gyroscope declaration
  #include <Gyroscope.h>
  #include <Gyroscope_IDG_IDZ500.h>
  Gyroscope_IDG_IDZ500 gyroSpecific;
  Gyroscope *gyro = &gyroSpecific;

  // Accelerometer declaration
  #include <Accelerometer.h>
  #include <Accelerometer_ADXL500.h>
  Accelerometer_ADXL500 accelSpecific;
  Accelerometer *accel = &accelSpecific;
  
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
    gyroSpecific.setAref(aref);
    accelSpecific.setAref(aref);
  }
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    gyro->measure();
    accel->measure();
  }
#endif

#ifdef AeroQuad_v1_IDG
  // Gyroscope declaration
  #include <Gyroscope.h>
  #include <Gyroscope_IDG_IDZ500.h>
  Gyroscope_IDG_IDZ500 gyroSpecific;
  Gyroscope *gyro = &gyroSpecific;

  // Accelerometer declaration
  #include <Accelerometer.h>
  #include <Accelerometer_ADXL500.h>
  Accelerometer_ADXL500 accelSpecific;
  Accelerometer *accel = &accelSpecific;
  
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
    gyroSpecific.setAref(aref);
    accelSpecific.setAref(aref);
  }
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    gyro->measure();
    accel->measure();
  }
#endif

#ifdef AeroQuad_v18
  #include <Device_I2C.h>
  
  // Gyroscope declaration
  #include <Gyroscope.h>
  #include <Gyroscope_ITG3200.h>
  Gyroscope_ITG3200 gyroSpecific;
  Gyroscope *gyro = &gyroSpecific;
  
  // Accelerometer declaraion
  #include <Accelerometer.h>
  #include <Accelerometer_BMA180.h>
  Accelerometer_BMA180 accelSpecific;
  Accelerometer *accel = &accelSpecific;
  
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
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    gyro->measure();
    accel->measure();
  }
#endif

#ifdef AeroQuad_Mini
  #include <Device_I2C.h>
  
  // Gyroscope declaration
  #include <Gyroscope.h>
  #include <Gyroscope_ITG3200.h>
  Gyroscope_ITG3200 gyroSpecific(true);
  Gyroscope *gyro = &gyroSpecific;

  // Accelerometer declaration
  #include <Accelerometer.h>
  #include <Accelerometer_ADXL345.h>
  Accelerometer_ADXL345 accelSpecific;
  Accelerometer *accel = &accelSpecific;
  
  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM_Timer
  
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
    
    pinMode(LED2PIN, OUTPUT);
    digitalWrite(LED2PIN, LOW);
    pinMode(LED3PIN, OUTPUT);
    digitalWrite(LED3PIN, LOW);

    Wire.begin();
    TWBR = 12;
  }
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    gyro->measure();
    accel->measure();
  }
#endif

#ifdef AeroQuadMega_v1
  // Special thanks to Wilafau for fixes for this setup
  // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11466&viewfull=1#post11466
  // Gyroscope declaration
  #include <Gyroscope.h>
  #include <Gyroscope_IDG_IDZ500.h>
  Gyroscope_IDG_IDZ500 gyroSpecific;
  Gyroscope *gyro = &gyroSpecific;

  // Accelerometer declaration
  #include <Accelerometer.h>
  #include <Accelerometer_ADXL500.h>
  Accelerometer_ADXL500 accelSpecific;
  Accelerometer *accel = &accelSpecific;

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
    gyroSpecific.setAref(aref);
    accelSpecific.setAref(aref);
  }
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    gyro->measure();
    accel->measure();
  }
#endif

#ifdef AeroQuadMega_v2
  #include <Device_I2C.h>
  
  // Gyroscope declaration
  #include <Gyroscope.h>
  #include <Gyroscope_ITG3200.h>
  Gyroscope_ITG3200 gyroSpecific;
  Gyroscope *gyro = &gyroSpecific;
  
  // Accelerometer declaration
  #include <Accelerometer.h>
  #include <Accelerometer_BMA180.h>
  Accelerometer_BMA180 accelSpecific;
  Accelerometer *accel = &accelSpecific;

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
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    gyro->measure();
    accel->measure();
  }
#endif

#ifdef ArduCopter
  #include <APM_ADC.h>
  #include <APM_RC.h>
  #include <Device_I2C.h>

  // Gyroscope declaration 
  #include <Gyroscope.h>
  #include <Gyroscope_APM.h>
  Gyroscope_APM gyroSpecific;
  Gyroscope *gyro = &gyroSpecific;
  
  // Accelerometer Declaration
  #include <Accelerometer.h>
  #include <Accelerometer_APM.h>
  Accelerometer_APM accelSpecific;
  Accelerometer *accel = &accelSpecific;

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
    gyro->measure();
    accel->measure();
  }
#endif

#ifdef AeroQuad_Wii
  #include <Device_I2C.h>
  
  // Platform Wii declaration
  #include <Platform_Wii.h>
  Platform_Wii platformWii;
  
  // Gyroscope declaration
  #include <Gyroscope.h>
  #include <Gyroscope_Wii.h>
  Gyroscope_Wii gyroSpecific;
  Gyroscope *gyro = &gyroSpecific;

  // Accelerometer declaration
  #include <Accelerometer.h>
  #include <Accelerometer_WII.h>
  Accelerometer_WII accelSpecific;
  Accelerometer *accel = &accelSpecific;
  
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
     
     gyroSpecific.setPlatformWii(&platformWii);
     accelSpecific.setPlatformWii(&platformWii);
  }
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    platformWii.measure();
    gyro->measure();
    accel->measure();
  }
#endif

#ifdef AeroQuadMega_Wii
  #include <Device_I2C.h>
  
  // Platform Wii declaration
  #include <Platform_Wii.h>
  Platform_Wii platformWii;
  
  // Gyroscope declaration
  #include <Gyroscope.h>
  #include <Gyroscope_Wii.h>
  Gyroscope_Wii gyroSpecific;
  Gyroscope *gyro = &gyroSpecific;

  // Accelerometer declaration
  #include <Accelerometer.h>
  #include <Accelerometer_WII.h>
  Accelerometer_WII accelSpecific;
  Accelerometer *accel = &accelSpecific;

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
    gyroSpecific.setPlatformWii(&platformWii);
    accelSpecific.setPlatformWii(&platformWii);    
  }
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    platformWii.measure();
    gyro->measure();
    accel->measure();
  }
#endif

#ifdef AeroQuadMega_CHR6DM
  #include <Device_I2C.h>
  #include <Platform_CHR6DM.h>
  CHR6DM chr6dm;
  
  // Gyroscope declaration
  #include <Gyroscope.h>
  #include <Gyroscope_CHR6DM.h>
  Gyroscope_CHR6DM gyroSpecific;
  Gyroscope *gyro = &gyroSpecific;

  // Accelerometer declaration
  #include <Accelerometer.h>
  #include <Accelerometer_CHR6DM.h>
  Accelerometer_CHR6DM accelSpecific;
  Accelerometer *accel = &accelSpecific;

  // Receiver declaration
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM
  
  // Kinematics declaration
  #include "Kinematics_CHR6DM.h"
  Kinematics_CHR6DM tempKinematics;
  
  // Compas declaration
  #define HeadingMagHold
  #define COMPASS_CHR6DM
  #include <Magnetometer_CHR6DM.h>
  Magnetometer_CHR6DM compassSpecific;

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
    
    gyroSpecific.setChr6dm(&chr6dm);
    accelSpecific.setChr6dm(&chr6dm);
    tempKinematics.setChr6dm(&chr6dm);
    tempKinematics.setGyroscope(&gyroSpecific);
    compassSpecific.setChr6dm(&chr6dm);
  }
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    
    chr6dm.read();
    gyro->measure();
    accel->measure();
  }
#endif

#ifdef APM_OP_CHR6DM
  #include <Device_I2C.h>
  #include <Platform_CHR6DM.h>
  CHR6DM chr6dm;
  
  // Gyroscope declaration
  #include <Gyroscope.h>
  #include <Gyroscope_CHR6DM.h>
  Gyroscope_CHR6DM gyroSpecific;
  Gyroscope *gyro = &gyroSpecific;
  
  // Accelerometer declaration
  #include <Accelerometer.h>
  #include <Accelerometer_CHR6DM.h>
  Accelerometer_CHR6DM accelSpecific;
  Accelerometer *accel = &accelSpecific;

  // Receiver declaration
  #define RECEIVER_APM

  // Motor declaration
  #define MOTOR_APM
  
  // Kinematics declaration
  #include "Kinematics_CHR6DM.h"
  Kinematics_CHR6DM tempKinematics;
  
  // Compas declaration
  #define HeadingMagHold
  #define COMPASS_CHR6DM
  #include <Magnetometer_CHR6DM.h>
  Magnetometer_CHR6DM compassSpecific;

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
    
    gyroSpecific.setChr6dm(&chr6dm);
    accelSpecific.setChr6dm(&chr6dm);
    tempKinematics.setChr6dm(&chr6dm);
    tempKinematics.setGyroscope(&gyroSpecific);
    compassSpecific.setChr6dm(&chr6dm);
  }
  
  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    
    chr6dm.read();
    gyro->measure();
    accel->measure();
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
/*
#include <Kinematics.h>
#if defined (AeroQuadMega_CHR6DM) || defined (APM_OP_CHR6DM)
  // CHR6DM have it's own kinematics, so, initialize in it's scope
#elif defined FlightAngleARG
  #include <Kinematics_ARG.h>
  Kinematics_ARG tempKinematics;
  Kinematics *kinematics = &tempKinematics;
#elif defined FlightAngleMARG
  #include <Kinematics_MARG.h>
  Kinematics_MARG tempKinematics;
  Kinematics *kinematics = &tempKinematics;
#elif FlightAngleDCM
  #include <Kinematics_DCM.h>
  Kinematics_DCM tempKinematics;
  Kinematics *kinematics = &tempKinematics;
#endif
*/

//********************************************************
//******************** RECEIVER DECLARATION **************
//********************************************************
#if defined (AeroQuad_Mini) && defined (hexY6Config)
  #include <Receiver.h>
  #include <Receiver_PPM.h>
  Receiver_PPM receiverSpecific;
  Receiver *receiver = &receiverSpecific;
#elif defined RemotePCReceiver
  #include <Receiver.h>
  #include <Receiver_RemotePC.h>
  Receiver_RemotePC receiverSpecific;
  Receiver *receiver = &receiverSpecific;
#elif defined RECEIVER_328P
  #include <Receiver.h>
  #include <Receiver_328p.h>
  Receiver_328p receiverSpecific;
  Receiver *receiver = &receiverSpecific;
#elif defined RECEIVER_MEGA
  #include <Receiver.h>
  #include <Receiver_MEGA.h>
  Receiver_MEGA receiverSpecific;
  Receiver *receiver = &receiverSpecific;
#elif defined RECEIVER_APM
  #include <Receiver.h>
  #include <Receiver_APM.h>
  Receiver_APM receiverSpecific;
  Receiver *receiver = &receiverSpecific;
#endif


//********************************************************
//********************** MOTORS DECLARATION **************
//********************************************************
#if defined triConfig 
  #include <Motors.h>
  #include <Motors_PWM_Tri.h>
  Motors_PWM_Tri motorsSpecific;
  Motors *motors = &motorsSpecific;
#elif defined MOTOR_PWM
  #include <Motors.h>
  #include <Motors_PWM.h>
  Motors_PWM motorsSpecific;
  Motors *motors = &motorsSpecific;
#elif defined MOTOR_PWM_Timer
  #include <Motors.h>
  #include <Motors_PWM_Timer.h>
  Motors_PWM_Timer motorsSpecific;
  Motors *motors = &motorsSpecific;
#elif defined MOTOR_APM
  #include <Motors.h>
  #include <Motors_APM.h>
  Motors_APM motorsSpecific;
  Motors *motors = &motorsSpecific;
#elif defined MOTOR_I2C
  #include <Motors.h>
  #include <Motors_I2C.h>
  Motors_I2C motorsSpecific;
  Motors *motors = &motorsSpecific;
#endif

//********************************************************
//******* HEADING HOLD MAGNETOMETER DECLARATION **********
//********************************************************
#if defined (HMC5843)
  #include <Compass.h>
  #include <Magnetometer_HMC5843.h>
  Magnetometer_HMC5843 compassSpecific;
  Compass *compass = &compassSpecific;
#elif defined (COMPASS_CHR6DM)
  #include <Compass.h>
  Compass *compass = &compassSpecific;
#endif

//********************************************************
//******* ALTITUDE HOLD BAROMETER DECLARATION ************
//********************************************************
#if defined (BMP085)
  #include <BarometricSensor.h>
  #include <BarometricSensor_BMP085.h>
  BarometricSensor_BMP085 barometricSensorSpecific;
  BarometricSensor *barometricSensor = &barometricSensorSpecific;
#endif

//********************************************************
//*************** BATTERY MONITOR DECLARATION ************
//********************************************************
#if defined (BATTERY_MONITOR_AQ)
  #include <BatteryMonitor.h>
  #include <BatteryMonitor_AQ.h>
  BatteryMonitor_AQ batteryMonitorSpecific;
    BatteryMonitor* batteryMonitor = &batteryMonitorSpecific;
#elif defined (BATTERY_MONITOR_APM)
  #include <BatteryMonitor.h>
  #include <BatteryMonitor_APM.h>
  BatteryMonitor_APM batteryMonitorSpecific;
  BatteryMonitor* batteryMonitor = &batteryMonitorSpecific;
#endif  


//********************************************************
//************** CAMERA CONTROL DECLARATION **************
//********************************************************
// used only on mega for now
#ifdef CameraControl
  #if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
    #include <CameraStabilizer.h>
    #include <CameraStabilizer_Aeroquad.h>
    CameraStabilizer_AeroQuad tempCameraStabilizer;
    CameraStabilizer *camera = &tempCameraStabilizer;
  #else
    #undef CameraControl
  #endif
#endif


//********************************************************
//******** FLIGHT CONFIGURATION DECLARATION **************
//********************************************************
/*
#if defined quadXConfig
  #include <FlightControlQuadXMode.h>
#elif defined quadPlusConfig
  #include <FlightControlQuadPlusMode.h>
#elif defined hexPlusConfig  
  #include <FlightControlHexPlusMode.h>
#elif defined hexXConfig
  #include <FlightControlHexXMode.h>
#elif defined triConfig
  #include <FlightControlTriMode.h>
#elif defined quadY4Config
  #include <FlightControlQuadY4.h>
#elif defined hexY6Config
  #include <FlightControlHexY6.h>  
#elif defined octoX8Congig
  #include <FlightControlOctoX8.h>
#endif
*/

#ifdef MAX7456_OSD
  #include "OSD.h"
  OSD osd;
#endif

FrameTimer timer;

// Include this last as it contains objects from above declarations
#include "DataStorage.h"

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
#ifdef MapleCopter_CSG  
  Serial.begin();
#else
  Serial.begin(115200);
#endif
  Serial.println("setup:");
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
//  delay(5000);     
  digitalWrite(LEDPIN, HIGH);
 
  // Copter initialize
  copter->initialize();
 
  // Read user values from EEPROM
  //readEEPROM(); // defined in DataStorage.h
  storage->load();
  
  copter->initPlatform();
  
  // Configure motors
  #if defined(quadXConfig) || defined(quadPlusConfig) || defined(quadY4Config)
     motors->initialize(); 
  #elif defined(hexPlusConfig) || defined(hexXConfig) || defined (hexY6Config)
     motors->initialize(6); 
  #elif defined (octoX8Congig)
     motors->initialize(8); 
  #endif

  // Setup receiver pins for pin change interrupts
  receiver->initialize();
  storage->initReceiver();
       
  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  gyro->initialize(); // defined in Gyro.h
  accel->initialize(); // defined in Accel.h
  storage->initSensorsZero();
    
  // Calibrate sensors
  gyro->calibrate(); // defined in Gyro.h
  accel->calibrate();
  zeroIntegralError();

  // Flight angle estimation
  #ifdef HeadingMagHold
    compass->initialize();
    //setHeading = compass->getHeading();
    kinematics->initialize(compass->getHdgXY(XAXIS), compass->getHdgXY(YAXIS));
  #else
    kinematics->initialize(1.0, 0.0);  // with no compass, DCM matrix initalizes to a heading of 0 degrees
  #endif
  
  // Integral Limit for attitude mode
  // This overrides default set in readEEPROM()
  // Set for 1/2 max attitude command (+/-0.75 radians)
  // Rate integral not used for now
  PID[LEVELROLL].windupGuard = 0.375;
  PID[LEVELPITCH].windupGuard = 0.375;

  // Optional Sensors
  #ifdef AltitudeHold
    barometricSensor->initialize();
  #endif
  
  // Battery Monitor
  #ifdef BattMonitor
    batteryMonitor->initialize();
  #endif
  
  // Camera stabilization setup
  #if defined (CameraControl)
    camera->initialize();
    camera->setmCameraRoll(11.11); // Need to figure out nice way to reverse servos
    camera->setCenterRoll(1500); // Need to figure out nice way to set center position
    camera->setmCameraPitch(11.11);
    camera->setCenterPitch(1300);
  #endif
  
  #if defined(MAX7456_OSD)
    osd.initialize();
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
  flight->safetyCheck = OFF;
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
  
  //timer.update();
  
  // High speed sampled sensor
#ifdef SENSOR_SAMPLED  
  measureCriticalSensors();
#endif

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
      
      copter->G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
      hundredHZpreviousTime = currentTime;
      
 #ifndef SENSOR_SAMPLED      
      measureCriticalSensors();
 #endif     
 
      // ****************** Calculate Absolute Angle *****************
      #if defined HeadingMagHold && defined FlightAngleMARG && defined SENSOR_SAMPLED
        kinematics->calculate(gyro->getRadPerSecSample(ROLL),                       
                               gyro->getRadPerSecSample(PITCH),                      
                               gyro->getRadPerSecSample(YAW),                        
                               accel->getMeterPerSecSample(XAXIS),                   
                               accel->getMeterPerSecSample(YAXIS),                   
                               accel->getMeterPerSecSample(ZAXIS),                   
                               compass->getRawData(XAXIS),                      
                               compass->getRawData(YAXIS),                     
                               compass->getRawData(ZAXIS),
                               copter->G_Dt);
      #endif
    
      #if defined HeadingMagHold && defined FlightAngleMARG && !defined SENSOR_SAMPLED
        kinematics->calculate(gyro->getRadPerSec(ROLL),                       
                               gyro->getRadPerSec(PITCH),                      
                               gyro->getRadPerSec(YAW),                        
                               accel->getMeterPerSec(XAXIS),                   
                               accel->getMeterPerSec(YAXIS),                   
                               accel->getMeterPerSec(ZAXIS),                   
                               compass->getRawData(XAXIS),                      
                               compass->getRawData(YAXIS),                     
                               compass->getRawData(ZAXIS),
                               copter->G_Dt);
      #endif
      
      #if defined FlightAngleARG
        kinematics->calculate(gyro->getRadPerSec(ROLL),                       
                               gyro->getRadPerSec(PITCH),                      
                               gyro->getRadPerSec(YAW),                        
                               accel->getMeterPerSec(XAXIS),                   
                               accel->getMeterPerSec(YAXIS),                   
                               accel->getMeterPerSec(ZAXIS),                   
                               0.0,                                            
                               0.0,                                            
                               0.0,
                               copter->G_Dt);
      #endif

      #if defined HeadingMagHold && !defined FlightAngleMARG && !defined FlightAngleARG
        kinematics->calculate(gyro->getRadPerSec(ROLL),                       
                               gyro->getRadPerSec(PITCH),                      
                               gyro->getRadPerSec(YAW),                        
                               accel->getMeterPerSec(XAXIS),                   
                               accel->getMeterPerSec(YAXIS),                   
                               accel->getMeterPerSec(ZAXIS),                   
                               accel->getOneG(),                              
                               compass->getHdgXY(XAXIS),                        
                               compass->getHdgXY(YAXIS),
                               copter->G_Dt);
      #endif

      
      #if !defined HeadingMagHold && !defined FlightAngleMARG && !defined FlightAngleARG
        kinematics->calculate(gyro->getRadPerSec(ROLL),                        
                               gyro->getRadPerSec(PITCH),                       
                               gyro->getRadPerSec(YAW),                         
                               accel->getMeterPerSec(XAXIS),                    
                               accel->getMeterPerSec(YAXIS),                    
                               accel->getMeterPerSec(ZAXIS),                    
                               accel->getOneG(),                               
                               0.0,                                             
                               0.0,
                               copter->G_Dt);  
      #endif
      
      // Combines external pilot commands and measured sensor data to generate motor commands
      //processFlightControl();
      flight->processControl();
      
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
    if (frameCounter % 2 == 0) {  //  50 Hz tasks
      #ifdef DEBUG_LOOP
        digitalWrite(10, HIGH);
      #endif
      
      copter->G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
      fiftyHZpreviousTime = currentTime;
      
      // Reads external pilot commands and performs functions based on stick configuration
      readPilotCommands(); // defined in FlightCommand.pde
      
      #ifdef AltitudeHold
        barometricSensor->measure(); // defined in altitude.h
      #endif

      #ifdef DEBUG_LOOP
        digitalWrite(10, LOW);
      #endif
    }

    // ================================================================
    // 25hz task loop
    // ================================================================
    if (frameCounter % 4 == 0) {  //  25 Hz tasks
      #ifdef DEBUG_LOOP    
        digitalWrite(9, HIGH);
      #endif
      
      copter->G_Dt = (currentTime - twentyFiveHZpreviousTime) / 1000000.0;
      twentyFiveHZpreviousTime = currentTime;
      
      #if defined(AltitudeHold)
        barometricSensor->measure(); // defined in altitude.h
      #endif
      
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
      
      copter->G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
      tenHZpreviousTime = currentTime;


        #if defined(HeadingMagHold)
          compass->measure(kinematics->getData(ROLL), kinematics->getData(PITCH));
        #endif
        #if defined(BattMonitor)
          batteryMonitor->measure(flight->armed);
        #endif
        flight->processAltitudeHold();

      // Listen for configuration commands and reports telemetry

        readSerialCommand(); // defined in SerialCom.pde
        sendSerialTelemetry(); // defined in SerialCom.pde

      #ifdef MAX7456_OSD
        osd.update();
      #endif
      
      #ifdef STATUSMONITOR
      statusSignal->update();
      #endif
      
      digitalWrite(LEDPIN, !digitalRead(LEDPIN));
      
      #ifdef DEBUG_LOOP
        digitalWrite(8, LOW);
      #endif
    }
  
    previousTime = currentTime;
  }

  if (frameCounter >= 1000) {
      frameCounter = 0;
  }      
}


