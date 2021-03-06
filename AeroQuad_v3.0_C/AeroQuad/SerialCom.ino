/*
  AeroQuad v3.0 - May 2011
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

// SerialCom.pde is responsible for the serial communication for commands and telemetry from the AeroQuad
// This comtains readSerialCommand() which listens for a serial command and it's arguments
// This also contains readSerialTelemetry() which listens for a telemetry request and responds with the requested data
// For more information on each command/telemetry look at: http://aeroquad.com/content.php?117

// Includes re-write / fixes from Aadamson and ala42, special thanks to those guys!
// http://aeroquad.com/showthread.php?1461-We-have-some-hidden-warnings&p=14618&viewfull=1#post14618

#ifndef _AQ_SERIAL_COMM_
#define _AQ_SERIAL_COMM_


//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************
bool validateCalibrateCommand(byte command)
{
  if (readFloatSerial() == 123.45) {// use a specific float value to validate full throttle call is being sent
    armed = OFF;
    calibrateESC = command;
    return true;
  } else {
    calibrateESC = 0;
    testCommand = 1000;
    return false;
  }
}

void readSerialPID(unsigned char PIDid) {
  struct PIDdata* pid = &PID[PIDid];
  pid->P = readFloatSerial();
  pid->I = readFloatSerial();
  pid->D = readFloatSerial();
  pid->lastPosition = 0;
  pid->integratedError = 0;
}

void readSerialCommand() {
  // Check for serial message
  if (SERIAL_AVAILABLE()) {
    queryType = SERIAL_READ();
    switch (queryType) {
    case 'A': // Receive roll and pitch rate mode PID
      readSerialPID(ROLL);
      readSerialPID(PITCH);
      break;
    case 'B': // Receive roll/pitch attitude mode PID
      readSerialPID(LEVELROLL);
      readSerialPID(LEVELPITCH);
      readSerialPID(LEVELGYROROLL);
      readSerialPID(LEVELGYROPITCH);
      windupGuard = readFloatSerial(); // defaults found in setup() of AeroQuad.pde
      break;
    case 'C': // Receive yaw PID
      readSerialPID(YAW);
      readSerialPID(HEADING);
      headingHoldConfig = readFloatSerial();
      heading = 0;
      relativeHeading = 0;
      headingHold = 0;
      break;
    case 'D': // Altitude hold PID
      #ifdef AltitudeHold
        readSerialPID(ALTITUDE);
        PID[ALTITUDE].windupGuard = readFloatSerial();
        altitudeHoldBump = readFloatSerial();
        altitudeHoldPanicStickMovement = readFloatSerial();
        minThrottleAdjust = readFloatSerial();
        maxThrottleAdjust = readFloatSerial();
        baroSmoothFactor = readFloatSerial();
        readSerialPID(ZDAMPENING);
      #endif
      break;
    case 'E': // Receive sensor filtering values
      gyroSmoothFactor = readFloatSerial();
      accelSmoothFactor = readFloatSerial();
      timeConstant = readFloatSerial();
      aref = readFloatSerial();
      break;
    case 'F': // Receive transmitter smoothing values
      receiverXmitFactor = readFloatSerial();
      for(byte channel = ROLL; channel<LASTCHANNEL; channel++) {
        receiverSmoothFactor[channel] = readFloatSerial();
      }
      break;
    case 'G': // Receive transmitter calibration values
      for(byte channel = ROLL; channel<LASTCHANNEL; channel++) {
        receiverSlope[channel] = readFloatSerial();
        receiverOffset[channel] = readFloatSerial();
      }
      break;
    case 'H': //  read Camera values
      #ifdef Camera
        camera.setMode(readFloatSerial());
        camera.setCenterPitch(readFloatSerial());
        camera.setCenterRoll(readFloatSerial());
        camera.setCenterYaw(readFloatSerial());
        camera.setmCameraPitch(readFloatSerial());
        camera.setmCameraRoll(readFloatSerial());
        camera.setmCameraYaw(readFloatSerial());
        camera.setServoMinPitch(readFloatSerial());
        camera.setServoMinRoll(readFloatSerial());
        camera.setServoMinYaw(readFloatSerial());
        camera.setServoMaxPitch(readFloatSerial());
        camera.setServoMaxRoll(readFloatSerial());
        camera.setServoMaxYaw(readFloatSerial());
      #endif
      break;
    case 'I': // Initialize EEPROM with default values
      initializeEEPROM(); // defined in DataStorage.h
      writeEEPROM();
      calibrateGyro();
      computeAccelBias();
      zeroIntegralError();
      #ifdef HeadingMagHold
        initializeMagnetometer();
      #endif
      #ifdef AltitudeHold
        initializeBaro();
      #endif
      break;
    case 'J': // calibrate gyros
      calibrateGyro();
      storeSensorsZeroToEEPROM();
      break;
    case 'K': // Write accel calibration values
      accelScaleFactor[XAXIS] = readFloatSerial();
      runTimeAccelBias[XAXIS] = readFloatSerial();      
      accelScaleFactor[YAXIS] = readFloatSerial();
      runTimeAccelBias[YAXIS] = readFloatSerial();      
      accelScaleFactor[ZAXIS] = readFloatSerial();
      runTimeAccelBias[ZAXIS] = readFloatSerial();
      writeEEPROM();
      break;
    case 'L': // generate accel bias
      computeAccelBias();
      #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
        calibrateKinematics();
        accelOneG = meterPerSec[ZAXIS];
      #endif
      storeSensorsZeroToEEPROM();
      break;
    case 'M': // calibrate magnetometer
      #ifdef HeadingMagHold
        magBias[XAXIS] = readFloatSerial();      
        magBias[YAXIS] = readFloatSerial();
        magBias[ZAXIS] = readFloatSerial();
      #endif
      break;
    case 'N': // battery monitor
      #ifdef BattMonitor
        batteryMonitorAlarmVoltage = readFloatSerial();
        batteryMonitorThrottleTarget = readFloatSerial();
        batteryMonitorGoinDownTime = readFloatSerial();
      #else
        readFloatSerial();
        readFloatSerial();
        readFloatSerial();
      #endif
      break;
    case 'W': // Write all user configurable values to EEPROM
      writeEEPROM(); // defined in DataStorage.h
      zeroIntegralError();
      break;
    case '1': // Calibrate ESCS's by setting Throttle high on all channels
        validateCalibrateCommand(1);
      break;
    case '2': // Calibrate ESC's by setting Throttle low on all channels
        validateCalibrateCommand(2);
      break;
    case '3': // Test ESC calibration
      if (validateCalibrateCommand(3)) {
        testCommand = readFloatSerial();
      }
      break;
    case '4': // Turn off ESC calibration
      if (validateCalibrateCommand(4)) {
        calibrateESC = 0;
        testCommand = 1000;
      }
      break;
    case '5': // Send individual motor commands (motor, command)
      if (validateCalibrateCommand(5)) {
        for (byte motor = 0; motor < LASTMOTOR; motor++)
          motorConfiguratorCommand[motor] = (int)readFloatSerial();
      }
      break;
    case 'Z': // fast telemetry transfer <--- get rid if this?
      if (readFloatSerial() == 1.0)
        fastTransfer = ON;
      else
        fastTransfer = OFF;
      break;
    }
  }
}

//***************************************************************************************************
//********************************* Serial Telemetry ************************************************
//***************************************************************************************************

void PrintValueComma(float val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(double val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(char val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(int val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(unsigned long val)
{
  SERIAL_PRINT(val);
  comma();
}

void PrintPID(unsigned char IDPid)
{
  PrintValueComma(PID[IDPid].P);
  PrintValueComma(PID[IDPid].I);
  PrintValueComma(PID[IDPid].D);
}

void sendSerialTelemetry() {
  update = 0;
  switch (queryType) {
  case '=': // Reserved debug command to view any variable from Serial Monitor
    break;
  case 'a': // Send roll and pitch rate mode PID values
    PrintPID(ROLL);
    PrintPID(PITCH);
    SERIAL_PRINTLN();
    queryType = 'X';
    break;
  case 'b': // Send roll and pitch attitude mode PID values
    PrintPID(LEVELROLL);
    PrintPID(LEVELPITCH);
    PrintPID(LEVELGYROROLL);
    PrintPID(LEVELGYROPITCH);
    SERIAL_PRINTLN(windupGuard);
    queryType = 'X';
    break;
  case 'c': // Send yaw PID values
    PrintPID(YAW);
    PrintPID(HEADING);
    SERIAL_PRINTLN((int)headingHoldConfig);
    queryType = 'X';
    break;
  case 'd': // Altitude Hold
    #ifdef AltitudeHold
      PrintPID(ALTITUDE);
      PrintValueComma(PID[ALTITUDE].windupGuard);
      PrintValueComma(altitudeHoldBump);
      PrintValueComma(altitudeHoldPanicStickMovement);
      PrintValueComma(minThrottleAdjust);
      PrintValueComma(maxThrottleAdjust);
      PrintValueComma(baroSmoothFactor);
      PrintPID(ZDAMPENING);
    #else
      for(byte i=0; i<10; i++) {
        PrintValueComma(0);
      }
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;
  case 'e': // Send sensor filtering values
    PrintValueComma(gyroSmoothFactor);
    PrintValueComma(accelSmoothFactor);
    PrintValueComma(timeConstant);
    SERIAL_PRINTLN(aref);
    queryType = 'X';
    break;
  case 'f': // Send transmitter smoothing values
    PrintValueComma(receiverXmitFactor);
    for (byte axis = ROLL; axis < LASTCHANNEL; axis++) {
      PrintValueComma(receiverSmoothFactor[axis]);
    }
    SERIAL_PRINTLN();
    queryType = 'X';
    break;
  case 'g': // Send transmitter calibration data
    for (byte axis = ROLL; axis < LASTCHANNEL; axis++) {
      PrintValueComma(receiverSlope[axis]);
      PrintValueComma(receiverOffset[axis]);
    }
    SERIAL_PRINTLN();
    queryType = 'X';
    break;
  case 'h': // Send Camera values
    #ifdef Camera
      PrintValueComma(camera.getMode());
      PrintValueComma(camera.getCenterPitch());
      PrintValueComma(camera.getCenterRoll());
      PrintValueComma(camera.getCenterYaw());
      PrintValueComma(camera.getmCameraPitch(), 2);
      PrintValueComma(camera.getmCameraRoll(), 2);
      PrintValueComma(camera.getmCameraYaw(), 2);
      PrintValueComma(camera.getServoMinPitch());
      PrintValueComma(camera.getServoMinRoll());
      PrintValueComma(camera.getServoMinYaw());
      PrintValueComma(camera.getServoMaxPitch());
      PrintValueComma(camera.getServoMaxRoll());
      SERIAL_PRINTLN(camera.getServoMaxYaw());
    #else
      for (byte index=0; index < 12; index++) {
        PrintValueComma(0);
      }
      SERIAL_PRINTLN(0);
    #endif
    break;
  case 'i': // Send sensor data
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(gyroRate[axis]);
    }
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(meterPerSec[axis]);
    }
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      #if defined(HeadingMagHold)
        PrintValueComma(getMagnetometerRawData(axis));
      #else
        PrintValueComma(0);
      #endif
    }
    SERIAL_PRINTLN();
    break;
  case 'j': // Send raw mag values
    #ifdef HeadingMagHold
      PrintValueComma(getMagnetometerRawData(XAXIS));
      PrintValueComma(getMagnetometerRawData(YAXIS));
      SERIAL_PRINTLN(getMagnetometerRawData(ZAXIS));
    #endif
    break;
  case 'k': // Send accelerometer cal values
    SERIAL_PRINT(accelScaleFactor[XAXIS], 6);
    comma();
    SERIAL_PRINT(runTimeAccelBias[XAXIS], 6);
    comma();
    SERIAL_PRINT(accelScaleFactor[YAXIS], 6);
    comma();
    SERIAL_PRINT(runTimeAccelBias[YAXIS], 6);
    comma();
    SERIAL_PRINT(accelScaleFactor[ZAXIS], 6);
    comma();
    SERIAL_PRINTLN(runTimeAccelBias[ZAXIS], 6);
    queryType = 'X';
    break;
  case 'l': // Send raw accel values
    measureAccelSum();
    PrintValueComma(accelSample[XAXIS]/accelSampleCount);
    accelSample[XAXIS] = 0.0;
    PrintValueComma(accelSample[YAXIS]/accelSampleCount);
    accelSample[YAXIS] = 0.0;
    SERIAL_PRINTLN (accelSample[ZAXIS]/accelSampleCount);
    accelSample[ZAXIS] = 0.0;
    accelSampleCount = 0;
    break;
  case 'm': // Send magnetometer cal values
    #ifdef HeadingMagHold
      SERIAL_PRINT(magBias[XAXIS], 6);
      comma();
      SERIAL_PRINT(magBias[YAXIS], 6);
      comma();
      SERIAL_PRINTLN(magBias[ZAXIS], 6);
    #endif
    queryType = 'X';
    break;
  case 'n': // battery monitor
    #ifdef BattMonitor
      PrintValueComma(batteryMonitorAlarmVoltage);
      PrintValueComma(batteryMonitorThrottleTarget);
      SERIAL_PRINTLN(batteryMonitorGoinDownTime);
    #else
      PrintValueComma(0);
      PrintValueComma(0);
      SERIAL_PRINTLN(0);
    #endif
    break;
  case 'r': // Vehicle attitude
    PrintValueComma(kinematicsAngle[ROLL]);
    PrintValueComma(kinematicsAngle[PITCH]);
    #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      SERIAL_PRINTLN(kinematicsAngle[YAW]);
    #else
      SERIAL_PRINTLN(gyroHeading);
    #endif
    break;
  case 's': // Send all flight data
    PrintValueComma(armed);
    PrintValueComma(kinematicsAngle[ROLL]);
    PrintValueComma(kinematicsAngle[PITCH]);
    #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      PrintValueComma(kinematicsAngle[YAW]);
    #else
      PrintValueComma(gyroHeading);
    #endif
    #ifdef AltitudeHold
      PrintValueComma(getBaroAltitude());
      PrintValueComma((int)altitudeHoldState);
    #else
      PrintValueComma(0);
      PrintValueComma(0);
    #endif
    for (byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      PrintValueComma(receiverCommand[channel]);
    }
    for (byte channel = 0; channel < (8 - LASTCHANNEL); channel++) {// max of 8 transmitter channel supported
      PrintValueComma(0); // zero out unused transmitter channels
    }
    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      PrintValueComma(motorCommand[motor]);
    }
    for (byte motor = 0; motor < (8 - (LASTMOTOR)); motor++) {// max of 8 motor outputs supported
      PrintValueComma(0); // zero out unused motor channels
    }
    #ifdef BattMonitor
      PrintValueComma(batteryData[0].voltage);
    #else
      PrintValueComma(0);
    #endif
    PrintValueComma(flightMode);
    SERIAL_PRINTLN();
    break;
  case 't': // Send processed transmitter values
    PrintValueComma(receiverXmitFactor);
    for (byte axis = 0; axis < LASTCHANNEL; axis++) {
      PrintValueComma(receiverCommand[axis]);
    }
    SERIAL_PRINTLN();
    break;
  case 'x': // Stop sending messages
    break;
  case '!': // Send flight software version
    SERIAL_PRINTLN(SOFTWARE_VERSION, 1);
    queryType = 'X';
    break;
  case '#': // Send software configuration
    // Determine which hardware is used to define max/min sensor values for Configurator plots
    SERIAL_PRINT("SW Version: ");
    SERIAL_PRINTLN(SOFTWARE_VERSION, 1);
    SERIAL_PRINT("Board Type: ");
    #if defined(AeroQuad_v1)
      SERIAL_PRINTLN("v1.x");
    #elif defined(AeroQuadMega_v1)
      SERIAL_PRINTLN("Mega v1.x");
    #elif defined(AeroQuad_v18)
      SERIAL_PRINTLN("v1.8 and greater");
    #elif defined(AeroQuadMega_v2)
      SERIAL_PRINTLN("Mega v2");
    #elif defined(AeroQuadMega_v21)
      SERIAL_PRINTLN("Mega v21");
    #elif defined(AutonavShield)
      SERIAL_PRINTLN("AutonavShield");
    #elif defined(AeroQuad_Wii)
      SERIAL_PRINTLN("Wii");
    #elif defined(AeroQuadMega_Wii)
      SERIAL_PRINTLN("Mega Wii");
    #elif defined(ArduCopter)
      SERIAL_PRINTLN("ArduCopter");
    #elif defined(AeroQuadMega_CHR6DM)
      SERIAL_PRINTLN("CHR6DM");
    #elif defined(APM_OP_CHR6DM)
      SERIAL_PRINTLN("APM w/ CHR6DM");
    #elif defined(AeroQuad_Mini)
      SERIAL_PRINTLN("Mini");
    #endif
    // Determine which motor flight configuration for Configurator GUI
    SERIAL_PRINT("Flight Config: ");
    #if defined(quadPlusConfig)
      SERIAL_PRINTLN("Quad +");
    #elif defined(quadXConfig) 
      SERIAL_PRINTLN("Quad X");
    #elif defined (quadY4Config)
      SERIAL_PRINTLN("Quad Y4");
    #elif defined (triConfig)
      SERIAL_PRINTLN("Tri");
    #elif defined(hexPlusConfig)
      SERIAL_PRINTLN("Hex +");
    #elif defined(hexXConfig)
      SERIAL_PRINTLN("Hex X");
    #elif defined(hexY6Config)
      SERIAL_PRINTLN("Hex Y6");
    #elif defined(octoX8Config)
      SERIAL_PRINTLN("Octo X8");
    #elif defined(octoXConfig)
      SERIAL_PRINTLN("Octo X");
    #elif defined(octoXPlusConfig)
      SERIAL_PRINTLN("Octo X+");
    #endif
    SERIAL_PRINT("Receiver Ch's: ");
    SERIAL_PRINTLN(LASTCHANNEL);
    SERIAL_PRINT("Motors: ");
    SERIAL_PRINTLN(LASTMOTOR);
    queryType = 'X';
    break;
  case '6': // Report remote commands
    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      PrintValueComma(motorCommand[motor]);
    }
    SERIAL_PRINTLN();
    queryType = 'X';
    break;
  }
}

// Used to read floating point values from the serial port
float readFloatSerial() {
  #define SERIALFLOATSIZE 15
  byte index = 0;
  byte timeout = 0;
  char data[SERIALFLOATSIZE] = "";

  do {
    if (SERIAL_AVAILABLE() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = SERIAL_READ();
      timeout = 0;
      index++;
    }
  } while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < sizeof(data)-1));
  data[index] = '\0';

  return atof(data);
}

void comma() {
  SERIAL_PRINT(',');
}

void printInt(int data) {
  byte msb, lsb;

  msb = data >> 8;
  lsb = data & 0xff;

  binaryPort->write(msb);
  binaryPort->write(lsb);
}

void sendBinaryFloat(float data) {
  union binaryFloatType {
    byte floatByte[4];
    float floatVal;
  } binaryFloat;

  binaryFloat.floatVal = data;
  binaryPort->write(binaryFloat.floatByte[3]);
  binaryPort->write(binaryFloat.floatByte[2]);
  binaryPort->write(binaryFloat.floatByte[1]);
  binaryPort->write(binaryFloat.floatByte[0]);
}

void sendBinaryuslong(unsigned long data) {
  union binaryuslongType {
    byte uslongByte[4];
    unsigned long uslongVal;
  } binaryuslong;

  binaryuslong.uslongVal = data;
  binaryPort->write(binaryuslong.uslongByte[3]);
  binaryPort->write(binaryuslong.uslongByte[2]);
  binaryPort->write(binaryuslong.uslongByte[1]);
  binaryPort->write(binaryuslong.uslongByte[0]);
}

#ifdef BinaryWrite
void fastTelemetry()
{
  // **************************************************************
  // ***************** Fast Transfer Of Sensor Data ***************
  // **************************************************************
  // AeroQuad.h defines the output rate to be 10ms
  // Since writing to UART is done by hardware, unable to measure data rate directly
  // Through analysis:  115200 baud = 115200 bits/second = 14400 bytes/second
  // If float = 4 bytes, then 3600 floats/second
  // If 10 ms output rate, then 36 floats/10ms
  // Number of floats written using sendBinaryFloat is 15

  if (armed == ON) {
    #ifdef OpenlogBinaryWrite
       printInt(21845); // Start word of 0x5555
       sendBinaryuslong(currentTime);
        printInt((int)flightMode);
       for (byte axis = ROLL; axis < LASTAXIS; axis++) {
         sendBinaryFloat(gyroRate[axis]);
       }
       for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
         sendBinaryFloat(meterPerSec[axis]);
       }
       sendBinaryFloat(accelOneG);
       #ifdef HeadingMagHold
          sendBinaryFloat(hdgX);
          sendBinaryFloat(hdgY);
          sendBinaryFloat(getMagnetometerRawData(XAXIS));
          sendBinaryFloat(getMagnetometerRawData(YAXIS));
          sendBinaryFloat(getMagnetometerRawData(ZAXIS));
       #else
         sendBinaryFloat(0.0);
         sendBinaryFloat(0.0);
         sendBinaryFloat(0.0);
       #endif
        for (byte axis = ROLL; axis < ZAXIS; axis++) {
          sendBinaryFloat(kinematicsAngle[axis]);
        }
       printInt(32767); // Stop word of 0x7FFF
    #else
       printInt(21845); // Start word of 0x5555
       for (byte axis = ROLL; axis < LASTAXIS; axis++) {
         sendBinaryFloat(gyroRate[axis]);
       }
       for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
         sendBinaryFloat(meterPerSec[axis]);
       }
       for (byte axis = ROLL; axis < LASTAXIS; axis++)
       #ifdef HeadingMagHold
         sendBinaryFloat(getMagnetometerRawData(axis));
       #else
         sendBinaryFloat(0);
       #endif
       for (byte axis = ROLL; axis < LASTAXIS; axis++) {
         sendBinaryFloat(getGyroUnbias(axis));
       }
       for (byte axis = ROLL; axis < LASTAXIS; axis++) {
         sendBinaryFloat(kinematicsAngle[axis]);
       }
       printInt(32767); // Stop word of 0x7FFF
    #endif
  }
}
#endif // BinaryWrite

#endif // _AQ_SERIAL_COMM_

