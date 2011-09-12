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

// SerialCom.pde is responsible for the serial communication for commands and telemetry from the AeroQuad
// This comtains readSerialCommand() which listens for a serial command and it's arguments
// This also contains readSerialTelemetry() which listens for a telemetry request and responds with the requested data
// For more information on each command/telemetry look at: http://aeroquad.com/content.php?117

// Includes re-write / fixes from Aadamson and ala42, special thanks to those guys!
// http://aeroquad.com/showthread.php?1461-We-have-some-hidden-warnings&p=14618&viewfull=1#post14618

//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************

bool validateCalibrateCommand(void)
{
  if (readFloatSerial() == 123.45 && armed == OFF)  // use a specific float value to validate full throttle call is being sent
    return true;
  else
    return false;
}

void readSerialPID(unsigned char PIDid) {
  struct PIDdata* pid = &PID[PIDid];
  pid->P = readFloatSerial();
  pid->I = readFloatSerial();
  pid->D = readFloatSerial();
  pid->windupGuard = readFloatSerial();
  pid->lastState = 0;
  pid->iTerm = 0;
  pid->firstPass = true;
}

void readSerialCommand() {
  // Check for serial message
  if (SERIAL_AVAILABLE()) {
    digitalWrite(LEDPIN, LOW);
    queryType = SERIAL_READ();
    switch (queryType) {
    
    case 'A':  // Receive Roll Rate PID values
      readSerialPID(ROLL_RATE_PID);
      break;
    
    case 'B':  // Receive Pitch Rate PID values
      readSerialPID(PITCH_RATE_PID);
      break;
      
    case 'C':  // Receive Yaw Rate PID values
      readSerialPID(YAW_RATE_PID);
      break;
      
    case 'D':  // Receive minAcro value
      minAcro = readFloatSerial();
      break;
      
    case 'E':  // Receive Roll Attitude PID values
      readSerialPID(ROLL_ATT_PID);
      break;
      
    case 'F':  // Receive Pitch Attitude PID values
      readSerialPID(PITCH_ATT_PID);
      break;
      
    case 'G':  // Receive Heading PID values
      readSerialPID(HEADING_PID);
      break;

    case 'I': // Initialize EEPROM with default values
      initializeEEPROM();
      computeGyroBias();
      computeAccelBias();
      zeroIntegralError();
      break;
      
    case 'K': // calibrate gyros
      computeGyroBias();
      break;
      
    case 'L': // calibrate accels
      computeAccelBias();
      break;     

    case 'W': // Write all user configurable values to EEPROM
      writeEEPROM(); // defined in DataStorage.h
      zeroIntegralError();
      break;
      
    #if !defined(I2C_ESC)
      case '1': // Calibrate ESCS's by setting MAXCOMMAND on all motors
      	if (validateCalibrateCommand())
        {
          SERIAL_PRINTLN("Setting MAXCOMMAND");
          escsCalibrating = ON;
          commandAllMotors(MAXCOMMAND);
        }
        break;
        
      case '2': // Calibrate ESC's by setting MINCOMMAND on all motors
      	if (validateCalibrateCommand())
        {
          SERIAL_PRINTLN("Setting MINCOMMAND");
          commandAllMotors(MINCOMMAND);
        }
        break;
        
      case '3': // Test ESC calibration
        if (validateCalibrateCommand())
        {
          SERIAL_PRINTLN("Setting Test Command");
          commandAllMotors(constrain(readFloatSerial(), MINCOMMAND, MINCOMMAND + 200));
        }
        break;
      #endif
           
    case '4': // Turn off ESC calibration
      if (validateCalibrateCommand())
      {
	  SERIAL_PRINTLN("ESC Calibration Off");
       escsCalibrating = OFF;
      }
      break;
      
    case '5': // Send individual motor commands (motor, command)
      if (validateCalibrateCommand())
      {
        escsCalibrating = ON;
        
        for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
          remoteMotorCommand[motor] = readFloatSerial();
        
        for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
          motorCommand[motor] = constrain(remoteMotorCommand[motor], 1000, 1200);
          
        writeMotors();
      }
      break;
    }
    digitalWrite(LEDPIN, HIGH);
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

void printPID(unsigned char IDPid)
{
  PrintValueComma(PID[IDPid].P);
  PrintValueComma(PID[IDPid].I);
  PrintValueComma(PID[IDPid].D);
  SERIAL_PRINTLN(PID[IDPid].windupGuard);
}

void sendSerialTelemetry() {
  update = 0;
  switch (queryType) {
  
  case 'a':  // Send Roll Rate PID values
    printPID(ROLL_RATE_PID);
    queryType = 'x';
    break;
    
  case 'b':  // Send Pitch Rate PID values
    printPID(PITCH_RATE_PID);
    queryType = 'x';
    break;
    
  case 'c':  // Send Yaw Rate PID values
    printPID(YAW_RATE_PID);
    queryType = 'x';
    break;
    
  case 'd':  // Send minAcro
    SERIAL_PRINTLN(minAcro);
    queryType = 'x';
    break;
    
  case 'e':  // Send Roll Attiude PID values
    printPID(ROLL_ATT_PID);
    queryType = 'x';
    break;
    
  case 'f':  // Send Pitch Attitude PID values
    printPID(PITCH_ATT_PID);
    queryType = 'x';
    break;
    
  case 'g':  // Send Heading PID values
    printPID(HEADING_PID);
    queryType = 'x';
    break;
    
  case 'h': // Send Filtered Accels
    PrintValueComma(filteredAccel.value[XAXIS]);
    PrintValueComma(filteredAccel.value[YAXIS]);
    SERIAL_PRINTLN(filteredAccel.value[ZAXIS]);
    break;
    
  case 'i': // Send Rate Gyros
    PrintValueComma(gyro.value[ROLL]  * 57.3);
    PrintValueComma(gyro.value[PITCH] * 57.3);
    SERIAL_PRINTLN(gyro.value[YAW]    * 57.3);
    break;
    
  case 'j': // Send Attitudes
    PrintValueComma(angle.value[ROLL]  * 57.3);
    PrintValueComma(angle.value[PITCH] * 57.3);
    SERIAL_PRINTLN(angle.value[YAW]    * 57.3);
    break;
    
  #if defined(HMC5843) | defined(HMC5883)
  case 'k': // Send Raw Mag
    PrintValueComma(mag.value[XAXIS]);
    PrintValueComma(mag.value[YAXIS]);
    SERIAL_PRINTLN(mag.value[ZAXIS]);
    break;
  #endif
  
  case 'm': // Send Motor Commands 1 thru 4
    #if defined(__AVR_ATmega328P__) && !defined(I2C_ESC)
      #if (LASTMOTOR == 4)
        PrintValueComma(int(OCR2B)*16);
        PrintValueComma(int(OCR1A)*16);
        PrintValueComma(int(OCR1B)*16);
        SERIAL_PRINTLN(int(OCR2A)*16);
      #elif (LASTMOTOR == 6)
        PrintValueComma(int(OCR2B)*16);
        PrintValueComma(int(OCR1A)*16);
        PrintValueComma(int(OCR1B)*16);
        PrintValueComma(int(OCR2A)*16);
        PrintValueComma(int(PWM_MOTOR4PIN_highState)*8);
        SERIAL_PRINTLN(int(PWM_MOTOR5PIN_highState)*8);
      #endif
    #elif (defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)) && !defined(I2C_ESC)
      #if (LASTMOTOR == 4)
        PrintValueComma(int(OCR3B)/2);
        PrintValueComma(int(OCR3C)/2);
        PrintValueComma(int(OCR3A)/2);
        SERIAL_PRINTLN(int(OCR4A)/2);
      #elif (LASTMOTOR == 6)
        PrintValueComma(int(OCR3B)/2);
        PrintValueComma(int(OCR3C)/2);
        PrintValueComma(int(OCR3A)/2);
        PrintValueComma(int(OCR4A)/2);
        PrintValueComma(int(OCR4B)/2);
        SERIAL_PRINTLN(int(OCR4C)/2);
      #elif (LASTMOTOR ==8)
        PrintValueComma(int(OCR3B)/2);
        PrintValueComma(int(OCR3C)/2);
        PrintValueComma(int(OCR3A)/2);
        PrintValueComma(int(OCR4A)/2);
        PrintValueComma(int(OCR4B)/2);
        PrintValueComma(int(OCR4C)/2);
        PrintValueComma(int(OCR1A)/2);
        SERIAL_PRINTLN(int(OCR1B)/2);
      #endif
    #elif defined(I2C_ESC)
      for (byte motor = FIRSTMOTOR; motor < LASTMOTOR -1; motor++)
        PrintValueComma(motorCommandI2C[motor] * 4 + 1000);
        
      SERIAL_PRINTLN(motorCommandI2C[LASTMOTOR] * 4 + 1000);
    #endif
    break;
  
  case 'n': // Send Motor Axis Commands
    PrintValueComma(motorAxisCommand[ROLL]);
    PrintValueComma(motorAxisCommand[PITCH]);
    SERIAL_PRINTLN(motorAxisCommand[YAW]);
    break;
    
  case 'r': // Send Receiver Commands 1 thru 5
    PrintValueComma(receiverData[ROLL]);
    PrintValueComma(receiverData[PITCH]);
    PrintValueComma(receiverData[YAW]);
    PrintValueComma(receiverData[THROTTLE]);
    SERIAL_PRINTLN(receiverData[MODE]);
    
  case 'x': // Stop sending messages
    break;

  case 'z': // Send flight software version
    SERIAL_PRINTLN(VERSION, 1);
    #if defined(AeroQuad_v18)
      SERIAL_PRINTLN("V18");
    #elif defined(AeroQuadMega_v2)
      SERIAL_PRINTLN("V2");
    #elif defined(AeroQuad_Mini) | defined(AeroQuad_Mini_FFIMUV2)
      SERIAL_PRINTLN("Mini");
    #endif
    #if defined(quadPlusConfig)
      SERIAL_PRINTLN("Quad +");
    #elif defined(quadXConfig)
      SERIAL_PRINTLN("Quad X");
    #elif defined(y4Config)
      SERIAL_PRINTLN("Y4");
    #elif defined(hexPlusConfig)
      SERIAL_PRINTLN("Hex +");
    #elif defined(hexXConfig)
      SERIAL_PRINTLN("Hex X");
    #elif defined(y6Config)
      SERIAL_PRINTLN("Y6");
    #endif
    queryType = 'X';
    break;
    
  #if !defined(I2C_ESC)
    case '6': // Report remote commands
      #if (LASTMOTOR == 4)
        PrintValueComma(remoteMotorCommand[0]);
        PrintValueComma(remoteMotorCommand[1]);
        PrintValueComma(remoteMotorCommand[2]);
        SERIAL_PRINTLN(remoteMotorCommand[3]);
      #else
        PrintValueComma(remoteMotorCommand[0]);
        PrintValueComma(remoteMotorCommand[1]);
        PrintValueComma(remoteMotorCommand[2]);
        PrintValueComma(remoteMotorCommand[3]);
        PrintValueComma(remoteMotorCommand[4]);
        SERIAL_PRINTLN(remoteMotorCommand[5]);
      #endif
      queryType = 'X';
      break;
  #endif
    
  case '=': // Send Free Form Debug
    // What are you looking at?  And why?
    SERIAL_PRINTLN((int)armed);
    break;
  }
}

// Used to read floating point values from the serial port
float readFloatSerial() {
  #define SERIALFLOATSIZE 10
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




