/*
  AeroQuad v2.4 - April 2011
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

// Special thanks for 1k space optimization update from Ala42
// http://aeroquad.com/showthread.php?1369-The-big-enhancement-addition-to-2.0-code&p=13359&viewfull=1#post13359

#ifndef _AQ_STORAGE_H_
#define _AQ_STORAGE_H_

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
  

  float GYRO_SMOOTH_ADR;
  float GYRO_ROLL_ZERO_ADR;
  float GYRO_PITCH_ZERO_ADR;
  float GYRO_YAW_ZERO_ADR;
  float ACCEL_SMOOTH_ADR;
  float ACCEL_XAXIS_ZERO_ADR;
  float ACCEL_YAXIS_ZERO_ADR;
  float ACCEL_ZAXIS_ZERO_ADR;
  float ACCEL_1G_ADR;
  
  
  float WINDUPGUARD_ADR;
  float XMITFACTOR_ADR;
//  float FILTERTERM_ADR;
//  float HEADINGSMOOTH_ADR;
  float AREF_ADR;
  float FLIGHTMODE_ADR;
  float HEADINGHOLD_ADR;
//  float MINACRO_ADR;
//  float ALTITUDE_PGAIN_ADR;
  float ALTITUDE_MAX_THROTTLE_ADR;
  float ALTITUDE_MIN_THROTTLE_ADR;
  float ALTITUDE_SMOOTH_ADR;
//  float ZDAMP_PGAIN_ADR;
  float ALTITUDE_WINDUP_ADR;
  float MAGXMAX_ADR;
  float MAGXMIN_ADR;
  float MAGYMAX_ADR;
  float MAGYMIN_ADR;
  float MAGZMAX_ADR;
  float MAGZMIN_ADR;
  float SERVOMINPITCH_ADR;
  float SERVOMINROLL_ADR;
} t_NVR_Data;  




// **************************************************************
// *************************** EEPROM ***************************
// **************************************************************




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
void readEEPROM(void); 
void initSensorsZeroFromEEPROM(void);
void storeSensorsZeroToEEPROM(void);
void initReceiverFromEEPROM(void);


// Utilities for writing and reading from the EEPROM
float nvrReadFloat(int address) {
/*  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatOut;

  for (int i = 0; i < 4; i++)
    floatOut.floatByte[i] = EEPROM.read(address + i);
  return floatOut.floatVal; */
  union floatStore {
    byte floatByte[4];
    unsigned short floatUShort[2];
    float floatVal;
  } floatOut;

#ifdef EEPROM_USES_16BIT_WORDS
  for (int i = 0; i < 2; i++)
    floatOut.floatUShort[i] = EEPROM.read(address + 2*i);
#else
  for (int i = 0; i < 4; i++)
    floatOut.floatByte[i] = EEPROM.read(address + i);
#endif
  return floatOut.floatVal; 
//  return 0.0;
}

void nvrWriteFloat(float value, int address) {
/*  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatIn;

  floatIn.floatVal = value;
  for (int i = 0; i < 4; i++)
    EEPROM.write(address + i, floatIn.floatByte[i]); */
  union floatStore {
    byte floatByte[4];
    unsigned short floatUShort[2];
    float floatVal;
  } floatIn;

  floatIn.floatVal = value;
#ifdef EEPROM_USES_16BIT_WORDS
  for (int i = 0; i < 2; i++)
    EEPROM.write(address + 2*i, floatIn.floatUShort[i]);
#else
  for (int i = 0; i < 4; i++)
    EEPROM.write(address + i, floatIn.floatByte[i]);
#endif
}

void nvrReadPID(unsigned char IDPid, unsigned int IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  pid->P = nvrReadFloat(IDEeprom);
  pid->I = nvrReadFloat(IDEeprom+4);
  pid->D = nvrReadFloat(IDEeprom+8);
  pid->lastPosition = 0;
  pid->integratedError = 0;
}

void nvrWritePID(unsigned char IDPid, unsigned int IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  nvrWriteFloat(pid->P, IDEeprom);
  nvrWriteFloat(pid->I, IDEeprom+4);
  nvrWriteFloat(pid->D, IDEeprom+8);
}

class Storage {
private:



public:
  Storage() {
  
  }
  
  
  
// contains all default values when re-writing EEPROM
void initializeEEPROM(void) {
  PID[ROLL].P = 100.0;
  PID[ROLL].I = 0.0;
  PID[ROLL].D = -300.0;
  PID[PITCH].P = 100.0;
  PID[PITCH].I = 0.0;
  PID[PITCH].D = -300.0;
  PID[YAW].P = 200.0;
  PID[YAW].I = 5.0;
  PID[YAW].D = 0.0;
  PID[LEVELROLL].P = 4.0;
  PID[LEVELROLL].I = 0.6;
  PID[LEVELROLL].D = 0.0;
  PID[LEVELPITCH].P = 4.0;
  PID[LEVELPITCH].I = 0.6;
  PID[LEVELPITCH].D = 0.0;
  PID[HEADING].P = 3.0;
  PID[HEADING].I = 0.1;
  PID[HEADING].D = 0.0;
  PID[LEVELGYROROLL].P = 100.0;
  PID[LEVELGYROROLL].I = 0.0;
  PID[LEVELGYROROLL].D = -300.0;
  PID[LEVELGYROPITCH].P = 100.0;
  PID[LEVELGYROPITCH].I = 0.0;
  PID[LEVELGYROPITCH].D = -300.0;
  
  PID[ALTITUDE].P = 25.0;
  PID[ALTITUDE].I = 0.1;
  PID[ALTITUDE].D = 0.0;
  PID[ZDAMPENING].P = 0.0;
  PID[ZDAMPENING].I = 0.0;
  PID[ZDAMPENING].D = 0.0;
  minThrottleAdjust = -50.0;
  maxThrottleAdjust = 50.0; //we don't want it to be able to take over totally
  #ifdef AltitudeHold    
    barometricSensor->setSmoothFactor(0.1);
  #endif
  #ifdef HeadingMagHold
    compass->setMagCal(XAXIS, 1, 0);
    compass->setMagCal(YAXIS, 1, 0);
    compass->setMagCal(ZAXIS, 1, 0);
  #endif
  windupGuard = 1000.0;

  // AKA - added so that each PID has its own windupGuard, will need to be removed once each PID's range is established and put in the eeprom
  for (byte i = ROLL; i <= ZDAMPENING; i++ ) {
    if (i != ALTITUDE)
        PID[i].windupGuard = windupGuard;
  }
  PID[LEVELROLL].windupGuard = 0.375;
  PID[LEVELPITCH].windupGuard = 0.375;
  
  PID[ALTITUDE].windupGuard = 25.0; //this prevents the 0.1 I term to rise too far
  
  
  receiver->setXmitFactor(1.0);
  gyro->setSmoothFactor(1.0);
  accel->setSmoothFactor(1.0);
  accel->setOneG(9.80665); 
//  timeConstant = 7.0;
  for (byte channel = ROLL; channel < LASTCHANNEL; channel++) {
    receiver->setTransmitterSlope(channel, 1.0);
    receiver->setTransmitterOffset(channel, 0.0);
    receiver->setSmoothFactor(channel, 1.0);
  }
  receiver->setSmoothFactor(YAW, 0.5);

 // smoothHeading = 1.0;
  flight->flightMode = ACRO;
  flight->headingHoldConfig = OFF;

  aref = 5.0; // Use 3.0 if using a v1.7 shield or use 2.8 for an AeroQuad Shield < v1.7
  
  /*#ifdef Camera
    mCameraPitch = 11.11;   // scale angle to servo....  caculated as +/- 90 (ie 180) degrees maped to 1000-2000 
    mCameraRoll = 11.11;        
    mCameraYaw = 11.11;
    centerPitch = 1500;       // (bCamera) Center of stabilisation in mode 1,  point here in mode 2  
    centerRoll = 1500;        // 1000 - 2000 nornaly centered 1500
    centerYaw = 1500;  
    servoMinPitch = 1000;     // don't drive the servo past here  
    servoMinRoll = 1000;
    servoMinYaw = 1000;
    servoMaxPitch = 2000;
    servoMaxRoll = 2000;
    servoMaxYaw = 2000;
  #endif*/
  
// SnorCopter  mattias@welponer.net
  PID[ROLL].P = 35.0;
  PID[ROLL].I = 0.0;
  PID[ROLL].D = -5*PID[ROLL].P;
  PID[PITCH].P = PID[ROLL].P;
  PID[PITCH].I = PID[ROLL].I;
  PID[PITCH].D = PID[ROLL].D;
  PID[YAW].P = 2*PID[ROLL].P;
  PID[YAW].I = 0*5.0;
  PID[YAW].D = -4*PID[ROLL].P;
  PID[LEVELROLL].P = PID[ROLL].P/10;
  PID[LEVELROLL].I = 0.2;
  PID[LEVELROLL].D = 0.0;
  PID[LEVELPITCH].P = PID[LEVELROLL].P;
  PID[LEVELPITCH].I = PID[LEVELROLL].I;
  PID[LEVELPITCH].D = PID[LEVELROLL].D;
  PID[HEADING].P = 3.0;
  PID[HEADING].I = 0.1;
  PID[HEADING].D = 0.0;  
  PID[LEVELGYROROLL].P = PID[ROLL].P;
  PID[LEVELGYROROLL].I = PID[ROLL].I;
  PID[LEVELGYROROLL].D = PID[ROLL].D;
  PID[LEVELGYROPITCH].P = PID[ROLL].P;
  PID[LEVELGYROPITCH].I = PID[ROLL].I;
  PID[LEVELGYROPITCH].D = PID[ROLL].D;
  flight->headingHoldConfig = ON;
  aref = 5.0;
  receiver->setTransmitterSlope(THROTTLE, 0.5);
  receiver->setTransmitterOffset(THROTTLE, 500);
}




  
  void readEEPROM(void) {
  readPID(ROLL, ROLL_PID_GAIN_ADR);
  readPID(PITCH, PITCH_PID_GAIN_ADR);
  readPID(YAW, YAW_PID_GAIN_ADR);
  readPID(LEVELROLL, LEVELROLL_PID_GAIN_ADR);
  readPID(LEVELPITCH, LEVELPITCH_PID_GAIN_ADR);
  readPID(HEADING, HEADING_PID_GAIN_ADR);
  readPID(LEVELGYROROLL, LEVEL_GYRO_ROLL_PID_GAIN_ADR);
  readPID(LEVELGYROPITCH, LEVEL_GYRO_PITCH_PID_GAIN_ADR);

  // Leaving separate PID reads as commented for now
  // Previously had issue where EEPROM was not reading right data
  readPID(ALTITUDE, ALTITUDE_PID_GAIN_ADR);
  PID[ALTITUDE].windupGuard = readFloat(ALTITUDE_WINDUP_ADR);
  minThrottleAdjust = readFloat(ALTITUDE_MIN_THROTTLE_ADR);
  maxThrottleAdjust = readFloat(ALTITUDE_MAX_THROTTLE_ADR);
  #ifdef AltitudeHold    
    barometricSensor->setSmoothFactor(readFloat(ALTITUDE_SMOOTH_ADR));
  #endif
  readPID(ZDAMPENING, ZDAMP_PID_GAIN_ADR);

  #ifdef HeadingMagHold
    compass->setMagCal(XAXIS, readFloat(MAGXMAX_ADR), readFloat(MAGXMIN_ADR));
    compass->setMagCal(YAXIS, readFloat(MAGYMAX_ADR), readFloat(MAGYMIN_ADR));
    compass->setMagCal(ZAXIS, readFloat(MAGZMAX_ADR), readFloat(MAGZMIN_ADR));
  #endif

  windupGuard = readFloat(WINDUPGUARD_ADR);

  // AKA - added so that each PID has its own windupGuard, will need to be removed once each PID's range is established and put in the eeprom
  for (byte i = ROLL; i <= ZDAMPENING; i++ ) {
    if (i != ALTITUDE)
        PID[i].windupGuard = windupGuard;
  }
    
//  timeConstant = readFloat(FILTERTERM_ADR);
//  smoothHeading = readFloat(HEADINGSMOOTH_ADR);
  aref = readFloat(AREF_ADR);
  flight->flightMode = readFloat(FLIGHTMODE_ADR);
  flight->headingHoldConfig = readFloat(HEADINGHOLD_ADR);
//  copter->minAcro = readFloat(MINACRO_ADR);
  accel->setOneG(readFloat(ACCEL_1G_ADR));
  
  /*#ifdef Camera
  mCameraPitch = readFloat(MCAMERAPITCH_ADR);
  mCameraRoll = readFloat(MCAMERAROLL_ADR);
  mCameraYaw = readFloat(MCAMERAYAW_ADR);
  centerPitch = readFloat(CENTERPITCH_ADR);
  centerRoll = readFloat(CENTERROLL_ADR);
  centerYaw = readFloat(CENTERYAW_ADR);
  servoMinPitch = readFloat(SERVOMINPITCH_ADR);
  servoMinRoll = readFloat(SERVOMINROLL_ADR);
  servoMinYaw = readFloat(SERVOMINYAW_ADR);
  servoMaxPitch = readFloat(SERVOMAXPITCH_ADR);
  servoMaxRoll = readFloat(SERVOMAXROLL_ADR);
  servoMaxYaw = readFloat(SERVOMAXYAW_ADR);
  #endif*/
}

void writeEEPROM(void){
//  cli(); // Needed so that APM sensor data doesn't overflow
  writePID(ROLL, ROLL_PID_GAIN_ADR);
  writePID(PITCH, PITCH_PID_GAIN_ADR);
  writePID(LEVELROLL, LEVELROLL_PID_GAIN_ADR);
  writePID(LEVELPITCH, LEVELPITCH_PID_GAIN_ADR);
  writePID(YAW, YAW_PID_GAIN_ADR);
  writePID(HEADING, HEADING_PID_GAIN_ADR);
  writePID(LEVELGYROROLL, LEVEL_GYRO_ROLL_PID_GAIN_ADR);
  writePID(LEVELGYROPITCH, LEVEL_GYRO_PITCH_PID_GAIN_ADR);
  writePID(ALTITUDE, ALTITUDE_PID_GAIN_ADR);
  writeFloat(PID[ALTITUDE].windupGuard, ALTITUDE_WINDUP_ADR);
  writeFloat(minThrottleAdjust, ALTITUDE_MIN_THROTTLE_ADR);
  writeFloat(maxThrottleAdjust, ALTITUDE_MAX_THROTTLE_ADR);
  #ifdef AltitudeHold    
    writeFloat(barometricSensor->getSmoothFactor(), ALTITUDE_SMOOTH_ADR);
  #else
    writeFloat(0.1, ALTITUDE_SMOOTH_ADR);
  #endif
  writePID(ZDAMPENING, ZDAMP_PID_GAIN_ADR);
  #ifdef HeadingMagHold
    writeFloat(compass->getMagMax(XAXIS), MAGXMAX_ADR);
    writeFloat(compass->getMagMin(XAXIS), MAGXMIN_ADR);
    writeFloat(compass->getMagMax(YAXIS), MAGYMAX_ADR);
    writeFloat(compass->getMagMin(YAXIS), MAGYMIN_ADR);
    writeFloat(compass->getMagMax(ZAXIS), MAGZMAX_ADR);
    writeFloat(compass->getMagMin(ZAXIS), MAGZMIN_ADR);
  #else
    writeFloat(1.0F, MAGXMAX_ADR);
    writeFloat(0.0F, MAGXMIN_ADR);
    writeFloat(1.0F, MAGYMAX_ADR);
    writeFloat(0.0F, MAGYMIN_ADR);
    writeFloat(1.0F, MAGZMAX_ADR);
    writeFloat(0.0F, MAGZMIN_ADR);
  #endif
  writeFloat(windupGuard, WINDUPGUARD_ADR);
  writeFloat(receiver->getXmitFactor(), XMITFACTOR_ADR);
  writeFloat(gyro->getSmoothFactor(), GYRO_SMOOTH_ADR);
  writeFloat(accel->getSmoothFactor(), ACCEL_SMOOTH_ADR);
//  writeFloat(timeConstant, FILTERTERM_ADR);

  for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
    writeFloat(receiver->getTransmitterSlope(channel),  RECEIVER_DATA[channel].slope);
    writeFloat(receiver->getTransmitterOffset(channel), RECEIVER_DATA[channel].offset);
    writeFloat(receiver->getSmoothFactor(channel),      RECEIVER_DATA[channel].smooth_factor);
  }

 // writeFloat(smoothHeading, HEADINGSMOOTH_ADR);
  writeFloat(aref, AREF_ADR);
  writeFloat(flight->flightMode, FLIGHTMODE_ADR);
  writeFloat(flight->headingHoldConfig, HEADINGHOLD_ADR);
//  writeFloat(copter->minAcro, MINACRO_ADR);
  writeFloat(accel->getOneG(), ACCEL_1G_ADR);
    
  /*#ifdef Camera
  writeFloat(mCameraPitch, MCAMERAPITCH_ADR);
  writeFloat(mCameraRoll, MCAMERAROLL_ADR);
  writeFloat(mCameraYaw, MCAMERAYAW_ADR);
  writeFloat(centerPitch, CENTERPITCH_ADR);
  writeFloat(centerRoll, CENTERROLL_ADR);
  writeFloat(centerYaw, CENTERYAW_ADR);
  writeFloat(servoMinPitch, SERVOMINPITCH_ADR);
  writeFloat(servoMinRoll, SERVOMINROLL_ADR);
  writeFloat(servoMinYaw, SERVOMINYAW_ADR);
  writeFloat(servoMaxPitch, SERVOMAXPITCH_ADR);
  writeFloat(servoMaxRoll, SERVOMAXROLL_ADR);
  writeFloat(servoMaxYaw, SERVOMAXYAW_ADR);
  #endif*/
  
//  sei(); // Restart interrupts
}


  
void initSensorsZeroFromEEPROM(void) {
  // Gyro initialization from EEPROM
  gyro->setZero(ROLL,readFloat(GYRO_ROLL_ZERO_ADR));
  gyro->setZero(PITCH,readFloat(GYRO_PITCH_ZERO_ADR));
  gyro->setZero(YAW,readFloat(GYRO_YAW_ZERO_ADR));
  gyro->setSmoothFactor(readFloat(GYRO_SMOOTH_ADR));
  
  // Accel initialization from EEPROM
  accel->setOneG(readFloat(ACCEL_1G_ADR));
  accel->setZero(XAXIS,readFloat(ACCEL_XAXIS_ZERO_ADR));
  accel->setZero(YAXIS,readFloat(ACCEL_YAXIS_ZERO_ADR));
  accel->setZero(ZAXIS,readFloat(ACCEL_ZAXIS_ZERO_ADR));
  accel->setSmoothFactor(readFloat(ACCEL_SMOOTH_ADR));
}


void storeSensorsZeroToEEPROM(void) {
  // Store gyro data to EEPROM
  writeFloat(gyro->getZero(ROLL), GYRO_ROLL_ZERO_ADR);
  writeFloat(gyro->getZero(PITCH), GYRO_PITCH_ZERO_ADR);
  writeFloat(gyro->getZero(YAW), GYRO_YAW_ZERO_ADR);
  writeFloat(gyro->getSmoothFactor(), GYRO_SMOOTH_ADR);
  
  // Store accel data to EEPROM
  writeFloat(accel->getOneG(), ACCEL_1G_ADR);
  writeFloat(accel->getZero(XAXIS), ACCEL_XAXIS_ZERO_ADR);
  writeFloat(accel->getZero(YAXIS), ACCEL_YAXIS_ZERO_ADR);
  writeFloat(accel->getZero(ZAXIS), ACCEL_ZAXIS_ZERO_ADR);
  writeFloat(accel->getSmoothFactor(), ACCEL_SMOOTH_ADR);
}

void initReceiverFromEEPROM(void) {
  receiver->setXmitFactor(readFloat(XMITFACTOR_ADR));
  
  for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
    receiver->setTransmitterSlope(channel,readFloat(RECEIVER_DATA[channel].slope));
    receiver->setTransmitterOffset(channel,readFloat(RECEIVER_DATA[channel].offset));
    receiver->setSmoothFactor(channel,readFloat(RECEIVER_DATA[channel].smooth_factor));
  }
}



  void init(void) {
    initializeEEPROM();
  }
  
  
  void load() {
    readEEPROM();
  }

  void store(void) {
    writeEEPROM();
  }  
  void initSensorsZero(void) {
    initSensorsZeroFromEEPROM();
  }
  
  void storeSensorsZero(void) {
    storeSensorsZeroToEEPROM();
  }
  
  void initReceiver(void) {
    initReceiverFromEEPROM();
  }

};

#endif
