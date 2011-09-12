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

// Special thanks for 1k space optimization update from Ala42
// http://aeroquad.com/showthread.php?1369-The-big-enhancement-addition-to-2.0-code&p=13359&viewfull=1#post13359

// Utilities for writing and reading from the EEPROM
float nvrReadFloat(int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatOut;

  for (int i = 0; i < 4; i++)
    floatOut.floatByte[i] = EEPROM.read(address + i);
  return floatOut.floatVal;
}

void nvrWriteFloat(float value, int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatIn;

  floatIn.floatVal = value;
  for (int i = 0; i < 4; i++)
    EEPROM.write(address + i, floatIn.floatByte[i]);
}

void nvrReadPID(unsigned char IDPid, unsigned int IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  pid->P = nvrReadFloat(IDEeprom);
  pid->I = nvrReadFloat(IDEeprom+4);
  pid->D = nvrReadFloat(IDEeprom+8);
  pid->windupGuard = nvrReadFloat(IDEeprom+12);
  pid->firstPass = true;
  pid->iTerm = 0;
  pid->lastState = 0;
}

void nvrWritePID(unsigned char IDPid, unsigned int IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  nvrWriteFloat(pid->P, IDEeprom);
  nvrWriteFloat(pid->I, IDEeprom+4);
  nvrWriteFloat(pid->D, IDEeprom+8);
  nvrWriteFloat(pid->windupGuard, IDEeprom+12);
}

// contains all default values when re-writing EEPROM
void initializeEEPROM(void) {
  PID[ROLL_RATE_PID].P = 100.0;
  PID[ROLL_RATE_PID].I = 0.0;
  PID[ROLL_RATE_PID].D = 300.0;
  PID[ROLL_RATE_PID].windupGuard = 100;  // PWMs
  
  PID[PITCH_RATE_PID].P = 100.0;
  PID[PITCH_RATE_PID].I = 0.0;
  PID[PITCH_RATE_PID].D = 300.0;
  PID[PITCH_RATE_PID].windupGuard = 100;  // PWMs
  
  PID[YAW_RATE_PID].P = 200.0;
  PID[YAW_RATE_PID].I = 5.0;
  PID[YAW_RATE_PID].D = 0.0;
  PID[YAW_RATE_PID].windupGuard = 100;  // PWMs
  
  PID[ROLL_ATT_PID].P = 4.0;
  PID[ROLL_ATT_PID].I = 0.6;
  PID[ROLL_ATT_PID].D = 0.0;
  PID[ROLL_ATT_PID].windupGuard = 0.5;  // Radians/Sec
  
  PID[PITCH_ATT_PID].P = 4.0;
  PID[PITCH_ATT_PID].I = 0.6;
  PID[PITCH_ATT_PID].D = 0.0;
  PID[PITCH_ATT_PID].windupGuard = 0.5;  // Radians/Sec
  
  PID[HEADING_PID].P = 3.0;
  PID[HEADING_PID].I = 0.1;
  PID[HEADING_PID].D = 0.0;
  PID[HEADING_PID].windupGuard = 0.5;  // Radians/Sec
  
  smoothHeading = 1.0;
  headingHoldConfig = OFF;
  minAcro = 1300;
  
  Serial.println("EEPROM Initialized");
}

void readEEPROM(void) {
  accelScaleFactor[XAXIS] = readFloat(XAXIS_ACCEL_SCALE_FACTOR_ADR);
  accelScaleFactor[YAXIS] = readFloat(YAXIS_ACCEL_SCALE_FACTOR_ADR);
  accelScaleFactor[ZAXIS] = readFloat(ZAXIS_ACCEL_SCALE_FACTOR_ADR);
  
  #if defined(HMC5843) | defined(HMC5883)
    magBias[XAXIS] = readFloat(XAXIS_MAG_BIAS_ADR);
    magBias[YAXIS] = readFloat(YAXIS_MAG_BIAS_ADR);
    magBias[ZAXIS] = readFloat(ZAXIS_MAG_BIAS_ADR);
  #endif  
  
  readPID(ROLL_RATE_PID,  ROLL_RATE_PID_GAIN_ADR);
  readPID(PITCH_RATE_PID, PITCH_RATE_PID_GAIN_ADR);
  readPID(YAW_RATE_PID,   YAW_RATE_PID_GAIN_ADR);
  
  readPID(ROLL_ATT_PID,  ROLL_ATT_PID_GAIN_ADR);
  readPID(PITCH_ATT_PID, PITCH_ATT_PID_GAIN_ADR);
  readPID(HEADING_PID,   HEADING_PID_GAIN_ADR);
  
  smoothHeading = readFloat(HEADINGSMOOTH_ADR);
  headingHoldConfig = readFloat(HEADINGHOLD_ADR);
  minAcro = readFloat(MINACRO_ADR);  
  
  Serial.println("EEPROM Read");
}

void writeEEPROM(void){
  cli();
  
  writePID(ROLL_RATE_PID,  ROLL_RATE_PID_GAIN_ADR);
  writePID(PITCH_RATE_PID, PITCH_RATE_PID_GAIN_ADR);
  writePID(YAW_RATE_PID,   YAW_RATE_PID_GAIN_ADR);
  
  writePID(ROLL_ATT_PID,  ROLL_ATT_PID_GAIN_ADR);
  writePID(PITCH_ATT_PID, PITCH_ATT_PID_GAIN_ADR);
  writePID(HEADING_PID,   HEADING_PID_GAIN_ADR);
  
  writeFloat(smoothHeading, HEADINGSMOOTH_ADR);
  writeFloat(headingHoldConfig, HEADINGHOLD_ADR);
  writeFloat(minAcro, MINACRO_ADR);
  
  Serial.println("EEPROM Written");
  sei(); 
}
