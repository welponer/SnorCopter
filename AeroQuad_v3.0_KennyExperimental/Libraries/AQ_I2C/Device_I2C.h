/*
  AeroQuad v2.2 - Feburary 2011
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

#ifndef _AEROQUAD_DEVICE_I2C_H_
#define _AEROQUAD_DEVICE_I2C_H_

// I2C functions
#include <Wire.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
 #else
  #include <WProgram.h>
#endif

void sendByteI2C(int deviceAddress, byte dataValue);
byte readByteI2C(int deviceAddress);
int readWordI2C(int deviceAddress);
int readWordWaitI2C(int deviceAddress);
int readReverseWordI2C(int deviceAddress);
byte readWhoI2C(int deviceAddress);
void updateRegisterI2C(int deviceAddress, byte dataAddress, byte dataValue);

int readShortI2C();
int readShortI2C(int deviceAddress);
int readReverseShortI2C();
int readReverseShortI2C(int deviceAddress);

#endif