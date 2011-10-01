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

// I2C functions

void sendByteI2C(int deviceAddress, byte dataValue) {
  Wire.beginTransmission(deviceAddress);
  Wire.send(dataValue);
  Wire.endTransmission();
}

byte readByteI2C(int deviceAddress) {
    Wire.requestFrom(deviceAddress, 1);
    return Wire.receive();
}

int readWordI2C()  __attribute__ ((noinline));
int readWordI2C() {
  return (Wire.receive() << 8) | Wire.receive();
}

int readShortI2C()  __attribute__ ((noinline));
int readShortI2C() {
  return (signed short)readWordI2C();
}

int readShortI2C(int deviceAddress) {
  Wire.requestFrom(deviceAddress, 2);
  return readShortI2C();
}

int readReverseShortI2C()  __attribute__ ((noinline));
int readReverseShortI2C() {
  return (signed short)( Wire.receive() | (Wire.receive() << 8));
}

int readReverseShortI2C(int deviceAddress) {
  Wire.requestFrom(deviceAddress, 2);
  return readReverseShortI2C();
}

byte readWhoI2C(int deviceAddress) {
  // read the ID of the I2C device
  Wire.beginTransmission(deviceAddress);
  Wire.send(0x00);
  Wire.endTransmission();
  delay(100);
  Wire.requestFrom(deviceAddress, 1);
  return Wire.receive();
}

void updateRegisterI2C(int deviceAddress, byte dataAddress, byte dataValue) {
  Wire.beginTransmission(deviceAddress);
  Wire.send(dataAddress);
  Wire.send(dataValue);
  Wire.endTransmission();
}

void updateRegisterI2C(int deviceAddress, byte dataAddress, byte dataValue) {
  Wire.beginTransmission(deviceAddress);
  Wire.send(dataAddress);
  Wire.send(dataValue);
  Wire.endTransmission();
}
