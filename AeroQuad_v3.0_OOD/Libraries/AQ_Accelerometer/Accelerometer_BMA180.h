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

#ifndef _AEROQUAD_ACCELEROMETER_BMA180_H_
#define _AEROQUAD_ACCELEROMETER_BMA180_H_

#include <Accelerometer.h>
#include <Device_I2C.h>

#define ACCEL_ADDRESS 0x40
#define ACCEL_ADDRESS_ALT 0x41
#define ACCEL_IDENTITY 0x03
#define ACCEL_RESET_REGISTER 0x10
#define ACCEL_TRIGER_RESET_VALUE 0xB6
#define ACCEL_ENABLE_WRITE_CONTROL_REGISTER 0x0D
#define ACCEL_CONTROL_REGISTER 0x10
#define ACCEL_BW_TCS 0x20
#define ACCEL_LOW_PASS_FILTER_REGISTER 0x20
#define ACCEL_10HZ_LOW_PASS_FILTER_VALUE 0x0F
#define ACCEL_OFFSET_REGISTER 0x35
#define ACCEL_READ_ROLL_ADDRESS 0x02
#define ACCEL_READ_PITCH_ADDRESS 0x04
#define ACCEL_READ_YAW_ADDRESS 0x06

class Accelerometer_BMA180 : public Accelerometer {
private:
  int deviceAddress;
  float meterPerSecSample[3];
  int countSample[3];
  
public:
  Accelerometer_BMA180(boolean useSeccondAddress = false) : Accelerometer() {
    accelScaleFactor = G_2_MPS2(1.0/4096.0);  //  g per LSB @ +/- 2g range
    smoothFactor = 1.0;
    deviceAddress = ACCEL_ADDRESS;
    if (useSeccondAddress)
	  deviceAddress = ACCEL_ADDRESS_ALT;
    for( byte axis = 0; axis < 3; axis++) {
      meterPerSecSample[axis] = 0;
      countSample[axis] = 1;
    }
  }

  void initialize(void) {
    if (readWhoI2C(deviceAddress) == ACCEL_IDENTITY) {  // page 52 of datasheet
      updateRegisterI2C(deviceAddress, ACCEL_RESET_REGISTER, ACCEL_TRIGER_RESET_VALUE); 					//reset device
      delay(10);  																							//sleep 10 ms after reset (page 25)

      // In datasheet, summary register map is page 21
      // Low pass filter settings is page 27
      // Range settings is page 28
      updateRegisterI2C(deviceAddress, ACCEL_ENABLE_WRITE_CONTROL_REGISTER, ACCEL_CONTROL_REGISTER); 		//enable writing to control registers
      sendByteI2C(deviceAddress, ACCEL_BW_TCS); 															// register bw_tcs (bits 4-7)
      byte data = readByteI2C(deviceAddress); 																// get current register value
      updateRegisterI2C(deviceAddress, ACCEL_LOW_PASS_FILTER_REGISTER, data & ACCEL_10HZ_LOW_PASS_FILTER_VALUE); 	// set low pass filter to 10Hz (value = 0000xxxx)

      // From page 27 of BMA180 Datasheet
      //  1.0g = 0.13 mg/LSB
      //  1.5g = 0.19 mg/LSB
      //  2.0g = 0.25 mg/LSB
      //  3.0g = 0.38 mg/LSB
      //  4.0g = 0.50 mg/LSB
      //  8.0g = 0.99 mg/LSB
      // 16.0g = 1.98 mg/LSB
      sendByteI2C(deviceAddress, ACCEL_OFFSET_REGISTER); 													// register offset_lsb1 (bits 1-3)
      data = readByteI2C(deviceAddress);
      data &= 0xF1;
      data |= 0x04; // Set range select bits for +/-2g
      updateRegisterI2C(deviceAddress, ACCEL_OFFSET_REGISTER, data);	
      Serial.println("init Accelerometer_BMA180: done"); 
    } else
      Serial.println("init Accelerometer_BMA180: failed");
  }

  
  void measure(void) {
    Wire.beginTransmission(deviceAddress);
    Wire.send(ACCEL_READ_ROLL_ADDRESS);
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, 6);
  
    int accelADC; 
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      if (axis == XAXIS)
        accelADC = (readReverseShortI2C() >> 2) - zero[axis];
      else
        accelADC = zero[axis] - (readReverseShortI2C() >> 2);
        
      meterPerSec[axis] = filterSmooth(accelADC * accelScaleFactor, meterPerSec[axis], smoothFactor);
      meterPerSecSample[axis] += accelADC * accelScaleFactor;
      countSample[axis]++;
    }  
  }

  void calibrate(void) {
    int findZero[FINDZERO];
    int dataAddress;
    
    for (byte calAxis = XAXIS; calAxis < ZAXIS; calAxis++) {
      if (calAxis == XAXIS) dataAddress = ACCEL_READ_ROLL_ADDRESS;
      if (calAxis == YAXIS) dataAddress = ACCEL_READ_PITCH_ADDRESS;
      if (calAxis == ZAXIS) dataAddress = ACCEL_READ_YAW_ADDRESS;
      for (int i=0; i<FINDZERO; i++) {
        sendByteI2C(deviceAddress, dataAddress);
        findZero[i] = readReverseShortI2C(deviceAddress) >> 2; // last two bits are not part of measurement
        delay(10);
      }
      zero[calAxis] = findMedianInt(findZero, FINDZERO);
    }

    // replace with estimated Z axis 0g value
    zero[ZAXIS] = (zero[XAXIS] + zero[PITCH]) / 2;
    // store accel value that represents 1g
    measure();
    oneG = -meterPerSec[ZAXIS];
  }
  
  float getMeterPerSecSample(byte axis) {
    //return meterPerSec[axis];
    float data = meterPerSecSample[axis] / countSample[axis];
    //Serial.print(countSample[axis]); Serial.print(" / "); Serial.println(data);
    countSample[axis] = 0;
    meterPerSecSample[axis] = 0;
    return data;
  }

};
#endif
