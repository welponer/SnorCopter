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

#ifndef _AEROQUAD_GYROSCOPE_ITG3200_H_
#define _AEROQUAD_GYROSCOPE_ITG3200_H_

#include <Gyroscope.h>
#include <Device_I2C.h>

#define ITG3200_ADDRESS					0x69
#define ITG3200_ADDRESS_ALT				0x68
#define ITG3200_MEMORY_ADDRESS			0x1D
#define ITG3200_BUFFER_SIZE				6
#define ITG3200_RESET_ADDRESS			0x3E
#define ITG3200_RESET_VALUE				0x80
#define ITG3200_LOW_PASS_FILTER_ADDR	0x16
#define ITG3200_LOW_PASS_FILTER_10HZ	0x1D	// 10Hz low pass filter
#define ITG3200_OSCILLATOR_ADDR			0x3E
#define ITG3200_OSCILLATOR_VALUE		0x01	// use X gyro oscillator
#define ITG3200_SCALE_TO_RADIANS		823.626831 // 14.375 LSBs per �/sec, / Pi / 180

class Gyroscope_ITG3200 : public Gyroscope {
private:
  int deviceAddress;
  float rateSample[3];
  int countSample[3];
  
public:
  Gyroscope_ITG3200(boolean useSeccondAddress = false) : Gyroscope() {
    scaleFactor = radians(1.0 / 14.375);  //  ITG3200 14.375 LSBs per �/sec
    smoothFactor = 1.0;
    deviceAddress = ITG3200_ADDRESS;
    if (useSeccondAddress)
	  deviceAddress = ITG3200_ADDRESS_ALT;
	for( byte axis = 0; axis < 3; axis++) 
	  countSample[axis] = 1;
  }
/*  
  Gyroscope_ITG3200(byte address = 0x69) {
    scaleFactor = radians(1.0 / 14.375);  //  ITG3200 14.375 LSBs per �/sec
    smoothFactor = 1.0;
    deviceAddress = address;
  }
*/  
  void initialize(void) {
    updateRegisterI2C(deviceAddress, ITG3200_RESET_ADDRESS, ITG3200_RESET_VALUE); // send a reset to the device
    updateRegisterI2C(deviceAddress, ITG3200_LOW_PASS_FILTER_ADDR, ITG3200_LOW_PASS_FILTER_10HZ); // 10Hz low pass filter
    updateRegisterI2C(deviceAddress, ITG3200_RESET_ADDRESS, ITG3200_OSCILLATOR_VALUE); // use internal oscillator 
    Serial.println("init Gyroscope_ITG3200: done"); 
  }
    
  void measure(void) {
    int gyroADC[3];
    
    sendByteI2C(deviceAddress, ITG3200_MEMORY_ADDRESS);
    Wire.requestFrom(deviceAddress, ITG3200_BUFFER_SIZE);
    
    // The following 3 lines read the gyro and assign it's data to gyroADC
    // in the correct order and phase to suit the standard shield installation
    // orientation.  See TBD for details.  If your shield is not installed in this
    // orientation, this is where you make the changes.
    gyroADC[ROLL]  = readShortI2C() - zero[ROLL];
    gyroADC[PITCH] = zero[PITCH] - readShortI2C();
    gyroADC[YAW]   = zero[YAW] - readShortI2C(); 

    for (byte axis = 0; axis <= YAW; axis++) {
      rate[axis] = filterSmooth(gyroADC[axis] * scaleFactor, rate[axis], smoothFactor);
      rateSample[axis] += gyroADC[axis] * scaleFactor;
      countSample[axis]++;
    }
 
    // Measure gyro heading
    long int currentTime = micros();
    if (rate[YAW] > radians(1.0) || rate[YAW] < radians(-1.0)) {
      heading += rate[YAW] * ((currentTime - lastMesuredTime) / 1000000.0);
    }
    lastMesuredTime = currentTime;
  }

  void calibrate() {
    int findZero[FINDZERO];
    
    for (byte axis = 0; axis < 3; axis++) {
      for (int i=0; i<FINDZERO; i++) {
	    sendByteI2C(deviceAddress, (axis * 2) + ITG3200_MEMORY_ADDRESS);
        findZero[i] = readShortI2C(deviceAddress);
        delay(10);
      }
      zero[axis] = findMedianInt(findZero, FINDZERO);
    }
  }
  
  float getRadPerSecSample(byte axis) {
    float data = rateSample[axis] / countSample[axis];
    countSample[axis] = 0;
    rateSample[axis] = 0;
    return data;
  }
  
};
#endif
