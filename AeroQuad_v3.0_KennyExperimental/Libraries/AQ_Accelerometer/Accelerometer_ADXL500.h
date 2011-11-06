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

#ifndef _AEROQUAD_ACCELEROMETER_IDG500_H_
#define _AEROQUAD_ACCELEROMETER_IDG500_H_

#include <Accelerometer.h>

float accelAref;
  
void initializeAccel() {
  accelScaleFactor = G_2_MPS2((3.0/1024.0) / 0.300);  // force aref to 3.0 for v1.7 shield
}

void setAccelAref(float aref) {
  accelAref = aref;
  accelScaleFactor = G_2_MPS2((accelAref/1024.0) / 0.300);
}
  
void measureAccel() {

  meterPerSec[XAXIS] = (analogRead(1)    - accelZero[PITCH]) * accelScaleFactor;
  meterPerSec[YAXIS] = (accelZero[ROLL]  - analogRead(0))    * accelScaleFactor;
  meterPerSec[ZAXIS] = (accelZero[ZAXIS] - analogRead(2))   * accelScaleFactor;
}

void measureAccelSum() {
  // do nothing here since it's already oversample in the APM_ADC class
}

void evaluateMetersPerSec() {
  // do nothing here since it's already oversample in the APM_ADC class
}

void calibrateAccel() {
  int findZero[FINDZERO];

  for (int i=0; i<FINDZERO; i++) {
    findZero[i] = analogRead(1);
    delay(2);
  }
  accelZero[XAXIS] = findMedianInt(findZero, FINDZERO);
  for (int i=0; i<FINDZERO; i++) {
    findZero[i] = analogRead(0);
    delay(2);
  }
  accelZero[YAXIS] = findMedianInt(findZero, FINDZERO);
  for (int i=0; i<FINDZERO; i++) {
    findZero[i] = analogRead(2);
	delay(2);
  }
  accelZero[ZAXIS] = findMedianInt(findZero, FINDZERO);
	
  // store accel value that represents 1g
  measureAccel();
  accelOneG = -meterPerSec[ZAXIS];
  // replace with estimated Z axis 0g value
  accelZero[ZAXIS] = (accelZero[XAXIS] + accelZero[YAXIS]) / 2;
}


#endif
