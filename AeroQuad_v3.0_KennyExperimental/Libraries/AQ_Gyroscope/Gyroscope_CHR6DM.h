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

#ifndef _AEROQUAD_GYROSCOPE_CHR6DM_H_
#define _AEROQUAD_GYROSCOPE_CHR6DM_H_

#include <Gyroscope.h>
#include <Platform_CHR6DM.h>
#include <AQMath.h>


CHR6DM *gyroChr6dm;
  
void initializeGyro(){
}
  
void measureGyro() {
  
  int gyroADC[3];
  gyroADC[ROLL] = gyroChr6dm->data.rollRate - gyroZero[ROLL]; //gx yawRate
  gyroADC[PITCH] = gyroZero[PITCH] - gyroChr6dm->data.pitchRate; //gy pitchRate
  gyroADC[YAW] = gyroChr6dm->data.yawRate - gyroZero[ZAXIS]; //gz rollRate

  gyroRate[ROLL] = filterSmooth(gyroADC[ROLL], gyroRate[ROLL], gyroSmoothFactor); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
  gyroRate[PITCH] = filterSmooth(gyroADC[PITCH], gyroRate[PITCH], gyroSmoothFactor); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
  gyroRate[YAW] = filterSmooth(gyroADC[YAW], gyroRate[YAW], gyroSmoothFactor); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1

  // Measure gyro heading
  long int currentTime = micros();
  if (gyroRate[YAW] > radians(1.0) || gyroRate[YAW] < radians(-1.0)) {
    gyroHeading += gyroRate[YAW] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
  }
  gyroLastMesuredTime = currentTime;
}

void measureGyroSum() {
  // do nothing here since it's already oversample in the APM_ADC class
}

void evaluateGyroRate() {
  // do nothing here since it's already oversample in the APM_ADC class
}

void calibrateGyro() {
  float zeroXreads[FINDZERO];
  float zeroYreads[FINDZERO];
  float zeroZreads[FINDZERO];

  for (int i=0; i<FINDZERO; i++) {
    gyroChr6dm->read();
    zeroXreads[i] = gyroChr6dm->data.rollRate;
    zeroYreads[i] = gyroChr6dm->data.pitchRate;
    zeroZreads[i] = gyroChr6dm->data.yawRate;
  }

  gyroZero[XAXIS] = findMedianFloat(zeroXreads, FINDZERO);
  gyroZero[YAXIS] = findMedianFloat(zeroYreads, FINDZERO);
  gyroZero[ZAXIS] = findMedianFloat(zeroZreads, FINDZERO);
}


#endif  // #ifndef _AEROQUAD_GYROSCOPE_CHR6DM_H_

