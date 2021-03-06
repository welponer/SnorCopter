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

#ifndef _AEROQUAD_ACCELEROMETER_H_
#define _AEROQUAD_ACCELEROMETER_H_

#include <WProgram.h>
#include <Axis.h>

class Accelerometer {
protected:
  float accelScaleFactor;
  float smoothFactor;
  float oneG;
  float meterPerSec[3];
  float zero[3];
  
public:  
  Accelerometer() {
    for (int i = 0; i < 3; i++) {
      meterPerSec[i] = 0;
      zero[i] = 0;
    }
    oneG = 9.9;
    accelScaleFactor = 1.0;
    smoothFactor = 1.0;
  };

  virtual void initialize() {}
  virtual void calibrate() {}
  virtual void measure() {}
    
  const float getSmoothFactor() {
    return smoothFactor;
  }

  void setSmoothFactor(float value) {
    smoothFactor = value;
  }

  void setOneG(float oneG) {
    this->oneG = oneG;
  }

  float getOneG() {
    return oneG;
  }

  float getMeterPerSec(byte axis) {
    return meterPerSec[axis];
  }
  
  virtual float getMeterPerSecSample(byte axis) { 
    return 0;
  }

  float getZero(byte axis) {
    return zero[axis];
  }

  void setZero(byte axis, float zero) {
    this->zero[axis] = zero;
  }
  
};
#endif