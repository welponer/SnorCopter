/*
  AeroQuad Maple32 v3.0 - October 2011
  Copyright (c) 2011 Mattias Welponer.  All rights reserved.
  An Open Source Maple STM32 based multicopter.
 
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


#ifndef _AEROQUAD_MAGNETOMETER_HMC5883L_H_
#define _AEROQUAD_MAGNETOMETER_HMC5883L_H_

#include "Compass.h"
#include <WProgram.h>

#define COMPASS_ADDRESS 0x1E
#define COMPASS_EXPECTED_XY 1264.4f // xy - Gain 2 (0x20): 1090 LSB/Ga * 1.16 Ga - see HMC5883L specs
#define COMPASS_EXPECTED_Z 1177.2f // z - Gain 2 (0x20): 1090 LSB/Ga * 1.08 Ga    

class Magnetometer_HMC5883L : public Compass {
private:
  float magCalibration[3];
  
public:
  Magnetometer_HMC5883L() : Compass() {
    magCalibration[XAXIS] = 1.0;
    magCalibration[YAXIS] = 1.0;
    magCalibration[ZAXIS] = 1.0;
  }

  void initialize() {
    byte numAttempts = 0;
    bool success = false;
    delay(10);        // Power up delay
   
    magCalibration[XAXIS] = 1.0;
    magCalibration[YAXIS] = 1.0;
    magCalibration[ZAXIS] = 1.0;
    
    while (success == false && numAttempts < 5 ) {
      numAttempts++;
      updateRegisterI2C(COMPASS_ADDRESS, 0x00, 0x11);  // Set positive bias configuration for sensor calibraiton
      delay(10);
      updateRegisterI2C(COMPASS_ADDRESS, 0x01, 0x20); // Set +/- 1G gain
      delay(10);
      updateRegisterI2C(COMPASS_ADDRESS, 0x02, 0x01);  // Perform single conversion
      delay(10);
 
      measure(0.0, 0.0);                    // Read calibration data
      delay(10);
      if ( fabs(measuredMagX) > 500.0 && fabs(measuredMagX) < (COMPASS_EXPECTED_XY + 300) \
          && fabs(measuredMagY) > 500.0 && fabs(measuredMagY) < (COMPASS_EXPECTED_XY + 300) \
          && fabs(measuredMagZ) > 500.0 && fabs(measuredMagZ) < (COMPASS_EXPECTED_Z + 300)) {
        magCalibration[XAXIS] = fabs(COMPASS_EXPECTED_XY / measuredMagX);
        magCalibration[YAXIS] = fabs(COMPASS_EXPECTED_XY / measuredMagY);
        magCalibration[ZAXIS] = fabs(COMPASS_EXPECTED_Z / measuredMagZ);    
        success = true;
      }
      updateRegisterI2C(COMPASS_ADDRESS, 0x00, 0x10);  // Set 10hz update rate and normal operaiton
      delay(50);
      updateRegisterI2C(COMPASS_ADDRESS, 0x02, 0x00); // Continuous Update mode
      delay(50);                           // Mode change delay (1/Update Rate) **
    }
    measure(0.0, 0.0);  // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
    if (success) Serial.println("init Magnetometer_HMC5883L: done"); 
    else Serial.println("init Magnetometer_HMC5883L: failed"); 
  }

  void measure(float roll, float pitch) {
    float magX;
    float magY;
    float tmp;
    
    sendByteI2C(COMPASS_ADDRESS, 0x03);
    Wire.requestFrom(COMPASS_ADDRESS, 6);
    measuredMagX =  readShortI2C() * magCalibration[XAXIS];
    // xchange Y and Z
    measuredMagZ = -readShortI2C() * magCalibration[ZAXIS];
    measuredMagY = -readShortI2C() * magCalibration[YAXIS];
    Wire.endTransmission();

    float cosRoll =  cos(roll);
    float sinRoll =  sin(roll);
    float cosPitch = cos(pitch);
    float sinPitch = sin(pitch);

    magX = ((float)measuredMagX * magScale[XAXIS] + magOffset[XAXIS]) * cosPitch + \
         ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * sinRoll * sinPitch + \
         ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * cosRoll * sinPitch;
    magY = ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * cosRoll - \
         ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * sinRoll;
    tmp  = sqrt(magX * magX + magY * magY);
   
    hdgX = magX / tmp;
    hdgY = -magY / tmp;
  }
};

#endif