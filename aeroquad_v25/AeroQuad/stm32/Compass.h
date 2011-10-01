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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This class updated by jihlein

class Compass {
public:
  float magMax[3];
  float magMin[3];
  float magCalibration[3];
  float magScale[3];
  float magOffset[3];
  float hdgX;
  float hdgY;
  int   compassAddress;
  float measuredMagX;
  float measuredMagY;
  float measuredMagZ;
  
  Compass(void) {}

  const float getHdgXY(byte axis) {
    if (axis == XAXIS) return hdgX;
    if (axis == YAXIS) return hdgY;
	return 0.0;
  }

  const int getRawData(byte axis) {
    if (axis == XAXIS) return measuredMagX;
    if (axis == YAXIS) return measuredMagY;
    if (axis == ZAXIS) return measuredMagZ;
	return 0.0;
  }
  
  void setMagCal(byte axis, float maxValue, float minValue) {
    magMax[axis] = maxValue;
    magMin[axis] = minValue;
    // Assume max/min is scaled to +1 and -1
    // y2 = 1, x2 = max; y1 = -1, x1 = min
    // m = (y2 - y1) / (x2 - x1)
    // m = 2 / (max - min)
    magScale[axis] = 2.0 / (magMax[axis] - magMin[axis]);
    // b = y1 - mx1; b = -1 - (m * min)
    magOffset[axis] = -(magScale[axis] * magMin[axis]) - 1;
  }
  
  const float getMagMax(byte axis) {
    return magMax[axis];
  }
  
  const float getMagMin(byte axis) {
    return magMin[axis];
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Magnetometer (HMC5843)
////////////////////////////////////////////////////////////////////////////////

class Magnetometer_HMC5843 : public Compass {
public: 
  Magnetometer_HMC5843() : Compass() {
    compassAddress = 0x1E;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Initialize AeroQuad Mega v2.0 Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void initialize(void) {
    byte numAttempts = 0;
    bool success = false;
    delay(10);                             // Power up delay **
   
    magCalibration[XAXIS] = 1.0;
    magCalibration[YAXIS] = 1.0;
    magCalibration[ZAXIS] = 1.0;
    
#ifdef DEBUG_I2C
    Serial.print("Looking for HMC 5843 ...  ");
#endif
    while (success == false && numAttempts < 5 ) {
      
      numAttempts++;
   
      updateRegisterI2C(0x1E, 0x00, 0x11);  // Set positive bias configuration for sensor calibraiton
      delay(50);
   
      updateRegisterI2C(compassAddress, 0x01, 0x20); // Set +/- 1G gain
      delay(10);

      updateRegisterI2C(0x1E, 0x02, 0x01);  // Perform single conversion
      delay(10);
   
      measure(0.0, 0.0);                    // Read calibration data
      delay(10);
   
      if ( fabs(measuredMagX) > 500.0 && fabs(measuredMagX) < 1000.0 \
          && fabs(measuredMagY) > 500.0 && fabs(measuredMagY) < 1000.0 \
          && fabs(measuredMagZ) > 500.0 && fabs(measuredMagZ) < 1000.0) {
        magCalibration[XAXIS] = fabs(715.0 / measuredMagX);
        magCalibration[YAXIS] = fabs(715.0 / measuredMagY);
        magCalibration[ZAXIS] = fabs(715.0 / measuredMagZ);
    
        success = true;
      }
   
      updateRegisterI2C(compassAddress, 0x00, 0x10);  // Set 10hz update rate and normal operaiton
      delay(50);

      updateRegisterI2C(compassAddress, 0x02, 0x00); // Continuous Update mode
      delay(50);                           // Mode change delay (1/Update Rate) **
    }

#ifdef DEBUG_I2C
    Serial.println("done");
#endif

	measure(0.0, 0.0);  // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  // Measure AeroQuad Mega v2.0 Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void measure(float roll, float pitch) {
    float magX;
    float magY;
    float tmp;
    
    sendByteI2C(compassAddress, 0x03);
    Wire.requestFrom(compassAddress, 6);

    measuredMagX =   readShortI2C() * magCalibration[XAXIS];
    measuredMagY = - readShortI2C() * magCalibration[YAXIS];
    measuredMagZ = - readShortI2C() * magCalibration[ZAXIS];

    Wire.endTransmission();

    float cosRoll  = cos(roll);
    float sinRoll  = sin(roll);
    float cosPitch = cos(pitch);
    float sinPitch = sin(pitch);

    magX =   (measuredMagX * magScale[XAXIS] + magOffset[XAXIS]) * cosPitch
           + (measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * sinRoll * sinPitch 
           + (measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * cosRoll * sinPitch;
           
    magY =   (measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * cosRoll 
           - (measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * sinRoll;

    tmp  = sqrt(magX * magX + magY * magY);
    
    hdgX =   magX / tmp;
    hdgY = - magY / tmp;
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

