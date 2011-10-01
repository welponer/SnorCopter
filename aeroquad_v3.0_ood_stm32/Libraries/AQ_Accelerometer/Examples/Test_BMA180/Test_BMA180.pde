/*
  AeroQuad v3.0 - March 2011
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

#define Serial SerialUSB
#define ACCEL_ALTERNATE TRUE
#include <Wire.h>
#include <Device_I2C.h>
#include <Axis.h>
#include <AQMath.h>
#include <Accelerometer_BMA180.h>

unsigned long timer;
Accelerometer_BMA180 accel;



void setup() {
  
  Serial.begin();
  Serial.println("Accelerometer library test (BMA180)");

  Wire.begin( 0, PORTI2C2, I2C_FAST_MODE);
 
  accel.initialize();  
  accel.calibrate();
  delay(100);
}

void loop() {
  /*
  for( int i = 0; i < 127; i++) {
    Serial.print(i, HEX);
    Serial.print(" - ");
    Serial.println(readWhoI2C(i));
    delay(20);
  }
  delay(10000);
*/
  if((millis() - timer) > 100) // 100Hz
  {
    timer = millis();
    Serial.print("BMP180: ");
    accel.measure();
    
    Serial.print("Roll: ");
    Serial.print(accel.getMeterPerSec(XAXIS));
    Serial.print(" Pitch: ");
    Serial.print(accel.getMeterPerSec(YAXIS));
    Serial.print(" Zaxis: ");
    Serial.print(accel.getMeterPerSec(ZAXIS));
    Serial.println();
  }

}
