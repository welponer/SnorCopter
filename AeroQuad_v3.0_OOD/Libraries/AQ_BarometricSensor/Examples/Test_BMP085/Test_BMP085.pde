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

#include <Wire.h>
#include <Device_I2C.h>
#include <AQMath.h>

#include <BarometricSensor.h>
#include <BarometricSensor_BMP085.h>

BarometricSensor_BMP085 barometricSensor;
unsigned long timer = 0;

void setup() {
  
  Serial.begin(115200);
  Serial.println("Barometric sensor library test (BMP085)");

  Wire.begin();
  
  barometricSensor.initialize();  
}

void loop() {
  
  if((millis() - timer) > 50) // 20Hz
  {
    timer = millis();
    barometricSensor.measure();
    
    Serial.print("altitude : ");
    Serial.print(barometricSensor.getAltitude());
    Serial.println();
  }
}
