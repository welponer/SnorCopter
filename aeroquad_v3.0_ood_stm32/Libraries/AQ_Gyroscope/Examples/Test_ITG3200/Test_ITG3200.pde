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
#include <APM_ADC.h>          // @see Kenny, Arduino IDE compiliation bug
#include <Platform_CHR6DM.h>  // @see Kenny, Arduino IDE compiliation bug
#define GYRO_ALTERNATE TRUE
#include <AQMath.h>
#include <Device_I2C.h>
#include <Gyroscope_ITG3200.h>
#include <Axis.h>

unsigned long timer;

Gyroscope_ITG3200 gyro;

void setup()
{
  SerialUSB.begin();
  SerialUSB.println("Gyroscope library test (ITG3200)");

  Wire.begin();

  gyro.initialize();
  gyro.calibrate();
  timer = millis();
}

void loop(void) 
{
  if((millis() - timer) > 100) // 100Hz
  {
    timer = millis();
    gyro.measure();
    
    SerialUSB.print("Roll: ");
    SerialUSB.print(degrees(gyro.getRadPerSec(ROLL)));
    SerialUSB.print(" Pitch: ");
    SerialUSB.print(degrees(gyro.getRadPerSec(PITCH)));
    SerialUSB.print(" Yaw: ");
    SerialUSB.print(degrees(gyro.getRadPerSec(YAW)));
    SerialUSB.print(" Heading: ");
    SerialUSB.print(degrees(gyro.getHeading()));
    SerialUSB.println();
  }
}

