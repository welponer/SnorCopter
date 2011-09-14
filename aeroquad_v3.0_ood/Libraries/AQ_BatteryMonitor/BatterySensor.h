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

#ifndef _AQ_BATTERY_SENSOR_
#define _AQ_BATTERY_SENSOR_

#include <WProgram.h>

#define SENSOR_VPIN A0      // Ain 0 (universal to every Arduino), pin 55 on Mega (1280)
#define SENSOR_IPIN A2      
#define SENSOR_GPIN A4      
//#define SENSOR_VSCALE 49.44/5*2.56 // 45A board 5V
#define SENSOR_VSCALE 12.95/5*2.56 // 90A board
//#define SENSOR_VSCALE 12.99/5*2.56 // 180A board  
//#define SENSOR_ISCALE 14.9/5*2.56 //45 Amp board
#define SENSOR_ISCALE 7.4/5*2.56 //90 Amp board
//#define SENSOR_ISCALE 3.7/5*2.56 //180 Amp board

#define OK 0
#define WARNING 1
#define ALARM 2
#define BATTERY_WARN 10.2
#define BATTERY_ALARM 9.5

class BatterySensor {
private:
  float batteryVoltage;
  float batteryCurrent;
  byte batteryStatus;

  unsigned long previousTime;
  float batteryCharge;
  
public:

  BatterySensor() {
    batteryVoltage = BATTERY_WARN + 2;
    batteryCurrent = 0;
    batteryCharge = 0;
    batteryStatus = OK;
  }

  void initialize() {
    analogReference(INTERNAL2V56); //use Oilpan 3V3 AREF or if wanted, define DEFAULT here to use VCC as reference and define that voltage in BatteryReadArmLed.h
    pinMode(SENSOR_GPIN, OUTPUT);
    digitalWrite(SENSOR_GPIN, LOW);

    measure(false);
  }
  
  void lowBatteryEvent(byte) {}

  void measure(boolean armed) {
    previousTime = currentTime;
    unsigned long currentTime = micros();
  
    batteryVoltage = filterSmooth(analogRead(SENSOR_VPIN) / SENSOR_VSCALE, batteryVoltage, 0.5);
    batteryCurrent = filterSmooth(analogRead(SENSOR_IPIN)*1000.0 / SENSOR_ISCALE, batteryCurrent, 0.5);

    batteryCharge += batteryCurrent * ((currentTime - previousTime) / 1000000.0) / 3.6; // mAh
/*    
    Serial.print("BatterySensor: ");
    Serial.print(batteryVoltage);
    Serial.print("; ");
    Serial.print(batteryCurrent);
    Serial.print("; ");
    Serial.println(batteryCharge);
 */   
    
    
 //   if (armed) {
      batteryStatus = OK;
      if (batteryVoltage < BATTERY_WARN) 
	    batteryStatus = WARNING;
      if (batteryVoltage < BATTERY_ALARM) 
	    batteryStatus = ALARM;
 //   }
 //   else
      batteryStatus = OK;
    lowBatteryEvent(batteryStatus);
  }
  
  const float getBatteryVoltage() {
    return batteryVoltage;
  }
};


#endif