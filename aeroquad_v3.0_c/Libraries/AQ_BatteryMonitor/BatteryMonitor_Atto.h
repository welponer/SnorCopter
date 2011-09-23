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

#ifndef _AQ_BATTERY_MONITOR_ATTO_
#define _AQ_BATTERY_MONITOR_ATTO_

//#include "BatteryMonitor.h"
#include <WProgram.h>

// ***********************************************************************************
// ************************ BatteryMonitor Atto          *****************************
// ***********************************************************************************

#define SENSOR_VPIN A0      // Ain 0 (universal to every Arduino), pin 55 on Mega (1280)
#define SENSOR_IPIN A2      
#define SENSOR_GPIN A4      
//#define SENSOR_VSCALE 49.44 /5*2.56 // 45A board
#define SENSOR_VSCALE 12.95 /5*2.56 // 90A board
//#define SENSOR_VSCALE 12.99 /5*2.56 // 180A board  
//#define SENSOR_ISCALE 14.9 /5*2.56 //45 Amp board
#define SENSOR_ISCALE 7.4 /5*2.56 //90 Amp board
//#define SENSOR_ISCALE 3.7 /5*2.56 //180 Amp board

#define OK 0
#define WARNING 1
#define ALARM 2
#define BATTERY_WARN 10.2
#define BATTERY_ALARM 9.5




#define LEDDELAY 200


float batteryVoltage = BATTERY_WARN + 2;
float batteryCurrent = 0;
float batteryCharge = 0;
byte batteryStatus = OK;
  
void measureBatteryVoltage(boolean);
//const float readBatteryVoltage(byte);

void initializeBatteryMonitor() {
  analogReference(INTERNAL2V56); //use Oilpan 3V3 AREF or if wanted, define DEFAULT here to use VCC as reference and define that voltage in BatteryReadArmLed.h
  
  pinMode(SENSOR_GPIN, OUTPUT);
  digitalWrite(SENSOR_GPIN, LOW);
  
  measureBatteryVoltage(false);
}

void lowBatteryEvent(byte level) {  // <- this logic by Jose Julio
  static byte batteryCounter=0;
  byte freq;
/*
  if (level == OK) {
    ledsON();
    autoDescent = 0; //reset autoAscent if battery is good
  }
  else {
    batteryCounter++;
    if (level == WARNING) freq = 40;  //4 seconds wait
    else freq = 5; //0.5 second wait
    if (batteryCounter < 2) ledsOFF();  //indicate with led's everytime autoDescent kicks in
    #if defined(AltitudeHold)
      if (throttle > 1400) holdAltitude -= 0.2; //-0.2m in 2 fixed rates, one where battery < 10.8V and one where battery < 10.2V, only done if in altitude hold mode
    #else
      if (throttle > 1400) autoDescent -= 2; //will remove 2µs throttle every time led's blink in two speeds (10.8 and 10.2V) as long as there is throttle to lower
    #endif
    else if (batteryCounter < freq) ledsON();
    else batteryCounter = 0;
  } */
}




void measureBatteryVoltage(boolean armed) {
  batteryVoltage = filterSmooth(analogRead(SENSOR_VPIN) / SENSOR_VSCALE, batteryVoltage, 0.5);
  batteryCurrent = filterSmooth(analogRead(SENSOR_IPIN)*1000.0 / SENSOR_ISCALE, batteryCurrent, 0.5);

    batteryCharge += batteryCurrent * ((currentTime - previousTime) / 1000000.0) / 3.6; // mAh
  if (armed) {
    if (batteryVoltage < BATTERY_WARN) 
      batteryStatus = WARNING;
    if (batteryVoltage < BATTERY_ALARM) 
	  batteryStatus = ALARM;
  }
  else
    batteryStatus = OK;
  lowBatteryEvent(batteryStatus);
}




#endif