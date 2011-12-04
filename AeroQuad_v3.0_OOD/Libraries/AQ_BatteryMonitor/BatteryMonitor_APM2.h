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

#ifndef _AQ_BATTERY_MONITOR_APM2_
#define _AQ_BATTERY_MONITOR_APM2_

#include "BatteryMonitor.h"
#include <WProgram.h>

#if defined (__AVR_ATmega328P__)
  #define BUZZERPIN 12
#else
  #define BUZZERPIN 49
#endif

class BatteryMonitor_APM2 : public BatteryMonitor {
private:
  byte state, firstAlarm;
  float diode; // raw voltage goes through diode on Arduino
  float batteryScaleFactor;
  long currentBatteryTime, previousBatteryTime;

public:
  BatteryMonitor_APM2() : BatteryMonitor() {}

  void initialize(float diodeValue = 0.0) {
    float R1   = 15000;
    float R2   =  7500;
    float Aref =     5.0;
    batteryScaleFactor = ((Aref / 1024.0) * ((R1 + R2) / R2));
    diode = diodeValue;
    analogReference(DEFAULT);
 //   pinMode(BUZZERPIN, OUTPUT); // connect a 12V buzzer to buzzer pin
 //   digitalWrite(BUZZERPIN, LOW);

    R1   = 30.48; //10050; //the SMD 10k resistor measured with DMM
    R2   =  15.24; //3260; //3k3 user mounted resistor measured with DMM
    Aref = 4.98F; 
    batteryScaleFactor = ((Aref / 1024.0) * ((R1 + R2) / R2));
    diode = 0.306F;
    #define BUZZERPIN 35
    #define BUZZERGND 40
    pinMode(BUZZERPIN, OUTPUT);
    pinMode(BUZZERGND, OUTPUT);
    digitalWrite(BUZZERGND, LOW);  
    pinMode(A14, OUTPUT);  digitalWrite(A14, HIGH);  
    pinMode(A12, OUTPUT);  digitalWrite(A12, HIGH);  
    pinMode(A10, OUTPUT);  digitalWrite(A10, HIGH);  
    pinMode(A8, OUTPUT);  digitalWrite(A8, HIGH); 
                
    previousBatteryTime = millis();
    state = LOW;
    firstAlarm = OFF;
    
    lowVoltageWarning = 10.0; //10.8;
    lowVoltageAlarm = 9.5; //10.2;
    batteryVoltage = lowVoltageWarning + 2;
    Serial.println("init MatterMonitor_APM2: done");
  }

  void lowBatteryEvent(byte level) {
    long currentBatteryTime = millis() - previousBatteryTime;
    if (level == OK) {
      digitalWrite(BUZZERPIN, LOW);
      autoDescent = 0;
      if ( flight->armed && (currentBatteryTime > 600)) {
            digitalWrite(A14, HIGH); 
                  digitalWrite(A12, HIGH); 
                        digitalWrite(A10, HIGH); 
                              digitalWrite(A8, HIGH); 
                                    previousBatteryTime = millis();
                                        } else {
                                              digitalWrite(A14, LOW); 
                                                    digitalWrite(A12, LOW); 
                                                          digitalWrite(A10, LOW); 
                                                                digitalWrite(A8, LOW);     
                                                                    } 
    }
    if (level == WARNING) {
      if (currentBatteryTime > 200) {
        //autoDescent = 50;
        digitalWrite(BUZZERPIN, HIGH);
        digitalWrite(A14, HIGH); 
              digitalWrite(A12, HIGH); 
                    digitalWrite(A10, HIGH); 
                          digitalWrite(A8, HIGH); 
      
      
        previousBatteryTime = millis();
        } else {
        
        //autoDescent = 0;
        digitalWrite(BUZZERPIN, LOW);
        digitalWrite(A14, LOW); 
              digitalWrite(A12, LOW); 
                    digitalWrite(A10, LOW); 
                          digitalWrite(A8, LOW); 
      }
    }
    if (level == ALARM) {
      if (firstAlarm == OFF) autoDescent = 0; // intialize autoDescent to zero if first time in ALARM state
      firstAlarm = ON;
      digitalWrite(BUZZERPIN, HIGH); // enable buzzer
  //    digitalWrite(LED3PIN, HIGH);
      
      digitalWrite(A14, HIGH); 
            digitalWrite(A12, HIGH); 
                  digitalWrite(A10, HIGH); 
                        digitalWrite(A8, HIGH); 
/*                        
      if ((currentBatteryTime > 500) && (throttle > 1400)) {
        autoDescent -= 1; // auto descend quad
        holdAltitude -= 0.2; // descend if in attitude hold mode
        previousBatteryTime = millis();
      }
*/      
    }
  }

  const float readBatteryVoltage(byte channel) {
    return (analogRead(channel) * batteryScaleFactor) + diode;
  }
};



#endif