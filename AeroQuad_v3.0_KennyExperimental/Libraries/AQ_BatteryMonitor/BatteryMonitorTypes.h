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

#ifndef _AQ_BATTERY_MONITOR_TYPES
#define _AQ_BATTERY_MONITOR_TYPES

#define BATTERY_MONITOR_OK      0
#define BATTERY_MONITOR_WARNING 1
#define BATTERY_MONITOR_ALARM   2

#define NOPIN 255

struct BatteryData {
  byte  vPin,cPin;        // A/D pins for voltage and current sensors (255 == no sensor)
  float vWarning,vAlarm;  // Warning and Alarm voltage levels
  float vScale,vBias;     // voltage polynom V = vbias + AnalogIn(vpin)*vscale
  float cScale,cBias;     // current polynom C = cbias + AnalogIn(cpin)*cscale
  float voltage;          // Current battery voltage
  float current;          // Current battery current
  float minVoltage;       // Minimum voltage since reset
  float maxCurrent;       // Maximum current since reset
  float usedCapacity;     // Capacity used since reset (in mAh)
  byte  status;           //
};

extern struct BatteryData batteryData[];     // BatteryMonitor config, !! MUST BE DEFINED BY MAIN SKETCH !!
extern byte               numberOfBatteries; // number of batteries monitored, defined by BatteryMonitor
extern byte               batteryStatus;     // combined state of batteries, defined by BatteryMonitor

// Helper macros to make battery difinitions cleaner

// for defining battery with just voltage sensing
#define BM_DEFINE_BATTERY_V(VPIN,VWARNING,VALARM,VSCALE,VBIAS) {VPIN,NOPIN,VWARNING,VALARM,VSCALE,VBIAS, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, BATTERY_MONITOR_OK},

// for defining battery with voltage and current sensors
#define BM_DEFINE_BATTERY_VC(VPIN,VWARNING,VALARM,VSCALE,VBIAS,CPIN,CSCALE,CBIAS) {VPIN,CPIN,VWARNING,VALARM,VSCALE,VBIAS, CSCALE, CBIAS, 0.0, 0.0, 0.0, 0.0, 0.0, BATTERY_MONITOR_OK},

#endif