/*
  AeroQuad v2.5 Beta 1 - July 2011
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

/******************************************************/

// PID Variables
struct PIDdata {
  boolean firstPass;
  float   P, I, D;
  float   iTerm;
  float   windupGuard;
  float   lastState;
} PID[LAST_PID];

/******************************************************/

float updatePID(float command, float state, float deltaT, struct PIDdata *PIDparameters) {
  float error;
  float dTerm;

  if (PIDparameters->firstPass) {
    PIDparameters->firstPass = false;
    PIDparameters->lastState = state;
  }

  error = command - state;

  PIDparameters->iTerm += error * deltaT;
  PIDparameters->iTerm = constrain(PIDparameters->iTerm, -PIDparameters->windupGuard, PIDparameters->windupGuard);
  
  dTerm = (state - PIDparameters->lastState) / deltaT;

  PIDparameters->lastState = state;
  
  return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->iTerm)) - (PIDparameters->D * dTerm);
}

/******************************************************/

void setIntegralError(byte IDPid, float value)
{
  PID[IDPid].iTerm = value;
}

/******************************************************/

void zeroIntegralError() __attribute__ ((noinline));
void zeroIntegralError() {
  for (byte axis = ROLL; axis < LAST_PID; axis++) {
    setIntegralError(axis, 0.0);
  }
}

/******************************************************/



