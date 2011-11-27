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

// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso
float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters) {
  float error;
  float dTerm;
  float deltaPIDTime = (currentTime - PIDparameters->previousPIDTime) / 1000000.0;

  PIDparameters->previousPIDTime = currentTime;  // AKA PID experiments
  error = targetPosition - currentPosition;
  PIDparameters->integratedError += error * deltaPIDTime;
  PIDparameters->integratedError = constrain(PIDparameters->integratedError, -PIDparameters->windupGuard, PIDparameters->windupGuard);
  dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastPosition) / (deltaPIDTime * 100); // dT fix from Honk
  PIDparameters->lastPosition = currentPosition;
  
  return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->integratedError)) + dTerm;
}

void zeroIntegralError() __attribute__ ((noinline));
void zeroIntegralError() {
  for (byte axis = ROLL; axis < LASTLEVELAXIS; axis++) {
    PID[axis].integratedError = 0;
    PID[axis].previousPIDTime = currentTime;
  }
}


class controlPID {
public:
  float paramP, paramI, paramD;
  float lastPosition;
  unsigned int previousTime;
  float integratedError;
  float windupGuard; // Thinking about having individual wind up guards for each PID
  
  controlPID(float cp, float ci, float cd, float cwg = 1000) {
    paramP = cp;
    paramI = ci;
    paramD = ci;
    windupGuard = cwg;
    lastPosition = 0;
    integratedError = 0;
    previousTime = micros();
  }
  
  float update(float targetPosition, float currentPosition) {
  float error;
  float dTerm;
  
  unsigned int currentTime = micros();
  float deltaTime = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;
  
  error = targetPosition - currentPosition;
  integratedError += error * deltaTime;
  integratedError = constrain( integratedError, -windupGuard, windupGuard);
  dTerm = paramD * (currentPosition - lastPosition) / (deltaTime * 100); // dT fix from Honk
  lastPosition = currentPosition;
  
  return (paramP * error) + (paramI * (integratedError)) + dTerm;
}


};
