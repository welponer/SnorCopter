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

// FlightCommand.pde is responsible for decoding transmitter stick combinations
// for setting up AeroQuad modes such as motor arming and disarming

void readPilotCommands() {
  readReceiver();
  // Read quad configuration commands from transmitter when throttle down
  if (receiverData[THROTTLE] < MINCHECK) {
    zeroIntegralError();
    // Disarm motors (left stick lower left corner)
    if (receiverData[YAW] < (MINCHECK - MIDCOMMAND) && armed == ON) {
      armed = OFF;
      for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
        commandAllMotors(MINCOMMAND);
    }    
    // Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
    if ((receiverData[YAW]   < MINCHECK - MIDCOMMAND) && 
        (receiverData[ROLL]  > MAXCHECK - MIDCOMMAND) && 
        (receiverData[PITCH] < MINCHECK - MIDCOMMAND))
    {
      computeGyroBias();
      computeAccelBias();
      zeroIntegralError();
      pulseMotors(3);
    }   
    // Arm motors (left stick lower right corner)
    if (receiverData[YAW] > (MAXCHECK - MIDCOMMAND) && armed == OFF && safetyCheck == ON) {
      zeroIntegralError();
      armed = ON;
      for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
        minCommand[motor] = MINTHROTTLE;
    }
    // Prevents accidental arming of motor output if no transmitter command received
    if (receiverData[YAW] > (MINCHECK - MIDCOMMAND)) 
    {
      safetyCheck = ON; 
    }
  }
  
  #ifdef RateModeOnly
    flightMode = ACRO;
  #else
    // Check Mode switch for Acro or Stable
    if (receiverData[MODE] > MIDCOMMAND) {
      if (flightMode == ACRO) {
        #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(AeroQuadMega_Wii)
          digitalWrite(LED2PIN, HIGH);
        #endif
        zeroIntegralError();
      }
      flightMode = STABLE;
   }
    else {
      #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
        if (flightMode == STABLE)
          digitalWrite(LED2PIN, LOW);
      #endif
      flightMode = ACRO;
    }
  #endif
}




