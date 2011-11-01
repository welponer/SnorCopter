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

// FlightCommand.pde is responsible for decoding transmitter stick combinations
// for setting up AeroQuad modes such as motor arming and disarming

void readPilotCommands() {
  readReceiver();
  // Read quad configuration commands from transmitter when throttle down
  if (receiverCommand[THROTTLE] < MINCHECK) {
    zeroIntegralError();
    //receiver->adjustThrottle(throttleAdjust);
    // Disarm motors (left stick lower left corner)
    if (receiverCommand[YAW] < MINCHECK && armed == ON) {
      armed = OFF;
      notifyOSD(OSD_CENTER|OSD_WARN, "MOTORS UNARMED");
      commandAllMotors(MINCOMMAND);
      #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
        digitalWrite(LED_Red, LOW);
      #endif
    }    
    // Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
    if ((receiverCommand[YAW] < MINCHECK) && (receiverCommand[ROLL] > MAXCHECK) && (receiverCommand[PITCH] < MINCHECK)) {
      calibrateGyro(); // defined in Gyro.h
      calibrateAccel(); // defined in Accel.h
      storeSensorsZeroToEEPROM();
      //accel.setOneG(accel.getFlightData(ZAXIS));
      calibrateKinematics();
      zeroIntegralError();
      pulseMotors(3);
      // ledCW() is currently a private method in BatteryMonitor.h, fix and agree on this behavior in next revision
      //#if defined(BattMonitor) && defined(ArduCopter)
      //  ledCW(); ledCW(); ledCW();
      //#endif
    }   
    // Arm motors (left stick lower right corner)
    if (receiverCommand[YAW] > MAXCHECK && armed == OFF && safetyCheck == ON) {
      zeroIntegralError();
      armed = ON;
      notifyOSD(OSD_CENTER|OSD_WARN, "!MOTORS ARMED!");
      #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
        digitalWrite(LED_Red, HIGH);
      #endif
      for (byte motor = 0; motor < LASTMOTOR; motor++) {
        motorCommand[motor] = MINTHROTTLE;
      }
      //   delay(100);
      //altitude.measureGround();
    }
    // Prevents accidental arming of motor output if no transmitter command received
    if (receiverCommand[YAW] > MINCHECK) safetyCheck = ON; 
  }
  
  #ifdef RateModeOnly
    flightMode = ACRO;
  #else
    // Check Mode switch for Acro or Stable
    if (receiverCommand[MODE] > 1500) {
      if (flightMode == ACRO) {
        #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
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
  
  #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
    if (flightMode == ACRO) {
      digitalWrite(LED_Yellow, HIGH);
      digitalWrite(LED_Green, LOW);
    }
   else if (flightMode == STABLE) {
      digitalWrite(LED_Green, HIGH);
      digitalWrite(LED_Yellow, LOW); 
   }
  #endif
  
  #ifdef AltitudeHold
   if (receiverCommand[AUX] < 1750) {
     if (altitudeHoldState != ALTPANIC ) {  // check for special condition with manditory override of Altitude hold
       if (isStoreAltitudeNeeded) {
         altitudeToHoldTarget = getBaroAltitude();
         altitudeHoldThrottle = receiverCommand[THROTTLE];
         PID[ALTITUDE].integratedError = 0;
         PID[ALTITUDE].lastPosition = altitudeToHoldTarget;  // add to initialize hold position on switch turn on.
         isStoreAltitudeNeeded = false;
       }
       altitudeHoldState = ON;
     }
     // note, Panic will stay set until Althold is toggled off/on
   } 
   else {
     isStoreAltitudeNeeded = true;
     altitudeHoldState = OFF;
   }
  #endif
}




