/*
  AeroQuad v3.0 - April 2011
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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)
//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_H_

#define ATTITUDE_SCALING (0.75 * PWM2RAD)

void calculateFlightError()
{
  if (flightMode == STABLE) {
    float rollAttitudeCmd = updatePID((receiverCommand[ROLL] - receiverZero[ROLL]) * ATTITUDE_SCALING, kinematicsAngle[ROLL], &PID[LEVELROLL]);
    float pitchAttitudeCmd = updatePID((receiverCommand[PITCH] - receiverZero[PITCH]) * ATTITUDE_SCALING, -kinematicsAngle[PITCH], &PID[LEVELPITCH]);
    motorAxisCommandRoll = updatePID(rollAttitudeCmd, gyroRate[ROLL], &PID[LEVELGYROROLL]);
    motorAxisCommandPitch = updatePID(pitchAttitudeCmd, -gyroRate[PITCH], &PID[LEVELGYROPITCH]);
  }
  else {
    motorAxisCommandRoll = updatePID(getReceiverSIData(ROLL), gyroRate[ROLL], &PID[ROLL]);
    motorAxisCommandPitch = updatePID(getReceiverSIData(PITCH), -gyroRate[PITCH], &PID[PITCH]);
  }
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processCalibrateESC //////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processCalibrateESC()
{
  switch (calibrateESC) { // used for calibrating ESC's
  case 1:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motorCommand[motor] = MAXCOMMAND;
    break;
  case 3:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motorCommand[motor] = constrain(testCommand, 1000, 1200);
    break;
  case 5:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motorCommand[motor] = constrain(motorConfiguratorCommand[motor], 1000, 1200);
    safetyCheck = ON;
    break;
  default:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motorCommand[motor] = MINCOMMAND;
  }
  // Send calibration commands to motors
  writeMotors(); // Defined in Motors.h
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processHeadingHold ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processHeading()
{
  if (headingHoldConfig == ON) {

    #if defined(HeadingMagHold)
      heading = degrees(kinematicsAngle[YAW]);
    #else
      heading = degrees(gyroHeading);
    #endif

    // Always center relative heading around absolute heading chosen during yaw command
    // This assumes that an incorrect yaw can't be forced on the AeroQuad >180 or <-180 degrees
    // This is done so that AeroQuad does not accidentally hit transition between 0 and 360 or -180 and 180
    // AKA - THERE IS A BUG HERE - if relative heading is greater than 180 degrees, the PID will swing from negative to positive
    // Doubt that will happen as it would have to be uncommanded.
    relativeHeading = heading - setHeading;
    if (heading <= (setHeading - 180)) {
      relativeHeading += 360;
    }
    if (heading >= (setHeading + 180)) {
      relativeHeading -= 360;
    }

    // Apply heading hold only when throttle high enough to start flight
    if (receiverCommand[THROTTLE] > MINCHECK ) { 
      
      if ((receiverCommand[YAW] > (MIDCOMMAND + 25)) || (receiverCommand[YAW] < (MIDCOMMAND - 25))) {
        
        // If commanding yaw, turn off heading hold and store latest heading
        setHeading = heading;
        headingHold = 0;
        PID[HEADING].integratedError = 0;
        headingHoldState = OFF;
        headingTime = currentTime;
      }
      else {
        if (relativeHeading < 0.25 && relativeHeading > -0.25) {
          headingHold = 0;
          PID[HEADING].integratedError = 0;
        }
        else if (headingHoldState == OFF) { // quick fix to soften heading hold on new heading
          if ((currentTime - headingTime) > 500000) {
            headingHoldState = ON;
            headingTime = currentTime;
            setHeading = heading;
            headingHold = 0;
          }
        }
        else {
        // No new yaw input, calculate current heading vs. desired heading heading hold
        // Relative heading is always centered around zero
          headingHold = updatePID(0, relativeHeading, &PID[HEADING]);
          headingTime = currentTime; // quick fix to soften heading hold, wait 100ms before applying heading hold
        }
      }
    }
    else {
      // minimum throttle not reached, use off settings
      setHeading = heading;
      headingHold = 0;
      PID[HEADING].integratedError = 0;
    }
  }
  // NEW SI Version
  commandedYaw = constrain(getReceiverSIData(YAW) + radians(headingHold), -PI, PI);
  motorAxisCommandYaw = updatePID(commandedYaw, gyroRate[YAW], &PID[YAW]);
}



//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processAltitudeHold //////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processAltitudeHold()
{
  // ****************************** Altitude Adjust *************************
  // Thanks to Honk for his work with altitude hold
  // http://aeroquad.com/showthread.php?792-Problems-with-BMP085-I2C-barometer
  // Thanks to Sherbakov for his work in Z Axis dampening
  // http://aeroquad.com/showthread.php?359-Stable-flight-logic...&p=10325&viewfull=1#post10325
#ifdef AltitudeHold
  if (altitudeHoldState == ON) {
    int altitudeHoldThrottleCorrection = updatePID(altitudeToHoldTarget, getBaroAltitude(), &PID[ALTITUDE]);
    altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
    if (abs(altitudeHoldThrottle - receiverCommand[THROTTLE]) > PANICSTICK_MOVEMENT) {
      altitudeHoldState = ALTPANIC; // too rapid of stick movement so PANIC out of ALTHOLD
    } else {
      if (receiverCommand[THROTTLE] > (altitudeHoldThrottle + ALTBUMP)) { // AKA changed to use holdThrottle + ALTBUMP - (was MAXCHECK) above 1900
        altitudeToHoldTarget += 0.01;
      }
      if (receiverCommand[THROTTLE] < (altitudeHoldThrottle - ALTBUMP)) { // AKA change to use holdThorrle - ALTBUMP - (was MINCHECK) below 1100
        altitudeToHoldTarget -= 0.01;
      }
    }
    throttle = altitudeHoldThrottle + altitudeHoldThrottleCorrection;
  }
  else {
    throttle = receiverCommand[THROTTLE];
  }
#else
  throttle = receiverCommand[THROTTLE];
#endif
}

#if defined BattMonitorAutoDescent
  void processBatteryMonitorThrottleAdjustment() {
    if (batteryStatus == BATTERY_MONITOR_ALARM) {
      if (batteryMonitorAlarmCounter < BATTERY_MONITOR_MAX_ALARM_COUNT) {
        batteryMonitorAlarmCounter++;
      }
      else {
        #ifdef AltitudeHold
          if (throttle > BATTERY_MONITOR_THROTTLE_TARGET) {
            altitudeToHoldTarget -= 0.2;
          }
        #else
          if (batteryMonitorStartThrottle == 0) {  // init battery monitor throttle correction!
            batteryMonitorStartTime = millis();
            batteryMonitorStartThrottle = throttle; 
          }
          const int batteryMonitorThrottle = map(millis()-batteryMonitorStartTime,0,BATTERY_MONITOR_GOIN_DOWN_TIME,batteryMonitorStartThrottle,BATTERY_MONITOR_THROTTLE_TARGET);
          if (throttle < batteryMonitorThrottle) {
            batteyMonitorThrottleCorrection = 0;
          }
          else {
            batteyMonitorThrottleCorrection = batteryMonitorThrottle - throttle;
          }
        #endif
      }
    }
    else {
      batteryMonitorStartThrottle = 0;
      batteyMonitorThrottleCorrection = 0.0;
    }
  }
#endif  


void processThrottleCorrection() {
 
  // Thank Ziojo for this little adjustment on throttle when manuevering!
  int throttleAsjust = throttle / (cos (radians (kinematicsAngle[ROLL])) * cos (radians (kinematicsAngle[PITCH])));
  throttleAsjust = constrain ((throttleAsjust - throttle), 0, 160); //compensate max  +/- 25 deg ROLL or PITCH or  +/- 18 ( 18(ROLL) + 18(PITCH))
  throttle = throttle + throttleAsjust + (int)batteyMonitorThrottleCorrection;
}


//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processFlightControl main function ///////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControl() {
  
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();
  
  if (frameCounter %  10 == 0) {  //   10 Hz tasks
    // ********************** Process Altitude hold **************************
    processAltitudeHold();
    // ********************** Process Battery monitor hold **************************
    #if defined BattMonitorAutoDescent
      processBatteryMonitorThrottleAdjustment();
    #endif
    // ********************** Process throttle correction ********************
    processThrottleCorrection();
  }

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    applyMotorCommand();
  } 

  // *********************** process min max motor command *******************
  processMinMaxCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
//  processHardManuevers();    // This is not a good way to handle loop, just learn to pilot and do it normally!
  
  // Apply limits to motor commands
  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    motorCommand[motor] = constrain(motorCommand[motor], motorMinCommand[motor], motorMaxCommand[motor]);
  }

  // If throttle in minimum position, don't apply yaw
  if (receiverCommand[THROTTLE] < MINCHECK) {
    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      motorCommand[motor] = MINTHROTTLE;
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    writeMotors();
  }
}

#endif //#define _AQ_PROCESS_FLIGHT_CONTROL_H_

