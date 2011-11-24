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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_OCTO_X_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_OCTO_X_MODE_H_


/*  
             UPPER/LOWER


       CW/CCW           CCW/CW
            
           0....Front....0  
           ......***......    
           ......***......
           ......***......    
           0....Back.....0  
      
       CCW/CW           CW/CCW
*/     

#define FRONT_LEFT      MOTOR1
#define FRONT_RIGHT     MOTOR2
#define MID_FRONT_RIGHT MOTOR3
#define MID_REAR_RIGHT  MOTOR4
#define REAR_RIGHT      MOTOR5
#define REAR_LEFT       MOTOR6
#define MID_REAR_LEFT   MOTOR7
#define MID_FRONT_LEFT  MOTOR8
#define LASTMOTOR       MOTOR8+1

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int throttleCorrection  = abs(motorAxisCommandYaw*4/8);
  motorCommand[FRONT_LEFT]      = (throttle-throttleCorrection) - motorAxisCommandPitch     + motorAxisCommandRoll*0.5 - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT]     = (throttle-throttleCorrection) - motorAxisCommandPitch     - motorAxisCommandRoll*0.5 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[MID_FRONT_RIGHT] = (throttle-throttleCorrection) - motorAxisCommandPitch*0.5 - motorAxisCommandRoll     - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[MID_REAR_RIGHT]  = (throttle-throttleCorrection) + motorAxisCommandPitch*0.5 - motorAxisCommandRoll     + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT]      = (throttle-throttleCorrection) + motorAxisCommandPitch     - motorAxisCommandRoll*0.5 - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT]       = (throttle-throttleCorrection) + motorAxisCommandPitch     + motorAxisCommandRoll*0.5 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[MID_REAR_LEFT]   = (throttle-throttleCorrection) + motorAxisCommandPitch*0.5 + motorAxisCommandRoll     - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[MID_FRONT_LEFT]  = (throttle-throttleCorrection) - motorAxisCommandPitch*0.5 + motorAxisCommandRoll     + (YAW_DIRECTION * motorAxisCommandYaw);
}


// @Kenny, check this one for v3.0
void processMinMaxCommand() {
  if ((motorCommand[FRONT_LEFT] <= MINTHROTTLE) || (motorCommand[MID_FRONT_RIGHT] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[FRONT_RIGHT]    = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[MID_REAR_RIGHT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_LEFT]      = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[MID_FRONT_LEFT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[FRONT_LEFT] >= MAXCOMMAND) || (motorCommand[MID_FRONT_RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[FRONT_RIGHT]    = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[MID_REAR_RIGHT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_LEFT]      = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[MID_FRONT_LEFT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_RIGHT]    = MAXCOMMAND;
    motorMaxCommand[MID_REAR_RIGHT] = MAXCOMMAND; 
    motorMaxCommand[REAR_LEFT]      = MAXCOMMAND;
    motorMaxCommand[MID_FRONT_LEFT] = MAXCOMMAND; 
    motorMinCommand[FRONT_RIGHT]    = MINTHROTTLE;
    motorMinCommand[MID_REAR_RIGHT] = MINTHROTTLE;
    motorMinCommand[REAR_LEFT]      = MINTHROTTLE;
    motorMinCommand[MID_FRONT_LEFT] = MINTHROTTLE;
  }

  if ((motorCommand[MID_REAR_RIGHT] <= MINTHROTTLE) || (motorCommand[FRONT_RIGHT] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[FRONT_LEFT]      = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[MID_FRONT_RIGHT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_RIGHT]      = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[MID_REAR_LEFT]   = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[MID_REAR_RIGHT] >= MAXCOMMAND) || (motorCommand[FRONT_RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[FRONT_LEFT]      = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[MID_FRONT_RIGHT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_RIGHT]      = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[MID_REAR_LEFT]   = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_LEFT]      = MAXCOMMAND;
    motorMaxCommand[MID_FRONT_RIGHT] = MAXCOMMAND;
    motorMaxCommand[REAR_RIGHT]      = MAXCOMMAND;
    motorMaxCommand[MID_REAR_LEFT]   = MAXCOMMAND;
    motorMinCommand[FRONT_LEFT]      = MINTHROTTLE;
    motorMinCommand[MID_FRONT_RIGHT] = MINTHROTTLE;
    motorMinCommand[REAR_RIGHT]      = MINTHROTTLE;
    motorMinCommand[MID_REAR_LEFT]   = MINTHROTTLE;
  }
}


// @Kenny, check this one for v3.0
void processHardManuevers() {
  if (flightMode == ACRO) {
    if (receiverCommand[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
      motorMinCommand[FRONT_RIGHT]     = MAXCOMMAND;
      motorMinCommand[MID_FRONT_RIGHT] = MAXCOMMAND;
      motorMinCommand[REAR_LEFT]       = MAXCOMMAND;
      motorMinCommand[MID_REAR_LEFT]   = MAXCOMMAND;
      motorMaxCommand[FRONT_LEFT]      = minAcro;
      motorMaxCommand[MID_REAR_RIGHT]  = minAcro;
      motorMaxCommand[REAR_RIGHT]      = minAcro;
      motorMaxCommand[MID_FRONT_LEFT]  = minAcro;
    }
    else if (receiverCommand[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
      motorMinCommand[FRONT_LEFT]      = MAXCOMMAND;
      motorMinCommand[MID_REAR_RIGHT]  = MAXCOMMAND;
      motorMinCommand[REAR_RIGHT]      = MAXCOMMAND;
      motorMinCommand[MID_FRONT_LEFT]  = MAXCOMMAND;
      motorMaxCommand[FRONT_RIGHT]     = minAcro;
      motorMaxCommand[MID_FRONT_RIGHT] = minAcro;
      motorMaxCommand[REAR_LEFT]       = minAcro;
      motorMaxCommand[MID_REAR_LEFT]   = minAcro;
    }
    else if (receiverCommand[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
      motorMinCommand[FRONT_LEFT]      = MAXCOMMAND;
      motorMinCommand[FRONT_RIGHT]     = MAXCOMMAND;
      motorMinCommand[REAR_RIGHT]      = MAXCOMMAND;
      motorMinCommand[REAR_LEFT]       = MAXCOMMAND;
      motorMaxCommand[MID_REAR_RIGHT]  = minAcro;
      motorMaxCommand[MID_FRONT_RIGHT] = minAcro;
      motorMaxCommand[MID_FRONT_LEFT]  = minAcro;
      motorMaxCommand[MID_REAR_LEFT]   = minAcro;
    }
    else if (receiverCommand[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      motorMinCommand[MID_REAR_RIGHT]  = MAXCOMMAND;
      motorMinCommand[MID_FRONT_RIGHT] = MAXCOMMAND;
      motorMinCommand[MID_FRONT_LEFT]  = MAXCOMMAND;
      motorMinCommand[MID_REAR_LEFT]   = MAXCOMMAND;
      motorMaxCommand[FRONT_LEFT]      = minAcro;
      motorMaxCommand[FRONT_RIGHT]     = minAcro;
      motorMaxCommand[REAR_RIGHT]      = minAcro;
      motorMaxCommand[REAR_LEFT]       = minAcro;
    }
  }
}



#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

