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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_TRI_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_TRI_MODE_H_


/*
       CW                  CCW
            
           0....Front....0  
           ......***......    
           ......***......
           ......***......    
           0....Back.....0  
      
                  CW           
*/


#define SERVO       MOTOR1
#define FRONT_LEFT  MOTOR2
#define FRONT_RIGHT MOTOR3
#define REAR        MOTOR4
#define LASTMOTOR   MOTOR4+1

#define TRI_YAW_CONSTRAINT_MIN 1100
#define TRI_YAW_CONSTRAINT_MAX 1900
#define TRI_YAW_MIDDLE 1500

#define MAX_RECEIVER_OFFSET 50

void applyMotorCommand() {
  motorCommand[FRONT_LEFT]  = throttle + motorAxisCommandRoll - motorAxisCommandPitch*2/3;
  motorCommand[FRONT_RIGHT] = throttle - motorAxisCommandRoll - motorAxisCommandPitch*2/3;
  motorCommand[REAR] =        throttle + motorAxisCommandPitch*4/3;
  const float yawMotorCommand = constrain(motorAxisCommandYaw,-MAX_RECEIVER_OFFSET-abs(receiverCommand[YAW]),+MAX_RECEIVER_OFFSET+abs(receiverCommand[YAW]));
  motorCommand[SERVO] = constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * yawMotorCommand, TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX);
}

void processMinMaxCommand() {
  if ((motorCommand[FRONT_LEFT] <= MINTHROTTLE) || (motorCommand[REAR] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[FRONT_RIGHT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR]        = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[FRONT_LEFT] >= MAXCOMMAND) || (motorCommand[REAR] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[FRONT_RIGHT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR]        = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_RIGHT] = MAXCOMMAND;
    motorMaxCommand[REAR]        = MAXCOMMAND; 
    motorMinCommand[FRONT_RIGHT] = MINTHROTTLE;
    motorMinCommand[REAR]        = MINTHROTTLE;
  }

  if ((motorCommand[REAR] <= MINTHROTTLE) || (motorCommand[FRONT_RIGHT] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[FRONT_LEFT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[REAR] >= MAXCOMMAND) || (motorCommand[FRONT_RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[FRONT_LEFT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_LEFT] = MAXCOMMAND;
    motorMaxCommand[REAR]       = MAXCOMMAND;
    motorMinCommand[FRONT_LEFT] = MINTHROTTLE;
    motorMinCommand[REAR]       = MINTHROTTLE;
  }
  motorMaxCommand[SERVO] = MAXCOMMAND;
  motorMinCommand[SERVO] = MINCOMMAND;
  
}


//void processHardManuevers() {
//
//  if (receiverCommand[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
//    motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
//    motorMaxCommand[FRONT_LEFT]  = minAcro;
//    motorMaxCommand[REAR]        = throttle + motorAxisCommandPitch*4/3;
//  }
//  else if (receiverCommand[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
//    motorMinCommand[FRONT_LEFT]  = MAXCOMMAND;
//    motorMaxCommand[FRONT_RIGHT] = minAcro;
//    motorMaxCommand[REAR]        = throttle + motorAxisCommandPitch*4/3;
//  }
//  else if (receiverCommand[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
//    motorMinCommand[FRONT_LEFT]  = MAXCOMMAND;
//    motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
//    motorMaxCommand[REAR]        = minAcro;
//  }
//  else if (receiverCommand[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
//    motorMinCommand[REAR]        = MAXCOMMAND;
//    motorMaxCommand[FRONT_LEFT]  = minAcro;
//    motorMaxCommand[FRONT_RIGHT] = minAcro;
//  }
//}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
