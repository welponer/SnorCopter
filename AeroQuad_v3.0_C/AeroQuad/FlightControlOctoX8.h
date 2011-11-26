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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_OCTO_X8_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_OCTO_X8_MODE_H_


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

#define FRONT_LEFT    MOTOR1
#define FRONT_RIGHT   MOTOR2
#define REAR_RIGHT    MOTOR3
#define REAR_LEFT     MOTOR4
#define FRONT_LEFT_2  MOTOR5
#define FRONT_RIGHT_2 MOTOR6
#define REAR_RIGHT_2  MOTOR7
#define REAR_LEFT_2   MOTOR8
#define LASTMOTOR     MOTOR8+1

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int throttleCorrection = abs(motorAxisCommandYaw*4/8);
  motorCommand[FRONT_LEFT] =    (throttle-throttleCorrection) - motorAxisCommandPitch + motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT] =   (throttle-throttleCorrection) - motorAxisCommandPitch - motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT] =     (throttle-throttleCorrection) + motorAxisCommandPitch + motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT] =    (throttle-throttleCorrection) + motorAxisCommandPitch - motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_LEFT_2] =  (throttle-throttleCorrection) - motorAxisCommandPitch + motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT_2] = (throttle-throttleCorrection) - motorAxisCommandPitch - motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT_2] =   (throttle-throttleCorrection) + motorAxisCommandPitch + motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT_2] =  (throttle-throttleCorrection) + motorAxisCommandPitch - motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
}

void processMinMaxCommand() {

  if (receiverCommand[THROTTLE] > MAXCHECK) { // if the throttle is about the max, we used tue PID values!
    return;
  }
  
  if ((motorCommand[FRONT_LEFT] <= MINTHROTTLE) || (motorCommand[REAR_RIGHT] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[FRONT_RIGHT] =   constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_LEFT] =     constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[FRONT_RIGHT_2] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_LEFT_2] =   constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[FRONT_LEFT] >= MAXCOMMAND) || (motorCommand[REAR_RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[FRONT_RIGHT]   = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_LEFT]     = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[FRONT_RIGHT_2] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_LEFT_2]   = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_RIGHT]   = MAXCOMMAND;
    motorMaxCommand[REAR_LEFT]     = MAXCOMMAND; 
    motorMaxCommand[FRONT_RIGHT_2] = MAXCOMMAND;
    motorMaxCommand[REAR_LEFT_2]   = MAXCOMMAND; 
    motorMinCommand[FRONT_RIGHT]   = MINTHROTTLE;
    motorMinCommand[REAR_LEFT]     = MINTHROTTLE;
    motorMinCommand[FRONT_RIGHT_2] = MINTHROTTLE;
    motorMinCommand[REAR_LEFT_2]   = MINTHROTTLE;
  }

  if ((motorCommand[REAR_LEFT] <= MINTHROTTLE) || (motorCommand[FRONT_RIGHT] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[FRONT_LEFT]   = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_RIGHT]   = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[FRONT_LEFT_2] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_RIGHT_2] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[REAR_LEFT] >= MAXCOMMAND) || (motorCommand[FRONT_RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[FRONT_LEFT]   = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_RIGHT]   = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[FRONT_LEFT_2] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_RIGHT_2] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_LEFT]   = MAXCOMMAND;
    motorMaxCommand[REAR_RIGHT]   = MAXCOMMAND;
    motorMaxCommand[FRONT_LEFT_2] = MAXCOMMAND;
    motorMaxCommand[REAR_RIGHT_2] = MAXCOMMAND;
    motorMinCommand[FRONT_LEFT]   = MINTHROTTLE;
    motorMinCommand[REAR_RIGHT]   = MINTHROTTLE;
    motorMinCommand[FRONT_LEFT_2] = MINTHROTTLE;
    motorMinCommand[REAR_RIGHT_2] = MINTHROTTLE;
  }
}

//void processHardManuevers() {
//
//  if (receiverCommand[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
//    motorMinCommand[FRONT_RIGHT]   = MAXCOMMAND;
//    motorMinCommand[REAR_RIGHT]    = MAXCOMMAND;
//    motorMinCommand[FRONT_RIGHT_2] = MAXCOMMAND;
//    motorMinCommand[REAR_RIGHT_2]  = MAXCOMMAND;
//    motorMaxCommand[FRONT_LEFT]    = minAcro;
//    motorMaxCommand[REAR_LEFT]     = minAcro;
//    motorMaxCommand[FRONT_LEFT_2]  = minAcro;
//    motorMaxCommand[REAR_LEFT_2]   = minAcro;
//  }
//  else if (receiverCommand[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
//    motorMinCommand[FRONT_LEFT]    = MAXCOMMAND;
//    motorMinCommand[REAR_LEFT]     = MAXCOMMAND;
//    motorMinCommand[FRONT_LEFT_2]  = MAXCOMMAND;
//    motorMinCommand[REAR_LEFT_2]   = MAXCOMMAND;
//    motorMaxCommand[FRONT_RIGHT]   = minAcro;
//    motorMaxCommand[REAR_RIGHT]    = minAcro;
//    motorMaxCommand[FRONT_RIGHT_2] = minAcro;
//    motorMaxCommand[REAR_RIGHT_2]  = minAcro;
//  }
//  else if (receiverCommand[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
//    motorMinCommand[FRONT_LEFT]    = MAXCOMMAND;
//    motorMinCommand[FRONT_RIGHT]   = MAXCOMMAND;
//    motorMinCommand[FRONT_LEFT_2]  = MAXCOMMAND;
//    motorMinCommand[FRONT_RIGHT_2] = MAXCOMMAND;
//    motorMaxCommand[REAR_LEFT]     = minAcro;
//    motorMaxCommand[REAR_RIGHT]    = minAcro;
//    motorMaxCommand[REAR_LEFT_2]   = minAcro;
//    motorMaxCommand[REAR_RIGHT_2]  = minAcro;
//  }
//  else if (receiverCommand[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
//    motorMinCommand[REAR_LEFT]     = MAXCOMMAND;
//    motorMinCommand[REAR_RIGHT]    = MAXCOMMAND;
//    motorMinCommand[REAR_LEFT_2]   = MAXCOMMAND;
//    motorMinCommand[REAR_RIGHT_2]  = MAXCOMMAND;
//    motorMaxCommand[FRONT_LEFT]    = minAcro;
//    motorMaxCommand[FRONT_RIGHT]   = minAcro;
//    motorMaxCommand[FRONT_LEFT_2]  = minAcro;
//    motorMaxCommand[FRONT_RIGHT_2] = minAcro;
//  }
//}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

