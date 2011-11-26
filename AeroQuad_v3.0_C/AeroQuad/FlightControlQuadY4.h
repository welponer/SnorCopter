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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_Y4_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_Y4_MODE_H_

/*
             UPPER/LOWER


       CW                  CCW
            
           0....Front....0  
           ......***......    
           ......***......
           ......***......    
           0....Back.....0  
      
                CW/CCW           
*/


#define LEFT        MOTOR1
#define RIGHT       MOTOR2
#define REAR        MOTOR3
#define REAR_UNDER  MOTOR4
#define LASTMOTOR   MOTOR4+1

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int throttleCorrection = abs(motorAxisCommandYaw*2/4);
  motorCommand[LEFT]        = (throttle)                    - motorAxisCommandPitch + motorAxisCommandRoll;
  motorCommand[RIGHT]       = (throttle)                    - motorAxisCommandPitch - motorAxisCommandRoll;
  motorCommand[REAR_UNDER]  = (throttle-throttleCorrection) + motorAxisCommandPitch + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR]        = (throttle-throttleCorrection) + motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw);
}

void processMinMaxCommand() {

  if (receiverCommand[THROTTLE] > MAXCHECK) { // if the throttle is about the max, we used tue PID values!
    return;
  }
  
  if ((motorCommand[LEFT] <= MINTHROTTLE) || (motorCommand[REAR_UNDER] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[RIGHT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR]  = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[LEFT] >= MAXCOMMAND) || (motorCommand[REAR_UNDER] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[RIGHT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR]  = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[RIGHT] = MAXCOMMAND;
    motorMaxCommand[REAR]  = MAXCOMMAND; 
    motorMinCommand[RIGHT] = MINTHROTTLE;
    motorMinCommand[REAR]  = MINTHROTTLE;
  }

  if ((motorCommand[REAR] <= MINTHROTTLE) || (motorCommand[RIGHT] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[LEFT]       = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_UNDER] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[REAR] >= MAXCOMMAND) || (motorCommand[RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[LEFT]       = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_UNDER] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[LEFT]       = MAXCOMMAND;
    motorMaxCommand[REAR_UNDER] = MAXCOMMAND;
    motorMinCommand[LEFT]       = MINTHROTTLE;
    motorMinCommand[REAR_UNDER] = MINTHROTTLE;
  }
}

//void processHardManuevers() {
//
//  if (receiverCommand[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
//    motorMinCommand[RIGHT] = MAXCOMMAND;
//    motorMaxCommand[LEFT]  = minAcro;
//  }
//  else if (receiverCommand[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
//    motorMinCommand[RIGHT] = minAcro;
//    motorMaxCommand[LEFT]  = MAXCOMMAND;
//  }
//  else if (receiverCommand[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
//    motorMinCommand[LEFT]        = MAXCOMMAND;
//    motorMinCommand[RIGHT]       = MAXCOMMAND;
//    motorMaxCommand[REAR_UNDER]  = minAcro;
//    motorMaxCommand[REAR]        = minAcro;
//  }
//  else if (receiverCommand[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
//    motorMinCommand[REAR_UNDER]  = MAXCOMMAND;
//    motorMinCommand[REAR]        = MAXCOMMAND;
//    motorMaxCommand[LEFT]        = minAcro;
//    motorMaxCommand[RIGHT]       = minAcro;
//  }
//}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
