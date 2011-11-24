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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_OCTO_PLUS_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_OCTO_PLUS_MODE_H_


/*  
             UPPER/LOWER


       CW/CCW           CCW/CW
            
           0....Front....0  
           ......***......    
           ......***......
           ......***......    
           0....Back.....0  
      
       CCW/CW           CW/CCW
       
#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z       
motor[4] = PIDMIX(+0   ,-1   ,-1); //FRONT
motor[1] = PIDMIX(-7/10,-7/10,+1); //FRONT_R
motor[5] = PIDMIX(-1   ,+0   ,-1); //RIGHT
motor[2] = PIDMIX(-7/10,+7/10,+1); //REAR_R
motor[6] = PIDMIX(+0   ,+1   ,-1); //REAR
motor[3] = PIDMIX(+7/10,+7/10,+1); //REAR_L
motor[7] = PIDMIX(+1   ,+0   ,-1); //LEFT 
motor[0] = PIDMIX(+7/10,-7/10,+1); //FRONT_L
*/     

#define FRONT       MOTOR1
#define FRONT_RIGHT MOTOR2
#define RIGHT       MOTOR3
#define REAR_RIGHT  MOTOR5
#define REAR        MOTOR4
#define REAR_LEFT   MOTOR6
#define LEFT        MOTOR7
#define FRONT_LEFT  MOTOR8
#define LASTMOTOR   MOTOR8+1

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int throttleCorrection  = abs(motorAxisCommandYaw*4/8);
  motorCommand[FRONT]       = (throttle-throttleCorrection) - motorAxisCommandPitch                                  - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT] = (throttle-throttleCorrection) - motorAxisCommandPitch*7/10 - motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[RIGHT]       = (throttle-throttleCorrection)                              - motorAxisCommandRoll      - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT]  = (throttle-throttleCorrection) + motorAxisCommandPitch*7/10 - motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR]        = (throttle-throttleCorrection) + motorAxisCommandPitch                                  - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT]   = (throttle-throttleCorrection) + motorAxisCommandPitch*7/10 + motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[LEFT]        = (throttle-throttleCorrection)                              + motorAxisCommandRoll      - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_LEFT]  = (throttle-throttleCorrection) - motorAxisCommandPitch*7/10 + motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
}


// @Kenny, check this one for v3.0
void processMinMaxCommand() {
  if ((motorCommand[FRONT] <= MINTHROTTLE) || (motorCommand[RIGHT] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[FRONT_RIGHT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR]        = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_LEFT]   = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[FRONT_LEFT]  = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[FRONT] >= MAXCOMMAND) || (motorCommand[RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[FRONT_RIGHT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR]        = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_LEFT]   = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[FRONT_LEFT]  = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_RIGHT] = MAXCOMMAND;
    motorMaxCommand[REAR]        = MAXCOMMAND; 
    motorMaxCommand[REAR_LEFT]   = MAXCOMMAND;
    motorMaxCommand[FRONT_LEFT]  = MAXCOMMAND; 
    motorMinCommand[FRONT_RIGHT] = MINTHROTTLE;
    motorMinCommand[REAR]        = MINTHROTTLE;
    motorMinCommand[REAR_LEFT]   = MINTHROTTLE;
    motorMinCommand[FRONT_LEFT]  = MINTHROTTLE;
  }

  if ((motorCommand[REAR] <= MINTHROTTLE) || (motorCommand[FRONT_RIGHT] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[FRONT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[RIGHT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR]  = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[LEFT]  = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[REAR] >= MAXCOMMAND) || (motorCommand[FRONT_RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[FRONT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[RIGHT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR]  = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[LEFT]  = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT] = MAXCOMMAND;
    motorMaxCommand[RIGHT] = MAXCOMMAND;
    motorMaxCommand[REAR]  = MAXCOMMAND;
    motorMaxCommand[LEFT]  = MAXCOMMAND;
    motorMinCommand[FRONT] = MINTHROTTLE;
    motorMinCommand[RIGHT] = MINTHROTTLE;
    motorMinCommand[REAR]  = MINTHROTTLE;
    motorMinCommand[LEFT]  = MINTHROTTLE;
  }
}


// @Kenny, check this one for v3.0
void processHardManuevers() {
  if (flightMode == ACRO) {
    if (receiverCommand[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
      motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
      motorMinCommand[RIGHT]       = MAXCOMMAND;
      motorMinCommand[REAR_LEFT]   = MAXCOMMAND;
      motorMinCommand[LEFT]        = MAXCOMMAND;
      motorMaxCommand[FRONT]       = minAcro;
      motorMaxCommand[REAR]        = minAcro;
      motorMaxCommand[REAR]        = minAcro;
      motorMaxCommand[FRONT_LEFT]  = minAcro;
    }
    else if (receiverCommand[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
      motorMinCommand[FRONT]       = MAXCOMMAND;
      motorMinCommand[REAR]        = MAXCOMMAND;
      motorMinCommand[REAR]        = MAXCOMMAND;
      motorMinCommand[FRONT_LEFT]  = MAXCOMMAND;
      motorMaxCommand[FRONT_RIGHT] = minAcro;
      motorMaxCommand[RIGHT]       = minAcro;
      motorMaxCommand[REAR_LEFT]   = minAcro;
      motorMaxCommand[LEFT]        = minAcro;
    }
    else if (receiverCommand[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
      motorMinCommand[FRONT]       = MAXCOMMAND;
      motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
      motorMinCommand[REAR]        = MAXCOMMAND;
      motorMinCommand[REAR_LEFT]   = MAXCOMMAND;
      motorMaxCommand[REAR]        = minAcro;
      motorMaxCommand[RIGHT]       = minAcro;
      motorMaxCommand[FRONT_LEFT]  = minAcro;
      motorMaxCommand[LEFT]        = minAcro;
    }
    else if (receiverCommand[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      motorMinCommand[REAR]        = MAXCOMMAND;
      motorMinCommand[RIGHT]       = MAXCOMMAND;
      motorMinCommand[FRONT_LEFT]  = MAXCOMMAND;
      motorMinCommand[LEFT]        = MAXCOMMAND;
      motorMaxCommand[FRONT]       = minAcro;
      motorMaxCommand[FRONT_RIGHT] = minAcro;
      motorMaxCommand[REAR]        = minAcro;
      motorMaxCommand[REAR_LEFT]   = minAcro;
    }
  }
}



#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

