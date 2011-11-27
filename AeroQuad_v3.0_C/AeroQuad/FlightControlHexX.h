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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_HEX_X_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_HEX_X_MODE_H_

/*  
            CW       CCW
            
           0....Front....0  
           ......***......    
     CCW   ......***......    CW
           ......***......    
           0....Back.....0  
      
           CW       CCW           
*/     

// @todo Kenny NOT FLIGHT TESTED



#define FRONT_LEFT  MOTOR1
#define FRONT_RIGHT MOTOR2
#define RIGHT       MOTOR3
#define REAR_RIGHT  MOTOR4
#define REAR_LEFT   MOTOR5
#define LEFT        MOTOR6
#define LASTMOTOR   MOTOR6+1

void applyMotorCommand() {
  const int throttleCorrection = abs(motorAxisCommandYaw*3/6);
  motorCommand[FRONT_LEFT]  = (throttle - throttleCorrection) + motorAxisCommandRoll/2 - motorAxisCommandPitch/2 - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT]  = (throttle - throttleCorrection) - motorAxisCommandRoll/2 + motorAxisCommandPitch/2 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT] = (throttle - throttleCorrection) - motorAxisCommandRoll/2 - motorAxisCommandPitch/2 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT]   = (throttle - throttleCorrection) + motorAxisCommandRoll/2 + motorAxisCommandPitch/2 - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[RIGHT]       = (throttle - throttleCorrection) - motorAxisCommandRoll                             - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[LEFT]        = (throttle - throttleCorrection) + motorAxisCommandRoll                             + (YAW_DIRECTION * motorAxisCommandYaw);
}

void processMinMaxCommand() {
  
  if (receiverCommand[THROTTLE] > MAXCHECK) { // if the throttle is about the max, we used tue PID values!
    return;
  }
  
  if ((motorCommand[FRONT_LEFT] <= MINTHROTTLE) || (motorCommand[REAR_LEFT] <= MINTHROTTLE) || (motorCommand[RIGHT] <= MINTHROTTLE)) {
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[REAR_RIGHT] =  constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[FRONT_RIGHT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[LEFT] =        constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[FRONT_LEFT] >= MAXCOMMAND) || (motorCommand[REAR_LEFT] >= MAXCOMMAND) || (motorCommand[RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[REAR_RIGHT] =  constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[FRONT_RIGHT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[LEFT] =        constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[REAR_RIGHT] =  MAXCOMMAND;
    motorMaxCommand[FRONT_RIGHT] = MAXCOMMAND; 
    motorMaxCommand[LEFT] =        MAXCOMMAND; 
    motorMinCommand[REAR_RIGHT] =  MINTHROTTLE;
    motorMinCommand[FRONT_RIGHT] = MINTHROTTLE;
    motorMinCommand[LEFT] =        MINTHROTTLE;
  }

  if ((motorCommand[REAR_RIGHT] <= MINTHROTTLE) || (motorCommand[FRONT_RIGHT] <= MINTHROTTLE) || (motorCommand[LEFT] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[FRONT_LEFT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_LEFT] =  constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[RIGHT] =      constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[REAR_RIGHT] >= MAXCOMMAND) || (motorCommand[FRONT_RIGHT] >= MAXCOMMAND) || (motorCommand[LEFT]  >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[FRONT_LEFT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_LEFT] =  constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[RIGHT] =      constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_LEFT] = MAXCOMMAND;
    motorMaxCommand[REAR_LEFT] =  MAXCOMMAND;
    motorMaxCommand[RIGHT] =      MAXCOMMAND;
    motorMinCommand[FRONT_LEFT] = MINTHROTTLE;
    motorMinCommand[REAR_LEFT] =  MINTHROTTLE;
    motorMinCommand[RIGHT] =      MINTHROTTLE;
  }
}

//void processHardManuevers() {
//
//  if (receiverCommand[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
//    motorMinCommand[RIGHT] =       MAXCOMMAND;
//    motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
//    motorMinCommand[REAR_RIGHT] =  MAXCOMMAND;
//    motorMaxCommand[LEFT] =        minAcro;
//    motorMaxCommand[FRONT_LEFT] =  minAcro;
//    motorMaxCommand[REAR_LEFT] =   minAcro;
//  }
//  else if (receiverCommand[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
//    motorMinCommand[LEFT] =        MAXCOMMAND;
//    motorMinCommand[FRONT_LEFT] =  MAXCOMMAND;
//    motorMinCommand[REAR_LEFT] =   MAXCOMMAND;
//    motorMaxCommand[RIGHT] =       minAcro;
//    motorMaxCommand[FRONT_RIGHT] = minAcro;
//    motorMaxCommand[REAR_RIGHT] =  minAcro;
//  }
//  else if (receiverCommand[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
//    motorMinCommand[FRONT_LEFT] =  MAXCOMMAND;
//    motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
//    motorMaxCommand[REAR_LEFT] =   minAcro;
//    motorMaxCommand[REAR_RIGHT] =  minAcro;
//  }
//  else if (receiverCommand[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
//    motorMinCommand[REAR_LEFT] =   MAXCOMMAND;
//    motorMinCommand[REAR_RIGHT] =  MAXCOMMAND;
//    motorMaxCommand[FRONT_LEFT] =  minAcro;
//    motorMaxCommand[FRONT_RIGHT] = minAcro;
//  }
//}

#endif  // #define _AQ_PROCESS_FLIGHT_CONTROL_HEX_X_MODE_H_
