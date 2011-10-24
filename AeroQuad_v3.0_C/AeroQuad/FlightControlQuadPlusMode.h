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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_

/*  
                 CW
            
           0....Front....0  
           ......***......    
      CCW  ......***......  CCW
           ......***......    
           0....Back.....0  
      
                 CW
*/     


#define FRONT MOTOR1
#define REAR  MOTOR2
#define RIGHT MOTOR3
#define LEFT  MOTOR4
#define LASTMOTOR MOTOR4+1

void applyMotorCommand() {
  const int throttleCorrection = abs(motorAxisCommandYaw*2/4);
  motorCommand[FRONT] = (throttle - throttleCorrection) - motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR] =  (throttle - throttleCorrection) + motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[RIGHT] = (throttle - throttleCorrection) - motorAxisCommandRoll  + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[LEFT] =  (throttle - throttleCorrection) + motorAxisCommandRoll  + (YAW_DIRECTION * motorAxisCommandYaw);
}

void processMinMaxCommand() {
  if ((motorCommand[FRONT] <= MINTHROTTLE) || (motorCommand[REAR] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[RIGHT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[LEFT] =  constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[FRONT] >= MAXCOMMAND) || (motorCommand[REAR] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[RIGHT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[LEFT] =  constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[RIGHT] = MAXCOMMAND;
    motorMaxCommand[LEFT] =  MAXCOMMAND;
    motorMinCommand[RIGHT] = MINTHROTTLE;
    motorMinCommand[LEFT] =  MINTHROTTLE;
  }

  if ((motorCommand[LEFT] <= MINTHROTTLE) || (motorCommand[RIGHT] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE; 
    motorMaxCommand[FRONT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR]  = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[LEFT] >= MAXCOMMAND) || (motorCommand[RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[FRONT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR]  = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT] = MAXCOMMAND; 
    motorMaxCommand[REAR]  = MAXCOMMAND;
    motorMinCommand[FRONT] = MINTHROTTLE;
    motorMinCommand[REAR]  = MINTHROTTLE;
  }
}

void processHardManuevers() {
  if (flightMode == ACRO) {
    if (receiverCommand[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
      motorMinCommand[RIGHT] = MAXCOMMAND;
      motorMaxCommand[LEFT] = minAcro;
    }
    else if (receiverCommand[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
      motorMinCommand[LEFT] = MAXCOMMAND;
      motorMaxCommand[RIGHT] = minAcro;
    }
    else if (receiverCommand[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
      motorMinCommand[FRONT] = MAXCOMMAND;
      motorMaxCommand[REAR] = minAcro;
    }
    else if (receiverCommand[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      motorMinCommand[REAR] = MAXCOMMAND;
      motorMaxCommand[FRONT] = minAcro;
    }
  }
}


#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_
