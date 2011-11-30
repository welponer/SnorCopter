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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_HEX_Y6_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_HEX_Y6_MODE_H_

/*  
             UPPER/LOWER


       CW/CCW           CCW/CW
            
           0....Front....0  
           ......***......    
           ......***......
           ......***......    
           0....Back.....0  
      
                CW/CCW           
*/     


#define LEFT            MOTOR1
#define REAR            MOTOR2
#define RIGHT           MOTOR3
#define REAR_UNDER      MOTOR4
#define LEFT_UNDER      MOTOR5
#define RIGHT_UNDER     MOTOR6
//#define LASTMOTOR       MOTOR6+1

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int throttleCorrection = abs(motorAxisCommandYaw*3/6);
  motors->setMotorCommand(REAR,        (throttle - throttleCorrection)                        + (motorAxisCommandPitch*4/3) - (YAW_DIRECTION * motorAxisCommandYaw));
  motors->setMotorCommand(RIGHT,       (throttle - throttleCorrection) - motorAxisCommandRoll - (motorAxisCommandPitch*2/3) + (YAW_DIRECTION * motorAxisCommandYaw));  
  motors->setMotorCommand(LEFT,        (throttle - throttleCorrection) + motorAxisCommandRoll - (motorAxisCommandPitch*2/3) - (YAW_DIRECTION * motorAxisCommandYaw));
  motors->setMotorCommand(REAR_UNDER,  (throttle - throttleCorrection)                        + (motorAxisCommandPitch*4/3) + (YAW_DIRECTION * motorAxisCommandYaw));
  motors->setMotorCommand(RIGHT_UNDER, (throttle - throttleCorrection) - motorAxisCommandRoll - (motorAxisCommandPitch*2/3) - (YAW_DIRECTION * motorAxisCommandYaw));
  motors->setMotorCommand(LEFT_UNDER,  (throttle - throttleCorrection) + motorAxisCommandRoll - (motorAxisCommandPitch*2/3) + (YAW_DIRECTION * motorAxisCommandYaw));
}
/*
void processMinMaxCommand() {
  int delta;
    
  if ((motors->getMotorCommand(LEFT) <= MINTHROTTLE) || (motors->getMotorCommand(REAR_UNDER) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motors->setMaxCommand(RIGHT, constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
    motors->setMaxCommand(RIGHT_UNDER, constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
    motors->setMaxCommand(REAR, constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
    motors->setMaxCommand(REAR_UNDER, constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
  }
  else if ((motors->getMotorCommand(LEFT) >= MAXCOMMAND) || (motors->getMotorCommand(REAR_UNDER) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motors->setMinCommand(RIGHT, constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
    motors->setMinCommand(RIGHT_UNDER, constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
    motors->setMinCommand(REAR, constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
    motors->setMinCommand(REAR_UNDER, constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
  }     
  else {
    motors->setMaxCommand(RIGHT, MAXCOMMAND);
    motors->setMaxCommand(RIGHT_UNDER, MAXCOMMAND);
    motors->setMaxCommand(REAR, MAXCOMMAND); 
    motors->setMaxCommand(REAR_UNDER, MAXCOMMAND); 
    motors->setMinCommand(RIGHT, MINTHROTTLE);
    motors->setMinCommand(RIGHT_UNDER, MINTHROTTLE);
    motors->setMinCommand(REAR, MINTHROTTLE);
    motors->setMinCommand(REAR_UNDER, MINTHROTTLE);
  }

  if ((motors->getMotorCommand(REAR) <= MINTHROTTLE) || (motors->getMotorCommand(RIGHT) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motors->setMaxCommand(LEFT, constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
    motors->setMaxCommand(LEFT_UNDER, constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
    motors->setMaxCommand(REAR, constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
    motors->setMaxCommand(REAR_UNDER, constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
  }
  else if ((motors->getMotorCommand(REAR) >= MAXCOMMAND) || (motors->getMotorCommand(RIGHT) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motors->setMinCommand(LEFT, constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
    motors->setMinCommand(LEFT_UNDER, constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
    motors->setMinCommand(REAR, constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
    motors->setMinCommand(REAR_UNDER, constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
  }     
  else {
    motors->setMaxCommand(LEFT, MAXCOMMAND);
    motors->setMaxCommand(LEFT_UNDER, MAXCOMMAND);
    motors->setMaxCommand(REAR, MAXCOMMAND);
    motors->setMaxCommand(REAR_UNDER, MAXCOMMAND);
    motors->setMinCommand(LEFT, MINTHROTTLE);
    motors->setMinCommand(LEFT_UNDER, MINTHROTTLE);
    motors->setMinCommand(REAR, MINTHROTTLE);
    motors->setMinCommand(REAR_UNDER, MINTHROTTLE);
  }
}

void processHardManuevers() {
    if (receiver->getData(ROLL) < MINCHECK) {        // Maximum Left Roll Rate
      motors->setMinCommand(RIGHT, MAXCOMMAND);
      motors->setMinCommand(RIGHT_UNDER, MAXCOMMAND);
      motors->setMaxCommand(LEFT, MINACRO);
      motors->setMaxCommand(LEFT_UNDER, MINACRO);
    }
    else if (receiver->getData(ROLL) > MAXCHECK) {   // Maximum Right Roll Rate
      motors->setMinCommand(LEFT, MAXCOMMAND);
      motors->setMinCommand(LEFT_UNDER, MAXCOMMAND);
      motors->setMaxCommand(RIGHT, MINACRO);
      motors->setMaxCommand(RIGHT_UNDER, MINACRO);
    }
    else if (receiver->getData(PITCH) < MINCHECK) {  // Maximum Nose Up Pitch Rate
      motors->setMinCommand(LEFT, MAXCOMMAND);
      motors->setMinCommand(LEFT_UNDER, MAXCOMMAND);
      motors->setMinCommand(RIGHT, MAXCOMMAND);
      motors->setMinCommand(RIGHT_UNDER, MAXCOMMAND);
      motors->setMaxCommand(REAR_UNDER, MINACRO);
      motors->setMaxCommand(REAR, MINACRO);
    }
    else if (receiver->getData(PITCH) > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      motors->setMinCommand(LEFT, MINACRO);
      motors->setMinCommand(LEFT_UNDER, MINACRO);
      motors->setMinCommand(RIGHT, MINACRO);
      motors->setMinCommand(RIGHT_UNDER, MINACRO);
      motors->setMaxCommand(REAR_UNDER, MAXCOMMAND);
      motors->setMaxCommand(REAR, MAXCOMMAND);
    }
}
*/
#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

