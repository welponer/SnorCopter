/*
  AeroQuad v3.0 - April 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multiflight.
 
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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

/*
       CW  0....Front....0 CCW
           ......***......    
           ......***......    
           ......***......    
      CCW  0....Back.....0  CW
*/


#define FRONT_LEFT  MOTOR1
#define REAR_RIGHT  MOTOR2
#define FRONT_RIGHT MOTOR3
#define REAR_LEFT   MOTOR4
//#define LASTMOTOR   MOTOR4+1

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int throttleCorrection = abs(motorAxisCommandYaw*2/4);
//  const int throttleCorrection = 0;
  motors->setMotorCommand(FRONT_LEFT,  (throttle - throttleCorrection) - motorAxisCommandPitch - motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw));
  motors->setMotorCommand(FRONT_RIGHT, (throttle - throttleCorrection) - motorAxisCommandPitch + motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw));
  motors->setMotorCommand(REAR_LEFT,   (throttle - throttleCorrection) + motorAxisCommandPitch - motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw));
  motors->setMotorCommand(REAR_RIGHT,  (throttle - throttleCorrection) + motorAxisCommandPitch + motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw));
}

/*
void processMinMaxCommand() {
  int delta;
  
  if ((motors->getMotorCommand(FRONT_LEFT) <= MINTHROTTLE) || (motors->getMotorCommand(REAR_RIGHT) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motors->setMaxCommand(FRONT_RIGHT, constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
    motors->setMaxCommand(REAR_LEFT, constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
  }
  else if ((motors->getMotorCommand(FRONT_LEFT) >= MAXCOMMAND) || (motors->getMotorCommand(REAR_RIGHT) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motors->setMinCommand(FRONT_RIGHT, constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
    motors->setMinCommand(REAR_LEFT, constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
  }     
  else {
    motors->setMaxCommand(FRONT_RIGHT, MAXCOMMAND);
    motors->setMaxCommand(REAR_LEFT, MAXCOMMAND); 
    motors->setMinCommand(FRONT_RIGHT, MINTHROTTLE);
    motors->setMinCommand(REAR_LEFT, MINTHROTTLE);
  }

  if ((motors->getMotorCommand(REAR_LEFT) <= MINTHROTTLE) || (motors->getMotorCommand(FRONT_RIGHT) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motors->setMaxCommand(FRONT_LEFT, constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
    motors->setMaxCommand(REAR_RIGHT, constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
  }
  else if ((motors->getMotorCommand(REAR_LEFT) >= MAXCOMMAND) || (motors->getMotorCommand(FRONT_RIGHT) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motors->setMinCommand(FRONT_LEFT, constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
    motors->setMinCommand(REAR_RIGHT, constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
  }     
  else {
    motors->setMaxCommand(FRONT_LEFT, MAXCOMMAND);
    motors->setMaxCommand(REAR_RIGHT, MAXCOMMAND);
    motors->setMinCommand(FRONT_LEFT, MINTHROTTLE);
    motors->setMinCommand(REAR_RIGHT, MINTHROTTLE);
  }
}

void processHardManuevers() {
    if (receiver->getData(ROLL) < MINCHECK) {        // Maximum Left Roll Rate
      motors->setMinCommand(FRONT_RIGHT, MAXCOMMAND);
      motors->setMinCommand(REAR_RIGHT, MAXCOMMAND);
      motors->setMaxCommand(FRONT_LEFT, MINACRO);
      motors->setMaxCommand(REAR_LEFT, MINACRO);
    }
    else if (receiver->getData(ROLL) > MAXCHECK) {   // Maximum Right Roll Rate
      motors->setMinCommand(FRONT_LEFT, MAXCOMMAND);
      motors->setMinCommand(REAR_LEFT, MAXCOMMAND);
      motors->setMaxCommand(FRONT_RIGHT, MINACRO);
      motors->setMaxCommand(REAR_RIGHT, MINACRO);
    }
    else if (receiver->getData(PITCH) < MINCHECK) {  // Maximum Nose Up Pitch Rate
      motors->setMinCommand(FRONT_LEFT, MAXCOMMAND);
      motors->setMinCommand(FRONT_RIGHT, MAXCOMMAND);
      motors->setMaxCommand(REAR_LEFT, MINACRO);
      motors->setMaxCommand(REAR_RIGHT, MINACRO);
    }
    else if (receiver->getData(PITCH) > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      motors->setMinCommand(REAR_LEFT, MAXCOMMAND);
      motors->setMinCommand(REAR_RIGHT, MAXCOMMAND);
      motors->setMaxCommand(FRONT_LEFT, MINACRO);
      motors->setMaxCommand(FRONT_RIGHT, MINACRO);
    } 
}

*/

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

