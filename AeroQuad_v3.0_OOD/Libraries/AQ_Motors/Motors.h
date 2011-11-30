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


#ifndef _AEROQUAD_MOTORS_H_
#define _AEROQUAD_MOTORS_H_

#include <WProgram.h>

#define MOTOR1 0
#define MOTOR2 1
#define MOTOR3 2
#define MOTOR4 3
#define MOTOR5 4
#define MOTOR6 5
#define MOTOR7 6
#define MOTOR8 7
#define MINCOMMAND 1000
#define MAXCOMMAND 2000


class Motors {
public:
  int* motorCommand;
  
  int* pinNumber;
  byte motorChannels;
  int minAcro;
  
public:

  Motors(byte number = 4) {
    motorChannels = number;
    motorCommand = (int*)malloc(motorChannels);
    pinNumber = (int*)malloc(motorChannels);
    minAcro = 1300;
  }
	
  virtual void initialize( void) {}
  
  virtual void write( void) {}
  virtual void commandAllMotors( int command) {}

  void setMotorCommand( byte motor, int command) {
    motorCommand[motor] = command;
  }

  int getMotorCommand( byte motor) {
    return motorCommand[motor];
  }
  
  void pulseMotors( int command) {
  
  }
  
  void setMaxCommand( byte motor, int maxCommand) {
    motorMaxCommand[motor] = maxCommand;
  }
  
  void setMinCommand( byte motor, int minCommand) {
    motorMinCommand[motor] = minCommand;
  }
  
  void constrainMotors( void) {
    for (byte motor = 0; motor < motorChannels; motor++) {
      motorCommand[motor] = constrain(motorCommand[motor], motorMinCommand[motor], motorMaxCommand[motor]);
    }
  }
  
};



#endif