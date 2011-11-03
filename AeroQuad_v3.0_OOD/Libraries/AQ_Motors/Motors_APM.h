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

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#ifndef _AEROQUAD_MOTORS_APM_H_
#define _AEROQUAD_MOTORS_APM_H_

#include <WProgram.h>
#include <APM_RC.h>
#include "Motors.h"


class Motors_APM : public Motors {
private:
  NB_Motors numbersOfMotors;
public:

  Motors_APM() {
  }

  void initialize(NB_Motors numbers) {
    numbersOfMotors = numbers;
    commandAllMotors(1000);
    Serial.println("init Motors_APM: done");
  }

  void write() {
    writeMotorCommand(0,motorCommand[MOTOR1]);
    writeMotorCommand(1,motorCommand[MOTOR2]);
    writeMotorCommand(2,motorCommand[MOTOR3]);
    writeMotorCommand(3,motorCommand[MOTOR4]);
	if (numbersOfMotors == SIX_Motors || numbersOfMotors == HEIGHT_Motors) {
	  writeMotorCommand(6,motorCommand[MOTOR5]);
	  writeMotorCommand(7,motorCommand[MOTOR6]);
	}
	if (numbersOfMotors == HEIGHT_Motors) {
	  writeMotorCommand(9,motorCommand[MOTOR7]);
	  writeMotorCommand(10,motorCommand[MOTOR8]);
	}
	force_Out0_Out1();
	force_Out2_Out3();
	if (numbersOfMotors == SIX_Motors || numbersOfMotors == HEIGHT_Motors) {
	  force_Out6_Out7();
	}
  }

  void commandAllMotors(int command) {
    for( int i = 0; i < 8; i++) 
      motorCommand[i] = command;
    write();
  }
  
};

#endif

#endif