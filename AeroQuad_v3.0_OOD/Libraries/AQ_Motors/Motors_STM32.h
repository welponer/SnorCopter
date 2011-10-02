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


#ifndef _AEROQUAD_MOTORS_PWM_H_
#define _AEROQUAD_MOTORS_PWM_H_

#include <WProgram.h>

#include "Motors.h"


  #define MOTORPIN0    3
  #define MOTORPIN1    9
  #define MOTORPIN2   10
  #define MOTORPIN3   11
  #define MOTORPIN4    5   
  #define MOTORPIN5    6  
  

class Motors_STM32 : public Motors {
private:
  NB_Motors numbersOfMotors;
public:

  Motors_STM32() {
  }

  void initialize(NB_Motors numbers) {
 
	  pinMode(MOTORPIN0, OUTPUT);
	  pinMode(MOTORPIN1, OUTPUT);
	  pinMode(MOTORPIN2, OUTPUT);
	  pinMode(MOTORPIN3, OUTPUT);

    
    numbersOfMotors = numbers;
    commandAllMotors(1000);
  }

  void write() {
    analogWrite(MOTORPIN0, motorCommand[MOTOR1] / 8);
    analogWrite(MOTORPIN1, motorCommand[MOTOR2] / 8);
    analogWrite(MOTORPIN2, motorCommand[MOTOR3] / 8);
    analogWrite(MOTORPIN3, motorCommand[MOTOR4] / 8); 
  }

  void commandAllMotors(int command) {
    analogWrite(MOTORPIN0, command / 8);
    analogWrite(MOTORPIN1, command / 8);
    analogWrite(MOTORPIN2, command / 8);
    analogWrite(MOTORPIN3, command / 8);
  }  
};

#endif