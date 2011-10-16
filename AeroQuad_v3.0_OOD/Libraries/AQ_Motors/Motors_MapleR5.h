/*
  AeroQuad Maple32 v3.0 - October 2011
  Copyright (c) 2011 Mattias Welponer.  All rights reserved.
  An Open Source Maple STM32 based multicopter.
 
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


#ifndef _AEROQUAD_MOTORS_STM32_PWM_H_
#define _AEROQUAD_MOTORS_STM32_PWM_H_

#include <WProgram.h>
#include "Motors.h"

#define MOTORPIN1 12
#define MOTORPIN2 11
#define MOTORPIN3 27
#define MOTORPIN4 28
#define MOTORPIN5 2
#define MOTORPIN6 3
 
class Motors_PWM_MapleR5 : public Motors {
private:
  NB_Motors numbersOfMotors;
public:

  Motors_PWM_MapleR5() {
    numbersOfMotors = FOUR_Motors;
  }

  void initialize(NB_Motors numbers) {
    numbersOfMotors = numbers;
/*    timer_set_mode(TIMER4, TIMER_CH1, TIMER_PWM);
   timer_set_mode(TIMER4, TIMER_CH2, TIMER_PWM);
   timer_set_mode(TIMER4, TIMER_CH3, TIMER_PWM);
   timer_pause(TIMER4);
   timer_set_count(TIMER4, 0);
   timer_set_reload(TIMER4, 60000);   // Sets the period of the PWM signal
   timer_resume(TIMER4);              // enable outputs to esc's
   */
    
    Timer3.setPrescaleFactor(72);
    Timer3.setOverflow(20000);

    pinMode(MOTORPIN1, PWM);
    pinMode(MOTORPIN2, PWM);
    pinMode(MOTORPIN3, PWM);
    pinMode(MOTORPIN4, PWM);

    if (numbersOfMotors == SIX_Motors) {
      Timer2.setPrescaleFactor(72);
      Timer2.setOverflow(20000);

      pinMode(MOTORPIN5, PWM);
      pinMode(MOTORPIN6,  PWM);
    }
    
    commandAllMotors(1000);
  }

  void write() {
    Timer3.setCompare(TIMER_CH1, motorCommand[MOTOR1]);
    Timer3.setCompare(TIMER_CH2, motorCommand[MOTOR2]);
    Timer3.setCompare(TIMER_CH3, motorCommand[MOTOR3]);
    Timer3.setCompare(TIMER_CH4, motorCommand[MOTOR4]);
    if (numbersOfMotors == SIX_Motors) {  
           
    }
  }

  void commandAllMotors(int command) {
    Timer3.setCompare(TIMER_CH1, command);
    Timer3.setCompare(TIMER_CH2, command);
    Timer3.setCompare(TIMER_CH3, command);
    Timer3.setCompare(TIMER_CH4, command);
    if (numbersOfMotors == SIX_Motors) {      
    }
  }  
};

#endif

