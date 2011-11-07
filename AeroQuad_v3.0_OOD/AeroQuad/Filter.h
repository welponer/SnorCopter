/*
  AeroQuad Maple32 v3.0 - November 2011
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

#ifndef _AQ_Filter_LowCheby4_H_
#define _AQ_Filter_LowCheby4_H_

class Filter_LowCheby4 {
private:
  float inputTm[4];
  float outputTm[4];
  
  float b[5], a[5];
  
public:
  Filter_LowCheby4(void) {
     // MATLAB: cheby2(4,60,12.5/50)
    b[0] = 0.001893594048567;
    b[1] = -0.002220262954039;
    b[2] = 0.003389066536478;
    b[3] = -0.002220262954039;
    b[4] = 0.001893594048567;
  
    a[1] = -3.362256889209355;
    a[2] = 4.282608240117919;
    a[3] = -2.444765517272841;
    a[4] = 0.527149895089809;
  }
  
  void initialize(float value = 0) { 
    for ( int i = 0; i < 4; i++) {
       inputTm[i] = value;
       outputTm[i] = value;
    }
  }
  
  void initialize1G() {  
    initialize(-9.8065);
  }
  
  float calculate(float input) {
    float output;
    
    output = b[0] * input + 
             b[1] * inputTm[0] + 
             b[2] * inputTm[1] +
             b[3] * inputTm[2] +
             b[4] * inputTm[3] -
             a[1] * outputTm[0] -
             a[2] * outputTm[1] -
             a[3] * outputTm[2] -
             a[4] * outputTm[3];
  
    inputTm[3] = inputTm[2];
    inputTm[2] = inputTm[1];
    inputTm[1] = inputTm[0];
    inputTm[0] = input;
    
    outputTm[3] = outputTm[2];
    outputTm[2] = outputTm[1];
    outputTm[1] = outputTm[0];
    outputTm[0] = output;
      
    return output;
  }
};

////////////////////////////////////////////////////////////////////////////////

#endif
