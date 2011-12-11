/*
  AeroQuad v3.0 - May 2011
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

#ifndef _AEROQUAD_MAX_SONAR_RANGE_FINDER_H_
#define _AEROQUAD_MAX_SONAR_RANGE_FINDER_H_

// @see http://www.arduino.cc/playground/Main/MaxSonar

#include "RangeFinder.h"

//
// default unit are centimeter
//

// default min max range constrain

void inititalizeRangeFinder(byte idx) {

  int maxRangeFinderRange = 700;
  int minRangeFinderRange = 30;
  
  pinMode(rangeFinderPins[idx], INPUT);
}

/**
 * inches * 2.54 = cm
 */
void readRangeFinderDistanceSum(byte idx) {
  rangeFinderRangeSum[idx] += (analogRead(rangeFinderPins[idx]) * 1.8333);// 2.54);
  rangeFinderSampleCount[idx]++;
}

void evaluateDistanceFromSample(byte idx) {
  rangeFinderRange[idx] = rangeFinderRangeSum[idx] / rangeFinderSampleCount[idx];
  rangeFinderRangeSum[idx] = 0;
  rangeFinderSampleCount[idx] = 0;
}

/**
 * @return true if we can use safely the sonar
 */ 
boolean isInRangeOfRangeFinder(byte idx) {
  return ((rangeFinderRange[idx] < maxRangeFinderRange) && 
          (rangeFinderRange[idx] > minRangeFinderRange));
}


#endif








