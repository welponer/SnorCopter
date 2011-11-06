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

#ifndef _AEROQUAD_RECEIVER_H_
#define _AEROQUAD_RECEIVER_H_

#include <WProgram.h>

#define PWM2RAD 0.002 //  Based upon 5RAD for full stick movement, you take this times the RAD to get the PWM conversion factor

// Receiver variables
#define TIMEOUT 25000
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINDELTA 200
#define MINCHECK MINCOMMAND + 100
#define MAXCHECK MAXCOMMAND - 100
#define MINTHROTTLE MINCOMMAND + 100
#define LEVELOFF 100
#define MAX_NB_CHANNEL 8

int lastChannel = 0;

float receiverXmitFactor = 0.0;
int receiverData[MAX_NB_CHANNEL] = {0,0,0,0,0,0,0,0};
//int receiverTrim[3] = {0,0,0};
int receiverZero[3] = {0,0,0};
int receiverCommand[MAX_NB_CHANNEL] = {0,0,0,0,0,0,0,0};
int receiverCommandSmooth[MAX_NB_CHANNEL] = {0,0,0,0,0,0,0,0};
float receiverSlope[MAX_NB_CHANNEL] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float receiverOffset[MAX_NB_CHANNEL] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float receiverSmoothFactor[MAX_NB_CHANNEL] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

void initializeReceiverParam(int nbChannel = 6) {
  
  lastChannel = nbChannel;

  receiverCommand[ROLL] = 1500;
  receiverCommand[PITCH] = 1500;
  receiverCommand[YAW] = 1500;
  receiverCommand[THROTTLE] = 1000;
  receiverCommand[MODE] = 1000;
  receiverCommand[AUX] = 1000;
  receiverCommand[AUX+1] = 1000;
  receiverCommand[AUX+2] = 1000;
  
  for (byte channel = ROLL; channel < lastChannel; channel++)
    receiverCommandSmooth[channel] = 1.0;
  for (byte channel = ROLL; channel < THROTTLE; channel++)
    receiverZero[channel] = 1500;
	
  for (byte channel = ROLL; channel < lastChannel; channel++)
    receiverSlope[channel] = 1;
  for (byte channel = ROLL; channel < lastChannel; channel++)
    receiverOffset[channel] = 1;
  for (byte channel = ROLL; channel < lastChannel; channel++)
    receiverSmoothFactor[channel] = 1; 
}
  
void readReceiver();
void setChannelValue(byte channel,int value);
  
// return the smoothed & scaled number of radians/sec in stick movement - zero centered
const float getReceiverSIData(byte channel) {
  return ((receiverCommand[channel] - receiverZero[channel]) * (2.5 * PWM2RAD));  // +/- 2.5RPS 50% of full rate
}

#endif



