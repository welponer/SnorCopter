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


#ifndef _AEROQUAD_MAGNETOMETER_CHR6DM_H_
#define _AEROQUAD_MAGNETOMETER_CHR6DM_H_

#include "Compass.h"
#include <WProgram.h>
#include <Platform_CHR6DM.h>

class Magnetometer_CHR6DM : public Compass{
private:
  CHR6DM *chr6dm;
  float absoluteHeading;
  
public:
  Magnetometer_CHR6DM() : Compass() {
  }
  
  void setChr6dm(CHR6DM *chr6dm) {
    this->chr6dm = chr6dm;
  }

  void initialize() {
    absoluteHeading = 0;
  }

  void measure(float roll, float pitch) {
    heading = chr6dm->data.yaw; //this hardly needs any filtering :)
    // Change from +/-180 to 0-360
    if (heading < 0) absoluteHeading = 360 + heading;
    else absoluteHeading = heading;
  }
};



#endif