/*
  AeroQuad v2.5 Beta 1 - July 2011
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

////////////////////////////////////////////////////////////////////////////////
//  Matrix Multiply
//  Multiply matrix A times matrix B, matrix A dimension m x n, matrix B dimension n x p
//  Result placed in matrix C, dimension m x p
//
//  Call as: matrixMultiply(m, n, p, C, A, B)
////////////////////////////////////////////////////////////////////////////////

void matrixMultiply(int aRows, int aCols_bRows, int bCols, float matrixC[], float matrixA[], float matrixB[])
{
  //int i, j, k;

  for (int i = 0; i < aRows * bCols; i++)
  {
    matrixC[i] = 0.0;
  }

  for (int i = 0; i < aRows; i++)
  {
    for(int j = 0; j < aCols_bRows; j++)
    {
      for(int k = 0;  k < bCols; k++)
      {
       matrixC[i * bCols + k] += matrixA[i * aCols_bRows + j] * matrixB[j * bCols + k];
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////

struct fourthOrderData
{
  float  inputTm1,  inputTm2,  inputTm3,  inputTm4;
  float outputTm1, outputTm2, outputTm3, outputTm4;
} fourthOrder[3];

#define AX_FILTER 0
#define AY_FILTER 1
#define AZ_FILTER 2

float computeFourthOrder(float currentInput, struct fourthOrderData *filterParameters)
{
  // cheby2(4,60,12.5/50)
  #define _b0  0.001893594048567
  #define _b1 -0.002220262954039
  #define _b2  0.003389066536478
  #define _b3 -0.002220262954039
  #define _b4  0.001893594048567
  
  #define _a1 -3.362256889209355
  #define _a2  4.282608240117919
  #define _a3 -2.444765517272841
  #define _a4  0.527149895089809
  
  float output;
  
  output = _b0 * currentInput                + 
           _b1 * filterParameters->inputTm1  + 
           _b2 * filterParameters->inputTm2  +
           _b3 * filterParameters->inputTm3  +
           _b4 * filterParameters->inputTm4  -
           _a1 * filterParameters->outputTm1 -
           _a2 * filterParameters->outputTm2 -
           _a3 * filterParameters->outputTm3 -
           _a4 * filterParameters->outputTm4;

  filterParameters->inputTm4 = filterParameters->inputTm3;
  filterParameters->inputTm3 = filterParameters->inputTm2;
  filterParameters->inputTm2 = filterParameters->inputTm1;
  filterParameters->inputTm1 = currentInput;
  
  filterParameters->outputTm4 = filterParameters->outputTm3;
  filterParameters->outputTm3 = filterParameters->outputTm2;
  filterParameters->outputTm2 = filterParameters->outputTm1;
  filterParameters->outputTm1 = output;
    
  return output;
}

void setupFourthOrder(void)
{
  fourthOrder[AX_FILTER].inputTm1 = 0.0;
  fourthOrder[AX_FILTER].inputTm2 = 0.0;
  fourthOrder[AX_FILTER].inputTm3 = 0.0;
  fourthOrder[AX_FILTER].inputTm4 = 0.0;
  
  fourthOrder[AX_FILTER].outputTm1 = 0.0;
  fourthOrder[AX_FILTER].outputTm2 = 0.0;
  fourthOrder[AX_FILTER].outputTm3 = 0.0;
  fourthOrder[AX_FILTER].outputTm4 = 0.0;
  
  //////////
  
  fourthOrder[AY_FILTER].inputTm1 = 0.0;
  fourthOrder[AY_FILTER].inputTm2 = 0.0;
  fourthOrder[AY_FILTER].inputTm3 = 0.0;
  fourthOrder[AY_FILTER].inputTm4 = 0.0;
  
  fourthOrder[AY_FILTER].outputTm1 = 0.0;
  fourthOrder[AY_FILTER].outputTm2 = 0.0;
  fourthOrder[AY_FILTER].outputTm3 = 0.0;
  fourthOrder[AY_FILTER].outputTm4 = 0.0;
  
  //////////
  
  fourthOrder[AZ_FILTER].inputTm1 = -9.8065;
  fourthOrder[AZ_FILTER].inputTm2 = -9.8065;
  fourthOrder[AZ_FILTER].inputTm3 = -9.8065;
  fourthOrder[AZ_FILTER].inputTm4 = -9.8065;
  
  fourthOrder[AZ_FILTER].outputTm1 = -9.8065;
  fourthOrder[AZ_FILTER].outputTm2 = -9.8065;
  fourthOrder[AZ_FILTER].outputTm3 = -9.8065;
  fourthOrder[AZ_FILTER].outputTm4 = -9.8065;
  }

////////////////////////////////////////////////////////////////////////////////