//=====================================================================================================
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
//=====================================================================================================

#ifndef _AQ_KINEMATICS_MEW_MARG_
#define _AQ_KINEMATICS_MEW_MARG_

#include "Kinematics.h"


////////////////////////////////////////////////////////////////////////////////
// MARG Variable Definitions
////////////////////////////////////////////////////////////////////////////////
//=====================================================================================================
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
//=====================================================================================================

////////////////////////////////////////////////////////////////////////////////
// MARG Variable Definitions
////////////////////////////////////////////////////////////////////////////////
float accelMagnitude;
float ax, ay, az; 
float bx, bz;
float dcm[9];
float exAcc = 0;
float eyAcc = 0;
float ezAcc = 0;
float exMag, eyMag, ezMag;
float exInt, eyInt, ezInt;  // scaled integral error
float gx, gy, gz;
float hx, hy, hz;
float kiAcc;                // integral gain governs rate of convergence of gyroscope biases
float kiMag;                // integral gain governs rate of convergence of gyroscope biases
float kpAcc;                // proportional gain governs rate of convergence to accelerometer/magnetometer
float kpMag;                // proportional gain governs rate of convergence to accelerometer/magnetometer
float halfT;                // half the sample period
float mx, my, mz;
float norm;
float q0, q1, q2, q3;       // quaternion elements representing the estimated orientation
float q0i, q1i, q2i, q3i;
float vx, vy, vz;
float wx, wy, wz;

float q0q0, q0q1, q0q2, q0q3;
float q1q1, q1q2, q1q3;
float q2q2, q2q3;
float q3q3;

float hEst;
float integrator1 = 0;
float integrator2 = 0;
float kHest = -0.008;
float kVz = -0.00001;
float sum1;
float sum2;
float sum3;
float vzEst = 0;

unsigned int margCurrentTime;
unsigned int margPreviousTime;
boolean margFirstPass = 1;
  
////////////////////////////////////////////////////////////////////////////////
// Marg Update
////////////////////////////////////////////////////////////////////////////////

void calculateKinematics(float rollRate,           float pitchRate,     float yawRate,       
                         float accelRoll,          float accelPitch,    float accelYaw, 
                         float oneG,               float magX,          float magY,
                         float G_Dt)
{						 
  if (margFirstPass == 1)
  {
    margCurrentTime  = micros();
    margPreviousTime = margCurrentTime;
    margFirstPass    = 0;
  }
  
  margCurrentTime = micros();
  halfT = float((margCurrentTime-margPreviousTime))/2000000;
  margPreviousTime = margCurrentTime;
    
  // normalize the measurements
  accelMagnitude = sqrt(accelRoll*accelRoll +\
                        accelPitch*accelPitch +\
                        accelYaw*accelYaw);       
  ax = accelRoll / accelMagnitude;
  ay = accelPitch / accelMagnitude;
  az = accelYaw / accelMagnitude;
  
  // estimated direction of gravity (v)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  
  // If the magnitude of the accel vector is within the cutoff limit the accel vector is useable for drift
  // compensation.  Compute the error terms and set the integral gain term to it's operational value.
  // If the magnitude of the accel vector is not within the cutoff, hold the previously computed error terms
  // and zero the integral gain term.
  // This should prevent the integral term from running away during accelerated flight and also prevent any
  // transients on both the attitudes and bias corrected rates when the cutoff switches back and forth.
  // That's the idea at any rate.
  
  if (abs(accelMagnitude - oneG) < accelCutoff)
  {
    exAcc = (vy*az - vz*ay);
    eyAcc = (vz*ax - vx*az);
    ezAcc = (vx*ay - vy*ax);
    kiAcc = 0.005;
//    digitalWrite(READY_LED, ON);
  }
  else
  {
    kiAcc = 0.0;
//    digitalWrite(READY_LED, OFF);
  }
  
  #if defined(HMC5843) | defined(HMC5883)
    norm = sqrt(mag.value[XAXIS]*mag.value[XAXIS] +\
                mag.value[YAXIS]*mag.value[YAXIS] +\
                mag.value[ZAXIS]*mag.value[ZAXIS]);          
    mx = mag.value[XAXIS] / norm;
    my = mag.value[YAXIS] / norm;
    mz = mag.value[ZAXIS] / norm;         
    
    // compute reference direction of flux
    hx = mx * 2*(0.5 - q2q2 - q3q3) + my * 2*(q1q2 - q0q3)       + mz * 2*(q1q3 + q0q2);
    hy = mx * 2*(q1q2 + q0q3)       + my * 2*(0.5 - q1q1 - q3q3) + mz * 2*(q2q3 - q0q1);
    hz = mx * 2*(q1q3 - q0q2)       + my * 2*(q2q3 + q0q1)       + mz * 2*(0.5 - q1q1 - q2q2);

    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz;    

    // estimated direction of flux (w)
    wx = bx * 2*(0.5 - q2q2 - q3q3) + bz * 2*(q1q3 - q0q2);
    wy = bx * 2*(q1q2 - q0q3)       + bz * 2*(q0q1 + q2q3);
    wz = bx * 2*(q0q2 + q1q3)       + bz * 2*(0.5 - q1q1 - q2q2);

    exMag = (my*wz - mz*wy);
    eyMag = (mz*wx - mx*wz);
    ezMag = (mx*wy - my*wx);

    #define X_MAG_INTEGRAL exMag*kiMag
    #define Y_MAG_INTEGRAL eyMag*kiMag
    #define Z_MAG_INTEGRAL ezMag*kiMag
    
    #define X_MAG_PROPORTIONAL exMag*kpMag
    #define Y_MAG_PROPORTIONAL eyMag*kpMag
    #define Z_MAG_PROPORTIONAL ezMag*kpMag
  #else
    #define X_MAG_INTEGRAL 0
    #define Y_MAG_INTEGRAL 0
    #define Z_MAG_INTEGRAL 0
    
    #define X_MAG_PROPORTIONAL 0
    #define Y_MAG_PROPORTIONAL 0
    #define Z_MAG_PROPORTIONAL 0  
  #endif
  
  // integral error scaled integral gain
  exInt = exInt + exAcc*kiAcc + X_MAG_INTEGRAL;
  eyInt = eyInt + eyAcc*kiAcc + Y_MAG_INTEGRAL;
  ezInt = ezInt + ezAcc*kiAcc + Z_MAG_INTEGRAL;
  	
  // adjusted gyroscope measurements
  gx = rollRate  + exInt + exAcc*kpAcc + X_MAG_PROPORTIONAL;
  gy = pitchRate + eyInt + eyAcc*kpAcc + Y_MAG_PROPORTIONAL;
  gz = yawRate   + ezInt + ezAcc*kpAcc + Z_MAG_PROPORTIONAL;

  // integrate quaternion rate and normalise
  q0i = (-q1*gx - q2*gy - q3*gz) * halfT;
  q1i = ( q0*gx + q2*gz - q3*gy) * halfT;
  q2i = ( q0*gy - q1*gz + q3*gx) * halfT;
  q3i = ( q0*gz + q1*gy - q2*gx) * halfT;
  q0 += q0i;
  q1 += q1i;
  q2 += q2i;
  q3 += q3i;

  // normalize quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  
  // auxillary variables to reduce number of repeated operations
  q0q0 = q0*q0;
  q0q1 = q0*q1;
  q0q2 = q0*q2;
  q0q3 = q0*q3;
  q1q1 = q1*q1;
  q1q2 = q1*q2;
  q1q3 = q1*q3;
  q2q2 = q2*q2;   
  q2q3 = q2*q3;
  q3q3 = q3*q3;

  #if defined(BMP085)
    dcm[0] = q0q0 + q1q1 - q2q2 - q3q3;
    dcm[1] = 2 * (q1q2 - q0q3);
    dcm[2] = 2 * (q0q2 + q1q3);
    dcm[3] = 2 * (q1q2 + q0q3);
    dcm[4] = q0q0 - q1q1 + q2q2 - q3q3;
    dcm[5] = 2 * (q2q3 - q0q1);
    dcm[6] = 2 * (q1q3 - q0q2);
    dcm[7] = 2 * (q0q1 + q2q3);
    dcm[8] = q0q0 - q1q1 - q2q2 + q3q3;
    
    angle.value[ROLL]  =  atan2(dcm[7], dcm[8]);
    angle.value[PITCH] =  -asin(dcm[6]);
    #if defined(HMC5843) | defined(HMC5883)
      angle.value[YAW]   =  atan2(dcm[3], dcm[0]);
    #else
      if (gz > radians(1.0) || gz < radians(-1.0))
        angle.value[YAW] += gz * halfT * 2;
    #endif
    
    matrixMultiply(3, 3, 1, earthAccel.value, dcm, accel.value);
    earthAccel.value[ZAXIS] = oneG - earthAccel.value[ZAXIS];
    
    integrator1 = integrator1 + earthAccel.value[ZAXIS] * halfT * 2;
    sum1 = integrator1 + vzEst;
    integrator2 = integrator2 + sum1 * halfT * 2;
    sum2 = integrator2 + hEst;
    sum3 = sum2 - pressureAltitude.value;
    hEst = sum2 + sum3 * kHest;
    vzEst = sum1 + sum3 * kVz;
  #else
    kinematicsAngle[ROLL]  =  atan2(2 * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3);
    kinematicsAngle[PITCH] =  -asin(2 * (q1q3 - q0q2));
    #if defined(HMC5843) | defined(HMC5883)
      kinematicsAngle[YAW]   =  atan2(2 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
    #else
      if (gz > radians(1.0) || gz < radians(-1.0))
        kinematicsAngle[YAW] += gz * halfT * 2;
    #endif
  #endif
}

////////////////////////////////////////////////////////////////////////////////
// MARG Initialization
////////////////////////////////////////////////////////////////////////////////

void initializeKinematics(float hdgX, float hdgY)
{
  initializeBaseKinematicsParam(hdgX, hdgY);
  
  #if defined(HMC5843) | defined(HMC5883)
    float hdg = atan2(-mag.value[YAXIS], mag.value[XAXIS]);
  #else
    float hdg = 0.0;
    kinematicsAngle[YAW] = 0.0;
  #endif
  
  q0 = cos(hdg/2.0);
  q1 = 0.0;
  q2 = 0.0;
  q3 = sin(hdg/2.0);
  
  // auxillary variables to reduce number of repeated operations, for 1st pass
  q0q0 = q0*q0;
  q0q1 = q0*q1;
  q0q2 = q0*q2;
  q0q3 = q0*q3;
  q1q1 = q1*q1;
  q1q2 = q1*q2;
  q1q3 = q1*q3;
  q2q2 = q2*q2;   
  q2q3 = q2*q3;
  q3q3 = q3*q3;

  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;

  kpAcc = 2.0;
  
  kpMag = 2.0;
  kiMag = 0.005;
}


void calibrateKinematics()
{
}

#endif