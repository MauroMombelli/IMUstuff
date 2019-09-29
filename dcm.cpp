#include "Arduino.h"
#include "dcm.h"
#include <math.h>
//#define sampleFreq  25.0f     // sample frequency in Hz
#define twoKpDef  (2.0f * 1.0f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

float twoKp = twoKpDef;                      // 2 * proportional gain (Kp)
float twoKi = twoKiDef;                      // 2 * integral gain (Ki)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // quaternion of sensor frame relative to auxiliary frame
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

int valid(float x, float y, float z){
  if (x == 0.0f && y == 0.0f && z == 0.0f){
    return false;
  }
    
  return true;
}

float invSqrt(float x) {
  /*float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));*/

  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  //return 1/sqrt(x);
}

float KP_M = 2.0f;
float KP_A = 2.0f;  
float KI_M = 0;

int count = -1;
int countA = 0;
int countM = 0;
int countG = 0;
int sampleFreq = 100;
int sampleFreqA=1600;
int sampleFreqG=1600;
int sampleFreqM=70;
long start = millis();
void FreeIMUUpdate(float gx, float gy, float gz, float rawAx, float rawAy, float rawAz, float mx, float my, float mz){
  static long lastFreqUp = millis();
  /* DINAMIC FREQUENCY! */
  if (count == -1){ //just the first time!
    lastFreqUp = millis();
  }
  count ++;
  if (millis()-lastFreqUp>=1000){
    sampleFreq = count;
    sampleFreqA = countA;
    sampleFreqM = countM;
    sampleFreqG = countG;
    count=countG=countA=countM=0;
    lastFreqUp = millis();
  }
  /* END DINAMIC FREQUENCY! */
  
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  float qa, qb, qc;

  // Auxiliary variables to avoid repeated arithmetic
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  float halfvx=0, halfvy=0, halfvz=0;
  if( valid(rawAx, rawAy, rawAz) ) {
    countA++;
    
    // Normalise accelerometer measurement
    recipNorm = invSqrt(rawAx * rawAx + rawAy * rawAy + rawAz * rawAz);
    /*
    if ( Float.isNaN(recipNorm) ){
      println("acce nan a recipNorm " +recipNorm);
    }*/
    float ax = rawAx * recipNorm;
    float ay = rawAy * recipNorm;
    float az = rawAz * recipNorm;/*
    if ( Float.isNaN(ax) || Float.isNaN(ay) || Float.isNaN(az) ){
      println("acce nan a pre " +halfvx+ " "+halfvy+" "+halfvz);
    }
*/
    // Estimated direction of gravity
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;/*
    if ( Float.isNaN(halfvx) || Float.isNaN(halfvy) || Float.isNaN(halfvz) ){
      println("acce nan halfv " +halfvx+ " "+halfvy+" "+halfvz);
    }*/
    
    halfex += KP_A * (ay * halfvz - az * halfvy) * (1.0f / sampleFreqA);
    halfey += KP_A * (az * halfvx - ax * halfvz) * (1.0f / sampleFreqA);
    halfez += KP_A * (ax * halfvy - ay * halfvx) * (1.0f / sampleFreqA);
    /*
    if ( Float.isNaN(halfex) || Float.isNaN(halfey) || Float.isNaN(halfez) ){
      println("acce nan halfe " +ax+ " "+ay+" "+az+" "+sampleFreqA + " " + (ay * halfvz - az * halfvy) + " " + (1.0f / sampleFreqA));
    }else{
      //println("acce halfe " +ax+ " "+ay+" "+az+" "+halfex + " " + halfey + " " + halfez);
    }*/
  }
  
  countG++;
  // Integrate rate of change of quaternion
  gx *= (1.0f / sampleFreqG);   // pre-multiply common factors
  gy *= (1.0f / sampleFreqG);
  gz *= (1.0f / sampleFreqG);
  /*
  if (Float.isNaN(gx) || Float.isNaN(gy) || Float.isNaN(gz)){
    println("gyro1 nan");
  }*/
    
  /*
  // Compute and apply integral feedback if enabled  
  if(twoKi > 0.0f) {
    integralFBx += twoKi * halfex;  // integral error scaled by Ki
    integralFBy += twoKi * halfey;
    integralFBz += twoKi * halfez;
    gx += integralFBx;  // apply integral feedback
    gy += integralFBy;
    gz += integralFBz;
  }
  */
  /*
  if ( Float.isNaN(halfex) || Float.isNaN(halfey) || Float.isNaN(halfez) ){
    println("final nan halfe");
  }*/
  
  // Apply error feedback
  gx += halfex;
  gy += halfey;
  gz += halfez;
  /*
  if (Float.isNaN(gx) || Float.isNaN(gy) || Float.isNaN(gz)){
    println("gyro2 nan");
  }

  if (Float.isNaN(q0) || Float.isNaN(q1) || Float.isNaN(q2) || Float.isNaN(q3)){
    println("quat0 nan");
  }*/
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);/*
  if (Float.isNaN(q0) || Float.isNaN(q1) || Float.isNaN(q2) || Float.isNaN(q3)){
    println("quat1 nan");
  }*/

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;/*
  if (Float.isNaN(q0) || Float.isNaN(q1) || Float.isNaN(q2) || Float.isNaN(q3)){
    println("quat2 nan");
    exit();
  }*/
  
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void quaternionToYawPitchRoll(float *q, float *angles) {
  /*
  float gx, gy, gz; // estimated gravity direction
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
  ypr[1] = atan2(gx, sqrt(gy*gy + gz*gz));
  ypr[2] = atan2(gy, sqrt(gx*gx + gz*gz));
  */
  angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

void quaternionToYawPitchRoll( float *ypr) {
  float q[4] = {q0, -q1, -q2, -q3};
  quaternionToYawPitchRoll(q, ypr);
}
