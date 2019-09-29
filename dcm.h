#ifndef DCM_H_
#define DCM_H_

extern float q0, q1, q2, q3;          // quaternion of sensor frame relative to auxiliary frame
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void quaternionToYawPitchRoll( float *ypr);

void FreeIMUUpdate(float gx, float gy, float gz, float rawAx, float rawAy, float rawAz, float mx, float my, float mz);
#endif /* DCM_H_ */
