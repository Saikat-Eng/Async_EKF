/*
 * File: IMUBasicEKF.h
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 14-Jun-2023 18:12:47
 */

#ifndef IMUBASICEKF_H
#define IMUBASICEKF_H

/* Include Files */
#define earth_g 9.81f
/* Function Declarations */
extern void IMUBasicEKF_correctEqn(float *x, float *P, const float *h, const float *H, const float *z, const float *R, float *innov);
extern void IMUBasicEKF_repairQuaternion(float *x);
extern void GPSBasicEKF_correctEqn(float *x, float *P, const float *h,
                                   const float *z,const float *R, float *innov);

#endif

/*
 * File trailer for IMUBasicEKF.h
 *
 * [EOF]
 */
