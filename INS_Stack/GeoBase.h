/*
 * File: lla2ecef.h
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 14-Jun-2023 18:12:47
 */

#ifndef LLA2ECEF_H
#define LLA2ECEF_H

/* Function Declarations */
extern void lla2ecef(const double *llaPos, double *ecefPos);
extern void lla2ned(const double *llaPos, const double *lla0, double *nedPos);
extern void ecef2enu(const double *ecefPos, const double *lla0, double *enuPos);
extern void ned2lla(const double *lla_m, const double *refloc, double *lla_d);
void ned2ecef(const double *ned, const double *ref, double *ecef);
void ecef2lla(const double *ecef, double *lla);
void earthrad(double lat, double *R_N, double *R_M);


float degrees_f(float rad);
float radians_f(float deg);
double degrees_d(double rad);
double radians_d(double deg);
#endif

/*
 * File trailer for lla2ecef.h
 *
 * [EOF]
 */
