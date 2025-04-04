/*
 * File: lla2ecef.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 14-Jun-2023 18:12:47
 */

/* Include Files */
#include <math.h>
#include <INS_Stack/PoseEstimator.h>
#include <INS_Stack/GeoBase.h>
#include <INS_Stack/IMUBasicEKF.h>
#include <util.h>

void lla2ned(const double *llaPos, const double *lla0, double *nedPos)
{
    double enuPos[3];
    double ecef[3];

    lla2ecef(llaPos, ecef);
    ecef2enu(ecef, lla0, enuPos);

    nedPos[0] = enuPos[1];
    nedPos[1] = enuPos[0];
    nedPos[2] = -enuPos[2];
}


/* Function Definitions */

/*
 * Arguments    : const float llaPos[3]
 *                float ecefPos[3]
 * Return Type  : void
 */
void lla2ecef(const double *llaPos, double *ecefPos)
{
    double Rew,Rns;

    double lat = radians_d(llaPos[0]);
    double lon = radians_d(llaPos[1]);
    double alt = llaPos[2];

    earthrad(lat,&Rew,&Rns);

    ecefPos[0] = (Rew + alt)*cos(lat)*cos(lon);
    ecefPos[1] = (Rew + alt)*cos(lat)*sin(lon);
    ecefPos[2] = ( (1-6.69437999014e-3)*Rew + alt )*sin(lat);
}

void ecef2enu(const double *ecefPos, const double *lla0, double *enuPos)
{
    double ecefPosWithENUOrigin[3];
    double coslambda;
    double cosphi;
    double sinlambda;
    double sinphi;
    double tmp;
    cosphi = radians_d(lla0[0]);
    cosphi = cos(cosphi);
    sinphi = radians_d(lla0[0]);
    sinphi = sin(sinphi);
    coslambda = radians_d(lla0[1]);
    coslambda = cos(coslambda);
    sinlambda = radians_d(lla0[1]);
    sinlambda = sin(sinlambda);
    lla2ecef(lla0, ecefPosWithENUOrigin);
    ecefPosWithENUOrigin[0] = ecefPos[0] - ecefPosWithENUOrigin[0];
    ecefPosWithENUOrigin[1] = ecefPos[1] - ecefPosWithENUOrigin[1];
    ecefPosWithENUOrigin[2] = ecefPos[2] - ecefPosWithENUOrigin[2];
    tmp =
            coslambda * ecefPosWithENUOrigin[0] + sinlambda * ecefPosWithENUOrigin[1];
    enuPos[0] = -sinlambda * ecefPosWithENUOrigin[0] +
            coslambda * ecefPosWithENUOrigin[1];
    enuPos[1] = -sinphi * tmp + cosphi * ecefPosWithENUOrigin[2];
    enuPos[2] = cosphi * tmp + sinphi * ecefPosWithENUOrigin[2];
}

void ned2ecef(const double *ned, const double *ref, double *ecef)
{
    double C[9] = {};
    double lat_ref = radians_d(ref[0]);
    double lon_ref = radians_d(ref[1]);

    C[0] = -sin(lat_ref)*cos(lon_ref);
    C[3] = -sin(lat_ref)*sin(lon_ref);
    C[6] = cos(lat_ref);
    C[1] = -sin(lon_ref);
    C[4] = cos(lon_ref);
    C[7] = 0;
    C[2] = -cos(lat_ref)*cos(lon_ref);
    C[5] = -cos(lat_ref)*sin(lon_ref);
    C[8] = -sin(lat_ref);

    ecef[0] = (C[0] * ned[0] + C[1] * ned[1]) + C[2] * ned[2];
    ecef[1] = (C[3] * ned[0] + C[4] * ned[1]) + C[5] * ned[2];
    ecef[2] = (C[6] * ned[0] + C[7] * ned[1]) + C[8] * ned[2];
}

/*
Return Earth R-major and R-minor in wgs84
*/
void earthrad(double lat_rad, double *R_N, double *R_M)
{
    double sin_lat = sin(lat_rad) * sin(lat_rad);
    double lamda = 1-0.00669437999014*sin_lat;
    *R_N = 6378137.0/pow(lamda,0.5);
    *R_M = 6378137.0*(1-0.00669437999014)/pow(lamda,1.5);
}

void ecef2lla(const double *ecef, double *lla)
{
    //longitude
    double x = ecef[0];
    double y = ecef[1];
    double z = ecef[2];
    double Rn,Rm;

    lla[1] = atan2(y,x);
    double p = sqrt(x*x+y*y);
    lla[0] = atan2(z,p*(1-6.69437999014e-3));
    double h=0;
    double err = 1;

    while(fabs(err)>1e-10)
    {
        earthrad(lla[0],&Rn,&Rm);

        h = p/cos(lla[0]) - Rn;
        err = atan2(z*(1+6.69437999014e-3*Rn*sin(lla[0])/z),p) - lla[0];
        
        lla[0] = lla[0] + err;
    }
    lla[0] = degrees_d(lla[0]);
    lla[1] = degrees_d(lla[1]);
    lla[2] = h;
}

void ned2lla(const double *lla_m, const double *refloc, double *lla_d)
{
    double ecef[3],ecef_ref[3];
    ned2ecef(lla_m,refloc,ecef);

    lla2ecef(refloc,ecef_ref);

    ecef[0] += ecef_ref[0];
    ecef[1] += ecef_ref[1];
    ecef[2] += ecef_ref[2];


    ecef2lla(ecef,lla_d);
}

/*
 * File trailer for lla2ecef.c
 *
 * [EOF]
 */
