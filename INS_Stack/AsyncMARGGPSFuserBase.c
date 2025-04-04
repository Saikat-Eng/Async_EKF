/*
 * File: AsyncMARGGPSFuserBase.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 14-Jun-2023 18:12:47
 */

/* Include Files */

#include <INS_Stack/PoseEstimator.h>
#include <INS_Stack/AsyncMARGGPSFuserBase.h>
#include <INS_Stack/IMUBasicEKF.h>
#include <INS_Stack/GeoBase.h>
#include <INS_Stack/matrix.h>


/* Function Declarations */
static void c_AsyncMARGGPSFuserBase_accelMe(const float *x, float *z);
static void c_AsyncMARGGPSFuserBase_magMeas(const float *x, float *z);
static void c_AsyncMARGGPSFuserBase_stateTr(const float *x, float *dfdx);
static void d_AsyncMARGGPSFuserBase_accelMe(const float *x, float *dhdx);
static void d_AsyncMARGGPSFuserBase_magMeas(const float *x, float *dhdx);
static void d_AsyncMARGGPSFuserBase_stateTr(float *x);
static void c_ContinuousEKFPredictor_eulerI(const float *x, const float *xdot, float *e, float dt);
static void b_ContinuousEKFPredictor_eulerI(const float *P, const float *dfdx, float *pStateCovariance, float dt);

/* Function Definitions */

void AsyncMARGGPSFuserBase_predict(c_fusion_internal_coder_AsyncMA *obj, const float dt)
{
    float *P;
    float xdot[28];
    float dfdx[784];
    float pDot[784];
    float b_dfdx[784];
    float d0;
    float d1;

    int i0;
    int i1;
    int i2;
    int dfdx_tmp;

    xdot[0]  = obj->pState[0];	xdot[1]  = obj->pState[1];	xdot[2]  = obj->pState[2];	xdot[3]  = obj->pState[3];
    xdot[4]  = obj->pState[4];	xdot[5]  = obj->pState[5];	xdot[6]  = obj->pState[6];	xdot[7]  = obj->pState[7];
    xdot[8]  = obj->pState[8];	xdot[9]  = obj->pState[9];	xdot[10] = obj->pState[10];	xdot[11] = obj->pState[11];
    xdot[12] = obj->pState[12];	xdot[13] = obj->pState[13];	xdot[14] = obj->pState[14];	xdot[15] = obj->pState[15];
    xdot[16] = obj->pState[16];	xdot[17] = obj->pState[17];	xdot[18] = obj->pState[18];	xdot[19] = obj->pState[19];
    xdot[20] = obj->pState[20];	xdot[21] = obj->pState[21];	xdot[22] = obj->pState[22];	xdot[23] = obj->pState[23];
    xdot[24] = obj->pState[24];	xdot[25] = obj->pState[25];	xdot[26] = obj->pState[26];	xdot[27] = obj->pState[27];

    //mat_copy(obj->pStateCovariance,P,784);
    P = obj->pStateCovariance;

    //state transition
    d_AsyncMARGGPSFuserBase_stateTr(xdot);
    //state transition jacobian
    c_AsyncMARGGPSFuserBase_stateTr(obj->pState, dfdx);

    for (i0 = 0; i0 < 28; i0++) {
        for (i1 = 0; i1 < 28; i1++) {
            d0 = 0.0f;
            d1 = 0.0f;
            for (dfdx_tmp = 0; dfdx_tmp < 28; dfdx_tmp++) {
                i2 = dfdx_tmp + 28 * i1;
                d0 += P[i0 + 28 * dfdx_tmp] * dfdx[i2];
                d1 += dfdx[dfdx_tmp + 28 * i0] * P[i2];
            }
            dfdx_tmp = i0 + 28 * i1;
            b_dfdx[dfdx_tmp] = d1;
            pDot[dfdx_tmp] = d0;
        }
    }

    //  for (i0 = 0; i0 < 784; i0++)
    //  {
    //    pDot[i0] = (pDot[i0] + b_dfdx[i0]) + obj->addProcNoise[i0];
    //  }

    for (i0 = 0; i0 < 28; i0++)
    {
        i1 = 0+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 28+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 56+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 84+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 112+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 140+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 168+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 196+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 224+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 252+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 280+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 308+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 336+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 364+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 392+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 420+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 448+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 476+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 504+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 532+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 560+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 588+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 616+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 644+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 672+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 700+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 728+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
        i1 = 756+i0;
        pDot[i1] = (pDot[i1] + b_dfdx[i1]) + obj->addProcNoise[i1];
    }

    //  for (i0 = 0; i0 < 28; i0++) {
    //    for (i1 = 0; i1 < 28; i1++) {
    //      dfdx_tmp = i1 + 28 * i0;
    //      b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 28 * i1]);
    //    }
    //  }
    // b_dfdx:pDot = 0.5 * (pDot + pDot.'); % ensure symmetry
    for (i0 = 0; i0 < 28; i0++)
    {
        dfdx_tmp = 0 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 0]);
        dfdx_tmp = 1 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 28]);
        dfdx_tmp = 2 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 56]);
        dfdx_tmp = 3 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 84]);
        dfdx_tmp = 4 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 112]);
        dfdx_tmp = 5 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 140]);
        dfdx_tmp = 6 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 168]);
        dfdx_tmp = 7 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 196]);
        dfdx_tmp = 8 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 224]);
        dfdx_tmp = 9 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 252]);
        dfdx_tmp = 10 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 280]);
        dfdx_tmp = 11 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 308]);
        dfdx_tmp = 12 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 336]);
        dfdx_tmp = 13 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 364]);
        dfdx_tmp = 14 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 392]);
        dfdx_tmp = 15 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 420]);
        dfdx_tmp = 16 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 448]);
        dfdx_tmp = 17 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 476]);
        dfdx_tmp = 18 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 504]);
        dfdx_tmp = 19 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 532]);
        dfdx_tmp = 20 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 560]);
        dfdx_tmp = 21 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 588]);
        dfdx_tmp = 22 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 616]);
        dfdx_tmp = 23 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 644]);
        dfdx_tmp = 24 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 672]);
        dfdx_tmp = 25 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 700]);
        dfdx_tmp = 26 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 728]);
        dfdx_tmp = 27 + 28 * i0;
        b_dfdx[dfdx_tmp] = 0.5f * (pDot[dfdx_tmp] + pDot[i0 + 756]);

    }


    c_ContinuousEKFPredictor_eulerI(obj->pState, xdot, obj->pState, dt);

    b_ContinuousEKFPredictor_eulerI(P, b_dfdx, obj->pStateCovariance, dt);

    IMUBasicEKF_repairQuaternion(obj->pState);

}


/*
 * Arguments    : const float x[28]
 *                float z[3]
 * Return Type  : void
 */
void c_AsyncMARGGPSFuserBase_accelMe(const float *x, float *z)
{  
    float q0q0  = x[0]*x[0];
    float q1q1  = x[1]*x[1];
    float q2q2  = x[2]*x[2];
    float q3q3  = x[3]*x[3];
    float q1q2 = x[1]*x[2];
    float q1q3 = x[1]*x[3];
    float q2q3 = x[2]*x[3];
    float q0q1 = x[0]*x[1];
    float q0q2 = x[0]*x[2];
    float q0q3 = x[0]*x[3];
    float an  = x[13];
    float ae  = x[14];
    float ad  = x[15];

    z[0] =  x[16] - (an - 0.0f)*(q0q0 + q1q1 - q2q2 - q3q3) + (ad - 9.81f)*(2*q0q2 - 2*q1q3) - (ae - 0.0f)*(2*q0q3 + 2*q1q2);
    z[1] =  x[17] - (ae - 0.0f)*(q0q0 - q1q1 + q2q2 - q3q3) - (ad - 9.81f)*(2*q0q1 + 2*q2q3) + (an - 0.0f)*(2*q0q3 - 2*q1q2);
    z[2] =  x[18] - (ad - 9.81f)*(q0q0 - q1q1 - q2q2 + q3q3) + (ae - 0.0f)*(2*q0q1 - 2*q2q3) - (an - 0.0f)*(2*q0q2 + 2*q1q3);

}


/*
 * Arguments    : const float x[28]
 *                float z[3]
 * Return Type  : void
 */
void c_AsyncMARGGPSFuserBase_magMeas(const float *x, float *z)
{
    float q0q0  = x[0]*x[0];
    float q1q1  = x[1]*x[1];
    float q2q2  = x[2]*x[2];
    float q3q3  = x[3]*x[3];
    float q1q2 = x[1]*x[2];
    float q1q3 = x[1]*x[3];
    float q2q3 = x[2]*x[3];
    float q0q1 = x[0]*x[1];
    float q0q2 = x[0]*x[2];
    float q0q3 = x[0]*x[3];
    float magNavX = x[22];
    float magNavY = x[23];
    float magNavZ = x[24];

    z[0] = x[25] + magNavX*(q0q0 + q1q1 - q2q2 - q3q3) - magNavZ*(2*q0q2 - 2*q1q3) + magNavY*(2*q0q3 + 2*q1q2);
    z[1] = x[26] + magNavY*(q0q0 - q1q1 + q2q2 - q3q3) + magNavZ*(2*q0q1 + 2*q2q3) - magNavX*(2*q0q3 - 2*q1q2);
    z[2] = x[27] + magNavZ*(q0q0 - q1q1 - q2q2 + q3q3) - magNavY*(2*q0q1 - 2*q2q3) + magNavX*(2*q0q2 + 2*q1q3);

}


/*
 * Arguments    : const float x[28]
 *                float dfdx[784]
 * Return Type  : void
 */
void c_AsyncMARGGPSFuserBase_stateTr(const float *x, float *dfdx)
{
    float q0   = x[0];
    float q1   = x[1];
    float q2   = x[2];
    float q3   = x[3];
    float wx   = x[4];
    float wy   = x[5];
    float wz   = x[6];

    int i;

    const char b_iv[28] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0};
    const char b_iv1[28] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0};
    const char iv2[28] =   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    const char iv3[28] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    const char iv4[28] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    const char iv5[28] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    dfdx[0]=0;dfdx[1]=-wx/2;dfdx[2]=-wy/2;dfdx[3]=-wz/2;dfdx[4]=-q1/2;dfdx[5]=-q2/2;dfdx[6]=-q3/2;dfdx[7]=0;dfdx[8]=0;dfdx[9]=0;dfdx[10]=0;dfdx[11]=0;dfdx[12]=0;dfdx[13]=0;dfdx[14]=0;dfdx[15]=0;dfdx[16]=0;dfdx[17]=0;dfdx[18]=0;dfdx[19]=0;dfdx[20]=0;dfdx[21]=0;dfdx[22]=0;dfdx[23]=0;dfdx[24]=0;dfdx[25]=0;dfdx[26]=0;dfdx[27]=0;dfdx[28]=wx/2;
    dfdx[29]=0;dfdx[30]= wz/2;dfdx[31]=-wy/2;dfdx[32]= q0/2;dfdx[33]=-q3/2;dfdx[34]= q2/2;dfdx[35]=0;dfdx[36]=0;dfdx[37]=0;dfdx[38]=0;dfdx[39]=0;dfdx[40]=0;dfdx[41]=0;dfdx[42]=0;dfdx[43]=0;dfdx[44]=0;dfdx[45]=0;dfdx[46]=0;dfdx[47]=0;dfdx[48]=0;dfdx[49]=0;dfdx[50]=0;dfdx[51]=0;dfdx[52]=0;dfdx[53]=0;dfdx[54]=0;dfdx[55]=0;dfdx[56]=wy/2;
    dfdx[57]=-wz/2;dfdx[58]=0;dfdx[59]= wx/2;dfdx[60]= q3/2;dfdx[61]= q0/2;dfdx[62]=-q1/2;dfdx[63]=0;dfdx[64]=0;dfdx[65]=0;dfdx[66]=0;dfdx[67]=0;dfdx[68]=0;dfdx[69]=0;dfdx[70]=0;dfdx[71]=0;dfdx[72]=0;dfdx[73]=0;dfdx[74]=0;dfdx[75]=0;dfdx[76]=0;dfdx[77]=0;dfdx[78]=0;dfdx[79]=0;dfdx[80]=0;dfdx[81]=0;dfdx[82]=0;dfdx[83]=0;dfdx[84]=wz/2;
    dfdx[85]= wy/2;dfdx[86]=-wx/2;dfdx[87]=0;dfdx[88]=-q2/2;dfdx[89]= q1/2;dfdx[90]= q0/2;dfdx[91]=0;dfdx[92]=0;dfdx[93]=0;dfdx[94]=0;dfdx[95]=0;dfdx[96]=0;dfdx[97]=0;dfdx[98]=0;dfdx[99]=0;dfdx[100]=0;dfdx[101]=0;dfdx[102]=0;dfdx[103]=0;dfdx[104]=0;dfdx[105]=0;dfdx[106]=0;dfdx[107]=0;dfdx[108]=0;dfdx[109]=0;dfdx[110]=0;dfdx[111]=0;
    
    for (i = 0; i < 28; i++)
    {
        dfdx[i + 112] = 0.0f;
        dfdx[i + 140] = 0.0f;
        dfdx[i + 168] = 0.0f;
        dfdx[i + 196] = b_iv[i];
        dfdx[i + 224] = b_iv1[i];
        dfdx[i + 252] = iv2[i];
        dfdx[i + 280] = iv3[i];
        dfdx[i + 308] = iv4[i];
        dfdx[i + 336] = iv5[i];
        dfdx[i + 364] = 0.0f;
        dfdx[i + 392] = 0.0f;
        dfdx[i + 420] = 0.0f;
        dfdx[i + 448] = 0.0f;
        dfdx[i + 476] = 0.0f;
        dfdx[i + 504] = 0.0f;
        dfdx[i + 532] = 0.0f;
        dfdx[i + 560] = 0.0f;
        dfdx[i + 588] = 0.0f;
        dfdx[i + 616] = 0.0f;
        dfdx[i + 644] = 0.0f;
        dfdx[i + 672] = 0.0f;
        dfdx[i + 700] = 0.0f;
        dfdx[i + 728] = 0.0f;
        dfdx[i + 756] = 0.0f;
    }
}

/*
 * Arguments    : const float x[28]
 *                float dhdx[84]
 * Return Type  : void
 */
void d_AsyncMARGGPSFuserBase_accelMe(const float *x, float *dfdx)
{
    float q0 = x[0];
    float q1 = x[1];
    float q2 = x[2];
    float q3 = x[3];
    float q0q0   = x[0]*x[0];
    float q1q1   = x[1]*x[1];
    float q2q2   = x[2]*x[2];
    float q3q3   = x[3]*x[3];
    float an  = x[13];
    float ae  = x[14];
    float ad  = x[15];

    dfdx[0] = 2*q2*(ad - 9.81f) - 2*q3*(ae - 0) - 2*q0*(an - 0);
    dfdx[1] = -2*q3*(ad - 9.81f) - 2*q2*(ae - 0) - 2*q1*(an - 0);
    dfdx[2] = 2*q0*(ad - 9.81f) - 2*q1*(ae - 0) + 2*q2*(an - 0);
    dfdx[3] =  2*q3*(an - 0) - 2*q0*(ae - 0) - 2*q1*(ad - 9.81f);
    dfdx[4] =  0;
    dfdx[5] =  0;
    dfdx[6] =  0;
    dfdx[7] =  0;
    dfdx[8] =  0;
    dfdx[9] =  0;
    dfdx[10] =  0;
    dfdx[11] =  0;
    dfdx[12] =  0;
    dfdx[13] =  -q0q0 - q1q1 + q2q2 + q3q3;
    dfdx[14] =  -2*q0*q3 - 2*q1*q2;
    dfdx[15] =  2*q0*q2 - 2*q1*q3;
    dfdx[16] =  1;
    dfdx[17] =  0;
    dfdx[18] =  0;
    dfdx[19] =  0;
    dfdx[20] =  0;
    dfdx[21] =  0;
    dfdx[22] =  0;
    dfdx[23] =  0;
    dfdx[24] =  0;
    dfdx[25] =  0;
    dfdx[26] =  0;
    dfdx[27] =  0;
    dfdx[28] = 2*q3*(an - 0) - 2*q0*(ae - 0) - 2*q1*(ad - 9.81f);
    dfdx[29] =    2*q1*(ae - 0) - 2*q0*(ad - 9.81f) - 2*q2*(an - 0);
    dfdx[30] =  - 2*q3*(ad - 9.81f) - 2*q2*(ae - 0) - 2*q1*(an - 0);
    dfdx[31] =    2*q3*(ae - 0) - 2*q2*(ad - 9.81f) + 2*q0*(an - 0);
    dfdx[32] =  0;
    dfdx[33] =  0;
    dfdx[34] =  0;
    dfdx[35] =  0;
    dfdx[36] =  0;
    dfdx[37] =  0;
    dfdx[38] =  0;
    dfdx[39] =  0;
    dfdx[40] =  0;
    dfdx[41] =  2*q0*q3 - 2*q1*q2;
    dfdx[42] =  - q0q0 + q1q1 - q2q2 + q3q3;
    dfdx[43] =  - 2*q0*q1 - 2*q2*q3;
    dfdx[44] =  0;
    dfdx[45] =  1;
    dfdx[46] =  0;
    dfdx[47] =  0;
    dfdx[48] =  0;
    dfdx[49] =  0;
    dfdx[50] =  0;
    dfdx[51] =  0;
    dfdx[52] =  0;
    dfdx[53] =  0;
    dfdx[54] =  0;
    dfdx[55] =  0;
    dfdx[56] =  2*q1*(ae - 0) - 2*q0*(ad - 9.81f) - 2*q2*(an - 0);
    dfdx[57] =  2*q1*(ad - 9.81f) + 2*q0*(ae - 0) - 2*q3*(an - 0);
    dfdx[58] =  2*q2*(ad - 9.81f) - 2*q3*(ae - 0) - 2*q0*(an - 0);
    dfdx[59] =  -2*q3*(ad - 9.81f) - 2*q2*(ae - 0) - 2*q1*(an - 0);
    dfdx[60] =  0;
    dfdx[61] =  0;
    dfdx[62] =  0;
    dfdx[63] =  0;
    dfdx[64] =  0;
    dfdx[65] =  0;
    dfdx[66] =  0;
    dfdx[67] =  0;
    dfdx[68] =  0;
    dfdx[69] = -2*q0*q2 - 2*q1*q3;
    dfdx[70] = 2*q0*q1 - 2*q2*q3;
    dfdx[71] = -q0q0 + q1q1 + q2q2 - q3q3;
    dfdx[72] =  0;
    dfdx[73] =  0;
    dfdx[74] =  1;
    dfdx[75] =  0;
    dfdx[76] =  0;
    dfdx[77] =  0;
    dfdx[78] =  0;
    dfdx[79] =  0;
    dfdx[80] =  0;
    dfdx[81] =  0;
    dfdx[82] =  0;
    dfdx[83] =  0;


}

/*
 * Arguments    : const float x[28]
 *                float dhdx[84]
 * Return Type  : void
 */
void d_AsyncMARGGPSFuserBase_magMeas(const float *x, float *dhdx)
{  
    float q0 = x[0];
    float q1 = x[1];
    float q2 = x[2];
    float q3 = x[3];
    float q0q0   = x[0]*x[0];
    float q1q1   = x[1]*x[1];
    float q2q2   = x[2]*x[2];
    float q3q3   = x[3]*x[3];
    float magNavX = x[22];
    float magNavY = x[23];
    float magNavZ = x[24];

    dhdx[0] = 2*magNavY*q3 - 2*magNavZ*q2 + 2*magNavX*q0;
    dhdx[1] = 2*magNavZ*q3 + 2*magNavY*q2 + 2*magNavX*q1;
    dhdx[2] = 2*magNavY*q1 - 2*magNavZ*q0 - 2*magNavX*q2;
    dhdx[3] = 2*magNavZ*q1 + 2*magNavY*q0 - 2*magNavX*q3;
    dhdx[4] = 0;
    dhdx[5] = 0;
    dhdx[6] = 0;
    dhdx[7] = 0;
    dhdx[8] = 0;
    dhdx[9] = 0;
    dhdx[10] = 0;
    dhdx[11] = 0;
    dhdx[12] = 0;
    dhdx[13] = 0;
    dhdx[14] = 0;
    dhdx[15] = 0;
    dhdx[16] = 0;
    dhdx[17] = 0;
    dhdx[18] = 0;
    dhdx[19] = 0;
    dhdx[20] = 0;
    dhdx[21] = 0;
    dhdx[22] = q0q0 + q1q1 - q2q2 - q3q3;
    dhdx[23] = 2*q0*q3 + 2*q1*q2;
    dhdx[24] = 2*q1*q3 - 2*q0*q2;
    dhdx[25] = 1;
    dhdx[26] = 0;
    dhdx[27] = 0;
    dhdx[28] = 2*magNavZ*q1 + 2*magNavY*q0 - 2*magNavX*q3;
    dhdx[29] = 2*magNavZ*q0 - 2*magNavY*q1 + 2*magNavX*q2;
    dhdx[30] = 2*magNavZ*q3 + 2*magNavY*q2 + 2*magNavX*q1;
    dhdx[31] = 2*magNavZ*q2 - 2*magNavY*q3 - 2*magNavX*q0;
    dhdx[32] = 0;
    dhdx[33] = 0;
    dhdx[34] = 0;
    dhdx[35] = 0;
    dhdx[36] = 0;
    dhdx[37] = 0;
    dhdx[38] = 0;
    dhdx[39] = 0;
    dhdx[40] = 0;
    dhdx[41] = 0;
    dhdx[42] = 0;
    dhdx[43] = 0;
    dhdx[44] = 0;
    dhdx[45] = 0;
    dhdx[46] = 0;
    dhdx[47] = 0;
    dhdx[48] = 0;
    dhdx[49] = 0;
    dhdx[50] = 2*q1*q2 - 2*q0*q3;
    dhdx[51] = q0q0 - q1q1 + q2q2 - q3q3;
    dhdx[52] = 2*q0*q1 + 2*q2*q3;
    dhdx[53] = 0;
    dhdx[54] = 1;
    dhdx[55] = 0;
    dhdx[56] = 2*magNavZ*q0 - 2*magNavY*q1 + 2*magNavX*q2;
    dhdx[57] = 2*magNavX*q3 - 2*magNavY*q0 - 2*magNavZ*q1;
    dhdx[58] = 2*magNavY*q3 - 2*magNavZ*q2 + 2*magNavX*q0;
    dhdx[59] = 2*magNavZ*q3 + 2*magNavY*q2 + 2*magNavX*q1;
    dhdx[60] = 0;
    dhdx[61] = 0;
    dhdx[62] = 0;
    dhdx[63] = 0;
    dhdx[64] = 0;
    dhdx[65] = 0;
    dhdx[66] = 0;
    dhdx[67] = 0;
    dhdx[68] = 0;
    dhdx[69] = 0;
    dhdx[70] = 0;
    dhdx[71] = 0;
    dhdx[72] = 0;
    dhdx[73] = 0;
    dhdx[74] = 0;
    dhdx[75] = 0;
    dhdx[76] = 0;
    dhdx[77] = 0;
    dhdx[78] = 2*q0*q2 + 2*q1*q3;
    dhdx[79] = 2*q2*q3 - 2*q0*q1;
    dhdx[80] = q0q0 - q1q1 - q2q2 + q3q3;
    dhdx[81] = 0;
    dhdx[82] = 0;
    dhdx[83] = 1;
}


/*
 * Arguments    : float x[28]
 * Return Type  : void
 */
void d_AsyncMARGGPSFuserBase_stateTr(float *x)
{
    float q0   = x[0];
    float q1   = x[1];
    float q2   = x[2];
    float q3   = x[3];
    float wx   = x[4];
    float wy   = x[5];
    float wz   = x[6];

    x[0] = -(q1*wx)/2 - (q2*wy)/2 - (q3*wz)/2;
    x[1] = (q0*wx)/2 - (q3*wy)/2 + (q2*wz)/2;
    x[2] = (q3*wx)/2 + (q0*wy)/2 - (q1*wz)/2;
    x[3] = (q1*wy)/2 - (q2*wx)/2 + (q0*wz)/2;
    x[4] = 0.0f;
    x[5] = 0.0f;
    x[6] = 0.0f;
    x[7] = x[10];
    x[8] = x[11];
    x[9] = x[12];
    x[10] = x[13];
    x[11] = x[14];
    x[12] = x[15];
    x[13] = 0;
    x[14] = 0;
    x[15] = 0;
    x[16] = 0;
    x[17] = 0;
    x[18] = 0;
    x[19] = 0;
    x[20] = 0;
    x[21] = 0;
    x[22] = 0;
    x[23] = 0;
    x[24] = 0;
    x[25] = 0;
    x[26] = 0;
    x[27] = 0;
}

/*
 * Arguments    : c_fusion_internal_coder_AsyncMA *obj
 *                const float accel[3]
 *                float Raccel
 * Return Type  : void
 */
void AsyncMARGGPSFuserBase_fuseaccel(c_fusion_internal_coder_AsyncMA *obj, const float *accel)
{
    float dv0[3];
    float dv1[84];
    //means
    c_AsyncMARGGPSFuserBase_accelMe(obj->pState, dv0);
    //jacobian
    d_AsyncMARGGPSFuserBase_accelMe(obj->pState, dv1);

    IMUBasicEKF_correctEqn(obj->pState, obj->pStateCovariance, dv0, dv1, accel, ekf_cfg_instance()->Racc, obj->accl_innov);

}

/*
 * Arguments    : c_fusion_internal_coder_AsyncMA *obj
 *                const float lla[3]
 *                float Rpos
 *                const float vel[3]
 *                float Rvel
 * Return Type  : void
 */
void AsyncMARGGPSFuserBase_fusegps(c_fusion_internal_coder_AsyncMA *obj, const float *lla, const float *vel)
{
    float dv1[6];
    double dv0[3];
    float h[6];
    double lla_f[3] = {(double)lla[0],(double)lla[1],(double)lla[2]};

    lla2ned(lla_f,obj->ReferenceLocation,dv0);

    dv1[0] = (float)dv0[0];
    dv1[1] = (float)dv0[1];
    dv1[2] = (float)dv0[2];
    dv1[3] = vel[0];
    dv1[4] = vel[1];
    dv1[5] = vel[2];

    h[0] = obj->pState[7];
    h[1] = obj->pState[8];
    h[2] = obj->pState[9];
    h[3] = obj->pState[10];
    h[4] = obj->pState[11];
    h[5] = obj->pState[12];

    GPSBasicEKF_correctEqn(obj->pState, obj->pStateCovariance, h, dv1, ekf_cfg_instance()->Rgps, obj->gps_innov);
}

/*
 * Arguments    : c_fusion_internal_coder_AsyncMA *obj
 *                const float gyro[3]
 *                float Rgyro
 * Return Type  : void
 */
void AsyncMARGGPSFuserBase_fusegyro(c_fusion_internal_coder_AsyncMA *obj, const float *gyro)
{
    float dv0[3];

    static const float dv1[84] = {
        0.0f, 0.0f, 0.0f, 0.0f, 1.0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    dv0[0] = obj->pState[19] + obj->pState[4];
    dv0[1] = obj->pState[20] + obj->pState[5];
    dv0[2] = obj->pState[21] + obj->pState[6];

    IMUBasicEKF_correctEqn(obj->pState, obj->pStateCovariance, dv0, dv1, gyro, ekf_cfg_instance()->Rgyro, obj->gyro_innov);

}

/*
 * Arguments    : c_fusion_internal_coder_AsyncMA *obj
 *                const float mag[3]
 *                float Rmag
 * Return Type  : void
 */
void AsyncMARGGPSFuserBase_fusemag(c_fusion_internal_coder_AsyncMA *obj, const float *mag)
{
    float dv0[3];
    float dv1[84];

    //means
    c_AsyncMARGGPSFuserBase_magMeas(obj->pState, dv0);
    //jakobian
    d_AsyncMARGGPSFuserBase_magMeas(obj->pState, dv1);

    IMUBasicEKF_correctEqn(obj->pState, obj->pStateCovariance, dv0, dv1, mag, ekf_cfg_instance()->Rmag, obj->mag_innov);

}

void c_ContinuousEKFPredictor_eulerI(const float *x, const float *xdot, float *e, float dt)
{
    e[0]  = x[0]  + dt * xdot[0] ;
    e[1]  = x[1]  + dt * xdot[1] ;
    e[2]  = x[2]  + dt * xdot[2] ;
    e[3]  = x[3]  + dt * xdot[3] ;
    e[4]  = x[4]  + dt * xdot[4] ;
    e[5]  = x[5]  + dt * xdot[5] ;
    e[6]  = x[6]  + dt * xdot[6] ;
    e[7]  = x[7]  + dt * xdot[7] ;
    e[8]  = x[8]  + dt * xdot[8] ;
    e[9]  = x[9]  + dt * xdot[9] ;
    e[10] = x[10] + dt * xdot[10];
    e[11] = x[11] + dt * xdot[11];
    e[12] = x[12] + dt * xdot[12];
    e[13] = x[13] + dt * xdot[13];
    e[14] = x[14] + dt * xdot[14];
    e[15] = x[15] + dt * xdot[15];
    e[16] = x[16] + dt * xdot[16];
    e[17] = x[17] + dt * xdot[17];
    e[18] = x[18] + dt * xdot[18];
    e[19] = x[19] + dt * xdot[19];
    e[20] = x[20] + dt * xdot[20];
    e[21] = x[21] + dt * xdot[21];
    e[22] = x[22] + dt * xdot[22];
    e[23] = x[23] + dt * xdot[23];
    e[24] = x[24] + dt * xdot[24];
    e[25] = x[25] + dt * xdot[25];
    e[26] = x[26] + dt * xdot[26];
    e[27] = x[27] + dt * xdot[27];
}


void b_ContinuousEKFPredictor_eulerI(const float *P,const float *dfdx, float *pStateCovariance, float dt)
{
    register int i0;
    register int i1;

    for (i0 = 0; i0 < 28; i0++)
    {
        i1 = 0+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 28+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 56+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 84+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 112+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 140+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 168+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 196+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 224+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 252+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 280+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 308+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 336+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 364+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 392+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 420+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 448+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 476+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 504+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 532+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 560+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 588+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 616+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 644+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 672+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 700+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 728+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
        i1 = 756+i0;
        pStateCovariance[i1] = P[i1] + dt * dfdx[i1];
    }
}
/*
 * File trailer for AsyncMARGGPSFuserBase.c
 *
 * [EOF]
 */
