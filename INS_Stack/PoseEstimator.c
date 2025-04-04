/*
 * File: run_PoseEstimationFromAsynchronousSensorsExample.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 14-Jun-2023 18:12:47
 */

/* Include Files */
#include <string.h>
#include <stdio.h>
#include <INS_Stack/PoseEstimator.h>
#include <INS_Stack/AsyncMARGGPSFuserBase.h>
#include <INS_Stack/IMUBasicEKF.h>
#include <INS_Stack/matrix.h>
#include <INS_Stack/GeoBase.h>

c_fusion_internal_coder_AsyncMA fusionfilt;
c_fusion_internal_coder_AsyncMA_Cfg fusionfilt_cfg;

c_fusion_internal_coder_AsyncMA* get_ekf_instance(void)
{
    return &fusionfilt;
}

c_fusion_internal_coder_AsyncMA_Cfg* ekf_cfg_instance(void)
{
    return &fusionfilt_cfg;
}


void ekf_init(double *ref_loc, float *mag_field, float *q_init)
{
    memset(&fusionfilt,0,sizeof(fusionfilt));
    memset(&fusionfilt_cfg,0,sizeof(fusionfilt_cfg));

    fusionfilt.ReferenceLocation[0] = ref_loc[0];
    fusionfilt.ReferenceLocation[1] = ref_loc[1];
    fusionfilt.ReferenceLocation[2] = ref_loc[2];

    IMUBasicEKF_repairQuaternion(q_init);

    fusionfilt.pState[0]  = q_init[0];
    fusionfilt.pState[1]  = q_init[1];
    fusionfilt.pState[2]  = q_init[2];
    fusionfilt.pState[3]  = q_init[3];
    fusionfilt.pState[4]  = 0.0f;
    fusionfilt.pState[5]  = 0.0f;
    fusionfilt.pState[6]  = 0.0f;
    fusionfilt.pState[7]  = 0.0f;
    fusionfilt.pState[8]  = 0.0f;
    fusionfilt.pState[9]  = 0.0f;
    fusionfilt.pState[10] = 0.0f;
    fusionfilt.pState[11] = 0.0f;
    fusionfilt.pState[12] = 0.0f;
    fusionfilt.pState[13] = 0.0f;
    fusionfilt.pState[14] = 0.0f;
    fusionfilt.pState[15] = 0.0f;
    fusionfilt.pState[16] = 0.0f;
    fusionfilt.pState[17] = 0.0f;
    fusionfilt.pState[18] = 0.0f;
    fusionfilt.pState[19] = radians_f(1.125f);
    fusionfilt.pState[20] = radians_f(1.125f);
    fusionfilt.pState[21] = radians_f(1.125f);
    fusionfilt.pState[22] = mag_field[0];
    fusionfilt.pState[23] = mag_field[1];
    fusionfilt.pState[24] = mag_field[2];
    fusionfilt.pState[25] = 0.0f;
    fusionfilt.pState[26] = 0.0f;
    fusionfilt.pState[27] = 0.0f;

    /*
>Increase in Sensor Bias Noise:

Reduced Accuracy: The state estimates become less accurate because the filter has to account for higher uncertainty in the measurements.
Higher Innovation: The difference between the predicted and actual measurements (innovation) will generally increase,
leading to larger corrections in the state estimates.
Increased Covariance: The covariance of the state estimate increases, indicating higher uncertainty in the estimated states.

>Decrease in Sensor Bias Noise:

Improved Accuracy: The state estimates become more accurate as the filter can rely more on the measurements.
Lower Innovation: The innovation decreases, leading to smaller corrections in the state estimates.
Decreased Covariance: The covariance of the state estimate decreases, indicating lower uncertainty in the estimated states.

Process Noise: This represents the uncertainty in the system model.
If the process noise covariance is too low, the filter may not account for all the dynamics of the system,
leading to drift over time. Increasing the process noise covariance can help the filter adapt better to changes and reduce drift.

Measurement Noise: This represents the uncertainty in the sensor measurements.
If the measurement noise covariance is not accurately set, the filter might give too much weight to noisy measurements,
causing drift. Ensuring that the measurement noise covariance accurately reflects the sensor's noise characteristics is crucial.
*/
    float rmag = sqrtf(mag_field[0]*mag_field[0]+mag_field[1]*mag_field[1]+mag_field[2]*mag_field[2]);

    fusionfilt_cfg.Rmag[0] = rmag; fusionfilt_cfg.Rmag[1] = 0.0f; fusionfilt_cfg.Rmag[2] = 0.0f;
    fusionfilt_cfg.Rmag[3] = 0.0f; fusionfilt_cfg.Rmag[4] = rmag; fusionfilt_cfg.Rmag[5] = 0.0f;
    fusionfilt_cfg.Rmag[6] = 0.0f; fusionfilt_cfg.Rmag[7] = 0.0f; fusionfilt_cfg.Rmag[8] = rmag;
    
    fusionfilt_cfg.Racc[0] = 0.0061f;fusionfilt_cfg.Racc[1] = 0.0f;  fusionfilt_cfg.Racc[2] = 0.0f;
    fusionfilt_cfg.Racc[3] = 0.0f;   fusionfilt_cfg.Racc[4] = 0.0061f;fusionfilt_cfg.Racc[5] = 0.0f;
    fusionfilt_cfg.Racc[6] = 0.0f;   fusionfilt_cfg.Racc[7] = 0.0f;  fusionfilt_cfg.Racc[8] = 0.0061f;

    fusionfilt_cfg.Rgyro[0] = 3.0462e-6f;fusionfilt_cfg.Rgyro[1] = 0.0f;   fusionfilt_cfg.Rgyro[2] = 0.0f;
    fusionfilt_cfg.Rgyro[3] = 0.0f;      fusionfilt_cfg.Rgyro[4] = 3.0462e-6f;fusionfilt_cfg.Rgyro[5] = 0.0f;
    fusionfilt_cfg.Rgyro[6] = 0.0f;      fusionfilt_cfg.Rgyro[7] = 0.0f;   fusionfilt_cfg.Rgyro[8] = 3.0462e-6f;

    float r_vel = powf(0.05f,2);

    fusionfilt_cfg.Rgps[0]=3.4f;   fusionfilt_cfg.Rgps[6]=0.0f;   fusionfilt_cfg.Rgps[12]=0.0f;
    fusionfilt_cfg.Rgps[1]=0.0f;   fusionfilt_cfg.Rgps[7]=3.4f;   fusionfilt_cfg.Rgps[13]=0.0f;
    fusionfilt_cfg.Rgps[2]=0.0f;   fusionfilt_cfg.Rgps[8]=0.0f;   fusionfilt_cfg.Rgps[14]=15.4f;
    fusionfilt_cfg.Rgps[3]=0.0f;   fusionfilt_cfg.Rgps[9]=0.0f;   fusionfilt_cfg.Rgps[15]=0.0f;
    fusionfilt_cfg.Rgps[4]=0.0f;   fusionfilt_cfg.Rgps[10]=0.0f;  fusionfilt_cfg.Rgps[16]=0.0f;
    fusionfilt_cfg.Rgps[5]=0.0f;   fusionfilt_cfg.Rgps[11]=0.0f;  fusionfilt_cfg.Rgps[17]=0.0f;
    fusionfilt_cfg.Rgps[18]=0.0f;  fusionfilt_cfg.Rgps[24]=0.0f;  fusionfilt_cfg.Rgps[30]=0.0f;
    fusionfilt_cfg.Rgps[19]=0.0f;  fusionfilt_cfg.Rgps[25]=0.0f;  fusionfilt_cfg.Rgps[31]=0.0f;
    fusionfilt_cfg.Rgps[20]=0.0f;  fusionfilt_cfg.Rgps[26]=0.0f;  fusionfilt_cfg.Rgps[32]=0.0f;
    fusionfilt_cfg.Rgps[21]=r_vel; fusionfilt_cfg.Rgps[27]=0.0f;  fusionfilt_cfg.Rgps[33]=0.0f;
    fusionfilt_cfg.Rgps[22]=0.0f;  fusionfilt_cfg.Rgps[28]=r_vel; fusionfilt_cfg.Rgps[34]=0.0f;
    fusionfilt_cfg.Rgps[23]=0.0f;  fusionfilt_cfg.Rgps[29]=0.0f;  fusionfilt_cfg.Rgps[35]=r_vel;

    //Process Noise
    fusionfilt_cfg.PositionNoise[0] = 1.0e-4f;
    fusionfilt_cfg.PositionNoise[1] = 1.0e-4f;
    fusionfilt_cfg.PositionNoise[2] = 1.0e-4f;

    fusionfilt_cfg.VelocityNoise[0] = 1.0e-6f;
    fusionfilt_cfg.VelocityNoise[1] = 1.0e-6f;
    fusionfilt_cfg.VelocityNoise[2] = 1.0e-6f;

    fusionfilt_cfg.QuaternionNoise[0] = 1.0e-6f;
    fusionfilt_cfg.QuaternionNoise[1] = 1.0e-6f;
    fusionfilt_cfg.QuaternionNoise[2] = 1.0e-6f;
    fusionfilt_cfg.QuaternionNoise[3] = 1.0e-6f;

    fusionfilt_cfg.AccelerationNoise[0] = 50.0f;
    fusionfilt_cfg.AccelerationNoise[1] = 50.0f;
    fusionfilt_cfg.AccelerationNoise[2] = 50.0f;

    fusionfilt_cfg.AngularVelocityNoise[0] = 50.85f;
    fusionfilt_cfg.AngularVelocityNoise[1] = 50.85f;
    fusionfilt_cfg.AngularVelocityNoise[2] = 50.85f;

    //Increasing it makes earth magnetic field bias estimation faster and noisier.
    fusionfilt_cfg.GeomagneticVectorNoise[0] = 1.5e-6f;
    fusionfilt_cfg.GeomagneticVectorNoise[1] = 1.5e-6f;
    fusionfilt_cfg.GeomagneticVectorNoise[2] = 1.5e-6f;

    //Increasing it makes compass offset estimation faster and noisier..
    fusionfilt_cfg.MagnetometerBiasNoise[0] = 4.5e-2f;
    fusionfilt_cfg.MagnetometerBiasNoise[1] = 4.5e-2f;
    fusionfilt_cfg.MagnetometerBiasNoise[2] = 4.5e-2f;

    //Increasing it makes accelerometer bias estimation faster and noisier.
    fusionfilt_cfg.AccelerometerBiasNoise[0] = 1.5e-14f;
    fusionfilt_cfg.AccelerometerBiasNoise[1] = 1.5e-14f;
    fusionfilt_cfg.AccelerometerBiasNoise[2] = 1.5e-14f;

    //Increasing it makes rate gyro bias estimation faster and noisier.
    fusionfilt_cfg.GyroscopeBiasNoise[0] = 1.5e-14f;
    fusionfilt_cfg.GyroscopeBiasNoise[1] = 1.5e-14f;
    fusionfilt_cfg.GyroscopeBiasNoise[2] = 1.5e-14f;

    int i=0;

    for(i=0;i<28;i++)
        fusionfilt.pStateCovariance[i*28+i] = 1.0e-4f;

    fusionfilt.addProcNoise[0]   = fusionfilt_cfg.QuaternionNoise[0];
    fusionfilt.addProcNoise[29]  = fusionfilt_cfg.QuaternionNoise[1];
    fusionfilt.addProcNoise[58]  = fusionfilt_cfg.QuaternionNoise[2];
    fusionfilt.addProcNoise[87]  = fusionfilt_cfg.QuaternionNoise[3];
    fusionfilt.addProcNoise[116] = fusionfilt_cfg.AngularVelocityNoise[0];
    fusionfilt.addProcNoise[145] = fusionfilt_cfg.AngularVelocityNoise[1];
    fusionfilt.addProcNoise[174] = fusionfilt_cfg.AngularVelocityNoise[2];
    fusionfilt.addProcNoise[203] = fusionfilt_cfg.PositionNoise[0];
    fusionfilt.addProcNoise[232] = fusionfilt_cfg.PositionNoise[1];
    fusionfilt.addProcNoise[261] = fusionfilt_cfg.PositionNoise[2];
    fusionfilt.addProcNoise[290] = fusionfilt_cfg.VelocityNoise[0];
    fusionfilt.addProcNoise[319] = fusionfilt_cfg.VelocityNoise[1];
    fusionfilt.addProcNoise[348] = fusionfilt_cfg.VelocityNoise[2];
    fusionfilt.addProcNoise[377] = fusionfilt_cfg.AccelerationNoise[0];
    fusionfilt.addProcNoise[406] = fusionfilt_cfg.AccelerationNoise[1];
    fusionfilt.addProcNoise[435] = fusionfilt_cfg.AccelerationNoise[2];
    fusionfilt.addProcNoise[464] = fusionfilt_cfg.AccelerometerBiasNoise[0];
    fusionfilt.addProcNoise[493] = fusionfilt_cfg.AccelerometerBiasNoise[1];
    fusionfilt.addProcNoise[522] = fusionfilt_cfg.AccelerometerBiasNoise[2];
    fusionfilt.addProcNoise[551] = fusionfilt_cfg.GyroscopeBiasNoise[0];
    fusionfilt.addProcNoise[580] = fusionfilt_cfg.GyroscopeBiasNoise[1];
    fusionfilt.addProcNoise[609] = fusionfilt_cfg.GyroscopeBiasNoise[2];
    fusionfilt.addProcNoise[638] = fusionfilt_cfg.GeomagneticVectorNoise[0];
    fusionfilt.addProcNoise[667] = fusionfilt_cfg.GeomagneticVectorNoise[1];
    fusionfilt.addProcNoise[696] = fusionfilt_cfg.GeomagneticVectorNoise[2];
    fusionfilt.addProcNoise[725] = fusionfilt_cfg.MagnetometerBiasNoise[0];
    fusionfilt.addProcNoise[754] = fusionfilt_cfg.MagnetometerBiasNoise[1];
    fusionfilt.addProcNoise[783] = fusionfilt_cfg.MagnetometerBiasNoise[2];
}

float *get_state(void)
{
    return fusionfilt.pState;
}

void set_ref_loc(double *ref_loc)
{
    fusionfilt.ReferenceLocation[0] = ref_loc[0];
    fusionfilt.ReferenceLocation[1] = ref_loc[1];
    fusionfilt.ReferenceLocation[2] = ref_loc[2];
}

/*
 * File trailer for run_PoseEstimationFromAsynchronousSensorsExample.c
 *
 * [EOF]
 */
