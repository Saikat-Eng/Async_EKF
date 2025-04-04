/*
 * File: PoseEstimatior.h
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 14-Jun-2023 18:12:47
 */

#ifndef RUN_POSEESTIMATIONFROMASYNCHRONOUSSENSORSEXAMPLE_H
#define RUN_POSEESTIMATIONFROMASYNCHRONOUSSENSORSEXAMPLE_H

typedef struct {
    double ReferenceLocation[3];
    float pState[28];
    float pStateCovariance[784];
    float addProcNoise[784];
    float accl_innov[3];
    float gyro_innov[3];
    float mag_innov[3];
    float gps_innov[6];
} c_fusion_internal_coder_AsyncMA;


typedef struct {

    float Rmag[9];
    float Racc[9];
    float Rgyro[9];
    float Rgps[36];

    float QuaternionNoise[4];
    float AngularVelocityNoise[3];
    float PositionNoise[3];
    float VelocityNoise[3];
    float AccelerationNoise[3];
    float GyroscopeBiasNoise[3];
    float AccelerometerBiasNoise[3];
    float GeomagneticVectorNoise[3];
    float MagnetometerBiasNoise[3];
}c_fusion_internal_coder_AsyncMA_Cfg;

extern c_fusion_internal_coder_AsyncMA* get_ekf_instance(void);
extern c_fusion_internal_coder_AsyncMA_Cfg* ekf_cfg_instance(void);
extern void ekf_init(double *ref_loc, float *mag_field, float *q_init);
extern float *get_state(void);
extern void set_ref_loc(double *ref_loc);
#endif

/*
 * File trailer for run_PoseEstimationFromAsynchronousSensorsExample.h
 *
 * [EOF]
 */
