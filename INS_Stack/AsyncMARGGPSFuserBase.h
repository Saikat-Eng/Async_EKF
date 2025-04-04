/*
 * File: AsyncMARGGPSFuserBase.h
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 14-Jun-2023 18:12:47
 */

#ifndef ASYNCMARGGPSFUSERBASE_H
#define ASYNCMARGGPSFUSERBASE_H

/* Include Files */

#include "PoseEstimator.h"
#include "math.h"

/* Function Declarations */
extern void AsyncMARGGPSFuserBase_fuseaccel(c_fusion_internal_coder_AsyncMA *obj, const float *accel);
extern void AsyncMARGGPSFuserBase_fusegyro(c_fusion_internal_coder_AsyncMA *obj, const float *gyro);
extern void AsyncMARGGPSFuserBase_fusemag(c_fusion_internal_coder_AsyncMA *obj, const float *mag);
extern void AsyncMARGGPSFuserBase_fusegps(c_fusion_internal_coder_AsyncMA *obj, const float *lla, const float *vel);
extern void AsyncMARGGPSFuserBase_predict(c_fusion_internal_coder_AsyncMA *obj, const float dt);

#endif

/*
 * File trailer for AsyncMARGGPSFuserBase.h
 *
 * [EOF]
 */
