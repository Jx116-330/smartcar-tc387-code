#ifndef __ICM_ATTITUDE_H__
#define __ICM_ATTITUDE_H__

#include "zf_common_typedef.h"

void icm_attitude_init(void);
void icm_attitude_reset(void);
void icm_attitude_update(float gyro_x_dps,
                         float gyro_y_dps,
                         float gyro_z_dps,
                         float acc_x_g,
                         float acc_y_g,
                         float acc_z_g,
                         float dt_s);
void icm_attitude_start_gyro_bias_calibration(uint32 sample_count);
void icm_attitude_cancel_gyro_bias_calibration(void);
uint8 icm_attitude_is_gyro_bias_calibrating(void);
uint8 icm_attitude_is_gyro_bias_valid(void);
void icm_attitude_get_gyro_bias(float *bias_x_dps, float *bias_y_dps, float *bias_z_dps);
void icm_attitude_get_gyro_bias_calibration_progress(uint32 *sample_count, uint32 *target_count);
void icm_attitude_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg);
float icm_attitude_get_acc_norm_g(void);
uint32 icm_attitude_get_update_count(void);

#endif /* __ICM_ATTITUDE_H__ */
