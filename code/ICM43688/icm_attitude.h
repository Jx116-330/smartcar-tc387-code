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
uint8 icm_attitude_is_gyro_bias_from_flash(void);
uint8 icm_attitude_save_gyro_bias_to_flash(void);
uint8 icm_attitude_load_gyro_bias_from_flash(void);
void icm_attitude_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg);
void icm_attitude_get_quaternion(float *q0, float *q1, float *q2, float *q3);
float icm_attitude_get_acc_norm_g(void);
uint32 icm_attitude_get_update_count(void);

/* ---- Yaw debug / tuning API ---- */

/* 获取全套 yaw 调试量（任意参数可传 NULL） */
void icm_attitude_yaw_get_debug(float *gz_raw,      float *gz_bias,
                                 float *gz_comp,     float *gz_scaled,
                                 float *yaw_integral,float *yaw_final,
                                 float *yaw_corr,    float *dt_s);

/* 手动 z 轴偏置（叠加在主校准 bias 之上） */
void  icm_attitude_yaw_set_manual_bias(float bias_dps);
float icm_attitude_yaw_get_manual_bias(void);

/* z 轴陀螺仪缩放因子（默认 1.0） */
void  icm_attitude_yaw_set_scale(float scale);
float icm_attitude_yaw_get_scale(void);

/* 调试输出使能（供 TELY 按需开关） */
void  icm_attitude_yaw_set_debug_enable(uint8 en);
uint8 icm_attitude_yaw_get_debug_enable(void);

/* 参数版本号（每次 SET/ZERO 后自增，供桌面端检测是否生效） */
uint32 icm_attitude_yaw_get_param_version(void);

/* Yaw Zero：静止采样 z-bias，采样结束后自动应用到 manual_bias */
void icm_attitude_yaw_zero_start(uint32 samples);
void icm_attitude_yaw_zero_get_state(uint8 *busy, uint8 *ok,
                                      uint32 *n,   float *gbz);

/* Yaw Test：90° 旋转测试辅助 */
void  icm_attitude_yaw_test_start(void);
void  icm_attitude_yaw_test_stop(float *out_start, float *out_end,
                                  float *out_delta, float *out_err90);
uint8 icm_attitude_yaw_test_is_on(void);
float icm_attitude_yaw_test_get_start(void);
float icm_attitude_yaw_test_get_last_delta(void);

#endif /* __ICM_ATTITUDE_H__ */
