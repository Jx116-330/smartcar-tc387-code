#ifndef __ICM_INS_H__
#define __ICM_INS_H__

#include "zf_common_typedef.h"

void  icm_ins_init(void);
void  icm_ins_reset_velocity(void);
void  icm_ins_reset_position(void);
void  icm_ins_update(float acc_x_g, float acc_y_g, float acc_z_g,
                     float gyro_x_dps, float gyro_y_dps, float gyro_z_dps,
                     float dt_s);
void  icm_ins_set_velocity(float vx_ms, float vy_ms, float vz_ms);
void  icm_ins_get_velocity(float *vx_ms, float *vy_ms, float *vz_ms);
void  icm_ins_get_position(float *px_m, float *py_m);
void  icm_ins_get_linear_acc(float *ax_ms2, float *ay_ms2, float *az_ms2);
float icm_ins_get_speed_ms(void);
uint8 icm_ins_is_stationary(void);

/* 外部校正接口（供 GPS 融合模块在主循环调用，增量式修正） */
void  icm_ins_correct_position(float delta_px_m, float delta_py_m);
void  icm_ins_correct_velocity(float delta_vx_ms, float delta_vy_ms);

/* 运动提示：编码器/轮速来源在主循环周期调用，告诉 INS 当前"确实在动"。
 * |mm_s| ≥ 内部阈值时抑制 ZUPT 进入并立即退出静止态；若超过 TTL 不再更新，
 * 自动回退到纯 IMU 的 ZUPT 判据。参数带符号，便于未来利用方向信息。 */
void  icm_ins_set_motion_hint_mm_s(int32 mm_s);

#endif /* __ICM_INS_H__ */
