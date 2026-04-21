/*********************************************************************************************************************
 * File: ins_enc_tune.h
 * Brief: Runtime-tunable INS<->encoder fusion gains with flash persistence.
 *
 *   6 gains total, wired into encoder_odom (left) and encoder_odom_right:
 *     L_vel_gain, L_pos_gain, L_sync_rate
 *     R_vel_gain, R_pos_gain, R_sync_rate
 *
 *   Desktop tuning tool streams current values via TELINSENC, sends
 *   SET INSENC_* commands to update at runtime, and SAVE INSENC
 *   to persist them to flash page 114 (sector 0). Reset restores defaults.
 *********************************************************************************************************************/

#ifndef __INS_ENC_TUNE_H__
#define __INS_ENC_TUNE_H__

#include "zf_common_typedef.h"

/* 默认值保持与原 #define 一致，确保首次刷写行为无差异。*/
#define INS_ENC_TUNE_DEF_L_VEL_GAIN   0.0375f
#define INS_ENC_TUNE_DEF_L_POS_GAIN   0.0075f
#define INS_ENC_TUNE_DEF_L_SYNC_RATE  0.0025f
#define INS_ENC_TUNE_DEF_R_VEL_GAIN   0.04f
#define INS_ENC_TUNE_DEF_R_POS_GAIN   0.008f
#define INS_ENC_TUNE_DEF_R_SYNC_RATE  0.0025f

/* 合法范围：非负 + 上限约束，防止 SET 到发散值 */
#define INS_ENC_TUNE_MIN_GAIN         0.0f
#define INS_ENC_TUNE_MAX_GAIN         1.0f

void  ins_enc_tune_init(void);            /* 上电调用，尝试从 flash 加载，失败回默认 */
void  ins_enc_tune_reset_to_default(void);

/* Getters — 由 encoder_odom / encoder_odom_right 每帧读取 */
float ins_enc_tune_get_l_vel_gain(void);
float ins_enc_tune_get_l_pos_gain(void);
float ins_enc_tune_get_l_sync_rate(void);
float ins_enc_tune_get_r_vel_gain(void);
float ins_enc_tune_get_r_pos_gain(void);
float ins_enc_tune_get_r_sync_rate(void);

/* Setters — 返回 1 表示成功，0 表示越界（值会原样保留） */
uint8 ins_enc_tune_set_l_vel_gain(float v);
uint8 ins_enc_tune_set_l_pos_gain(float v);
uint8 ins_enc_tune_set_l_sync_rate(float v);
uint8 ins_enc_tune_set_r_vel_gain(float v);
uint8 ins_enc_tune_set_r_pos_gain(float v);
uint8 ins_enc_tune_set_r_sync_rate(float v);

/* Flash 持久化 */
uint8 ins_enc_tune_save_to_flash(void);
uint8 ins_enc_tune_load_from_flash(void);
uint8 ins_enc_tune_is_from_flash(void);
uint8 ins_enc_tune_is_flash_synced(void);

/* 参数版本号：每次 SET/LOAD/RESET 后自增，供桌面端确认生效 */
uint32 ins_enc_tune_get_param_version(void);

#endif /* __INS_ENC_TUNE_H__ */
