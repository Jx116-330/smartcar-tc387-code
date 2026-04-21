/*********************************************************************************************************************
 * File: ins_enc_tune.c
 * Brief: Runtime-tunable INS<->encoder fusion gains with flash persistence.
 *********************************************************************************************************************/

#include "ins_enc_tune.h"
#include "zf_driver_flash.h"

/* Flash 布局（sector 0）：
 *   page 115: gyro bias           (icm_attitude)
 *   page 116..123: path recorder
 *   page 124..127: menu params
 *   page 114:  INS encoder tuning  <-- 本模块
 */
#define INS_ENC_TUNE_FLASH_SECTOR   0U
#define INS_ENC_TUNE_FLASH_PAGE     114U
#define INS_ENC_TUNE_FLASH_WORDS    8U   /* 6 floats + magic + version */
#define INS_ENC_TUNE_MAGIC          0x49454E43U  /* 'IENC' */

typedef struct
{
    float l_vel_gain;
    float l_pos_gain;
    float l_sync_rate;
    float r_vel_gain;
    float r_pos_gain;
    float r_sync_rate;
    uint8 from_flash;
    uint8 flash_synced;
    uint32 param_version;
} ins_enc_tune_state_t;

static volatile ins_enc_tune_state_t g_tune = {
    INS_ENC_TUNE_DEF_L_VEL_GAIN,
    INS_ENC_TUNE_DEF_L_POS_GAIN,
    INS_ENC_TUNE_DEF_L_SYNC_RATE,
    INS_ENC_TUNE_DEF_R_VEL_GAIN,
    INS_ENC_TUNE_DEF_R_POS_GAIN,
    INS_ENC_TUNE_DEF_R_SYNC_RATE,
    0U,
    0U,
    0U
};

static uint8 ins_enc_tune_in_range(float v)
{
    if (v < INS_ENC_TUNE_MIN_GAIN) return 0U;
    if (v > INS_ENC_TUNE_MAX_GAIN) return 0U;
    return 1U;
}

static void ins_enc_tune_bump_version(void)
{
    g_tune.param_version++;
    g_tune.flash_synced = 0U;
}

void ins_enc_tune_reset_to_default(void)
{
    g_tune.l_vel_gain  = INS_ENC_TUNE_DEF_L_VEL_GAIN;
    g_tune.l_pos_gain  = INS_ENC_TUNE_DEF_L_POS_GAIN;
    g_tune.l_sync_rate = INS_ENC_TUNE_DEF_L_SYNC_RATE;
    g_tune.r_vel_gain  = INS_ENC_TUNE_DEF_R_VEL_GAIN;
    g_tune.r_pos_gain  = INS_ENC_TUNE_DEF_R_POS_GAIN;
    g_tune.r_sync_rate = INS_ENC_TUNE_DEF_R_SYNC_RATE;
    g_tune.from_flash  = 0U;
    ins_enc_tune_bump_version();
}

void ins_enc_tune_init(void)
{
    /* 先设默认，再尝试加载 flash；失败则保持默认 */
    ins_enc_tune_reset_to_default();
    (void)ins_enc_tune_load_from_flash();
}

/* Getters */
float ins_enc_tune_get_l_vel_gain(void)   { return g_tune.l_vel_gain;  }
float ins_enc_tune_get_l_pos_gain(void)   { return g_tune.l_pos_gain;  }
float ins_enc_tune_get_l_sync_rate(void)  { return g_tune.l_sync_rate; }
float ins_enc_tune_get_r_vel_gain(void)   { return g_tune.r_vel_gain;  }
float ins_enc_tune_get_r_pos_gain(void)   { return g_tune.r_pos_gain;  }
float ins_enc_tune_get_r_sync_rate(void)  { return g_tune.r_sync_rate; }

/* Setters */
uint8 ins_enc_tune_set_l_vel_gain(float v)
{
    if (!ins_enc_tune_in_range(v)) return 0U;
    g_tune.l_vel_gain = v;
    ins_enc_tune_bump_version();
    return 1U;
}
uint8 ins_enc_tune_set_l_pos_gain(float v)
{
    if (!ins_enc_tune_in_range(v)) return 0U;
    g_tune.l_pos_gain = v;
    ins_enc_tune_bump_version();
    return 1U;
}
uint8 ins_enc_tune_set_l_sync_rate(float v)
{
    if (!ins_enc_tune_in_range(v)) return 0U;
    g_tune.l_sync_rate = v;
    ins_enc_tune_bump_version();
    return 1U;
}
uint8 ins_enc_tune_set_r_vel_gain(float v)
{
    if (!ins_enc_tune_in_range(v)) return 0U;
    g_tune.r_vel_gain = v;
    ins_enc_tune_bump_version();
    return 1U;
}
uint8 ins_enc_tune_set_r_pos_gain(float v)
{
    if (!ins_enc_tune_in_range(v)) return 0U;
    g_tune.r_pos_gain = v;
    ins_enc_tune_bump_version();
    return 1U;
}
uint8 ins_enc_tune_set_r_sync_rate(float v)
{
    if (!ins_enc_tune_in_range(v)) return 0U;
    g_tune.r_sync_rate = v;
    ins_enc_tune_bump_version();
    return 1U;
}

/* Flash 持久化：采用与 icm_attitude_save_gyro_bias_to_flash 一致的 punning 风格 */
uint8 ins_enc_tune_save_to_flash(void)
{
    uint32 buf[INS_ENC_TUNE_FLASH_WORDS];
    float  lvg = g_tune.l_vel_gain;
    float  lpg = g_tune.l_pos_gain;
    float  lsr = g_tune.l_sync_rate;
    float  rvg = g_tune.r_vel_gain;
    float  rpg = g_tune.r_pos_gain;
    float  rsr = g_tune.r_sync_rate;

    buf[0] = *((const uint32 *)(const void *)&lvg);
    buf[1] = *((const uint32 *)(const void *)&lpg);
    buf[2] = *((const uint32 *)(const void *)&lsr);
    buf[3] = *((const uint32 *)(const void *)&rvg);
    buf[4] = *((const uint32 *)(const void *)&rpg);
    buf[5] = *((const uint32 *)(const void *)&rsr);
    buf[6] = INS_ENC_TUNE_MAGIC;
    buf[7] = g_tune.param_version;

    flash_write_page(INS_ENC_TUNE_FLASH_SECTOR, INS_ENC_TUNE_FLASH_PAGE,
                     buf, (uint16)INS_ENC_TUNE_FLASH_WORDS);
    g_tune.flash_synced = 1U;
    g_tune.from_flash   = 1U;
    return 1U;
}

uint8 ins_enc_tune_load_from_flash(void)
{
    uint32 buf[INS_ENC_TUNE_FLASH_WORDS];
    float  lvg, lpg, lsr, rvg, rpg, rsr;

    flash_read_page(INS_ENC_TUNE_FLASH_SECTOR, INS_ENC_TUNE_FLASH_PAGE,
                    buf, (uint16)INS_ENC_TUNE_FLASH_WORDS);

    if (buf[6] != INS_ENC_TUNE_MAGIC)
    {
        return 0U;
    }

    lvg = *((const float *)(const void *)&buf[0]);
    lpg = *((const float *)(const void *)&buf[1]);
    lsr = *((const float *)(const void *)&buf[2]);
    rvg = *((const float *)(const void *)&buf[3]);
    rpg = *((const float *)(const void *)&buf[4]);
    rsr = *((const float *)(const void *)&buf[5]);

    /* 有效性检查：任一项越界则拒绝加载（视为损坏的 flash 区块） */
    if (!ins_enc_tune_in_range(lvg) || !ins_enc_tune_in_range(lpg) ||
        !ins_enc_tune_in_range(lsr) || !ins_enc_tune_in_range(rvg) ||
        !ins_enc_tune_in_range(rpg) || !ins_enc_tune_in_range(rsr))
    {
        return 0U;
    }

    g_tune.l_vel_gain   = lvg;
    g_tune.l_pos_gain   = lpg;
    g_tune.l_sync_rate  = lsr;
    g_tune.r_vel_gain   = rvg;
    g_tune.r_pos_gain   = rpg;
    g_tune.r_sync_rate  = rsr;
    g_tune.from_flash   = 1U;
    g_tune.flash_synced = 1U;
    g_tune.param_version++;   /* 通知桌面端参数已变 */
    return 1U;
}

uint8  ins_enc_tune_is_from_flash(void)    { return g_tune.from_flash; }
uint8  ins_enc_tune_is_flash_synced(void)  { return g_tune.flash_synced; }
uint32 ins_enc_tune_get_param_version(void){ return g_tune.param_version; }
