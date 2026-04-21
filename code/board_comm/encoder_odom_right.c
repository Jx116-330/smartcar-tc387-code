/*********************************************************************************************************************
 * File: encoder_odom_right.c
 * Brief: Right rear wheel odometry to INS fusion.
 *
 * The structure mirrors encoder_odom.c, but reads local rear_right_encoder data instead of TC264 ENCL frames.
 * It is intentionally independent from the left rear odometry state so both wheels can be fused in parallel.
 *********************************************************************************************************************/

#include <ICM42688/icm_attitude.h>
#include <ICM42688/icm_ins.h>
#include "encoder_odom_right.h"
#include "ins_enc_tune.h"

#include "rear_right_encoder.h"
#include <math.h>

/* 融合增益运行时可调：默认值定义在 ins_enc_tune.h，桌面端通过
 * SET INSENC_R_* / SAVE INSENC 修改与持久化。*/

#ifndef M_PI
#define M_PI             3.14159265358979323846f
#endif
#define DEG2RAD(d)       ((d) * (M_PI / 180.0f))

static float  eor_px_m   = 0.0f;
static float  eor_py_m   = 0.0f;
static float  eor_spd_ms = 0.0f;

static int32  eor_last_dist_mm    = 0;
static uint32 eor_last_process_ms = 0U;
static uint8  eor_initialized     = 0U;
static uint8  eor_active          = 0U;
static uint8  eor_enabled         = 1U;

void encoder_odom_right_init(void)
{
    eor_px_m   = 0.0f;
    eor_py_m   = 0.0f;
    eor_spd_ms = 0.0f;

    eor_last_dist_mm    = 0;
    eor_last_process_ms = 0U;
    eor_initialized     = 0U;
    eor_active          = 0U;
    eor_enabled         = 1U;
}

void encoder_odom_right_task(void)
{
    uint32 rx_ms;
    int32  cur_dist_mm;
    int32  delta_dist_mm;
    float  delta_dist_m;
    float  yaw_deg;
    float  yaw_rad;
    float  enc_vx, enc_vy;
    float  ins_vx, ins_vy;
    float  ins_px, ins_py;

    rx_ms = rear_right_get_last_rx_ms();
    if (rx_ms == eor_last_process_ms)
    {
        return;
    }
    eor_last_process_ms = rx_ms;

    eor_spd_ms = (float)rear_right_get_spd_mm_s() * 0.001f;

    if (0U == eor_enabled)
    {
        eor_active = 0U;
        return;
    }
    if (0U == rear_right_is_online())
    {
        eor_active = 0U;
        return;
    }
    if (0U != icm_ins_is_stationary())
    {
        eor_active = 0U;
        return;
    }

    icm_ins_get_position(&ins_px, &ins_py);
    icm_ins_get_velocity(&ins_vx, &ins_vy, NULL);
    icm_attitude_get_euler(NULL, NULL, &yaw_deg);
    yaw_rad = DEG2RAD(yaw_deg);

    if (0U == eor_initialized)
    {
        eor_px_m          = ins_px;
        eor_py_m          = ins_py;
        eor_last_dist_mm  = rear_right_get_dist_mm();
        eor_initialized   = 1U;
        eor_active        = 1U;
        return;
    }

    cur_dist_mm = rear_right_get_dist_mm();
    delta_dist_mm = cur_dist_mm - eor_last_dist_mm;
    eor_last_dist_mm = cur_dist_mm;
    delta_dist_m = (float)delta_dist_mm * 0.001f;

    enc_vx = eor_spd_ms * cosf(yaw_rad);
    enc_vy = eor_spd_ms * sinf(yaw_rad);

    eor_px_m += delta_dist_m * cosf(yaw_rad);
    eor_py_m += delta_dist_m * sinf(yaw_rad);

    {
        float sync_rate = ins_enc_tune_get_r_sync_rate();
        float vel_gain  = ins_enc_tune_get_r_vel_gain();
        float pos_gain  = ins_enc_tune_get_r_pos_gain();

        eor_px_m += sync_rate * (ins_px - eor_px_m);
        eor_py_m += sync_rate * (ins_py - eor_py_m);

        icm_ins_correct_velocity(vel_gain * (enc_vx - ins_vx),
                                 vel_gain * (enc_vy - ins_vy));

        icm_ins_correct_position(pos_gain * (eor_px_m - ins_px),
                                 pos_gain * (eor_py_m - ins_py));
    }

    eor_active = 1U;
}

void encoder_odom_right_reset(void)
{
    eor_px_m   = 0.0f;
    eor_py_m   = 0.0f;
    eor_spd_ms = 0.0f;
    eor_last_dist_mm    = 0;
    eor_last_process_ms = 0U;
    eor_initialized     = 0U;
    eor_active          = 0U;
}

void encoder_odom_right_set_enable(uint8 en)
{
    eor_enabled = (0U != en) ? 1U : 0U;
    if (0U == eor_enabled)
    {
        eor_active = 0U;
    }
}

uint8 encoder_odom_right_is_enabled(void) { return eor_enabled; }

float encoder_odom_right_get_px_m(void)     { return eor_px_m; }
float encoder_odom_right_get_py_m(void)     { return eor_py_m; }
float encoder_odom_right_get_speed_ms(void) { return eor_spd_ms; }
uint8 encoder_odom_right_is_active(void)    { return eor_active; }
