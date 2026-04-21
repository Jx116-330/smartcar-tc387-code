/*********************************************************************************************************************
 * File: encoder_odom.c
 * Brief: 编码器里程计 → INS 融合
 *
 * 算法概要:
 *   1. 每收到新 ENCL 帧 (25Hz)，将编码器速度沿航向分解为 ENU 分量
 *   2. 速度校正: gain=0.15 的互补滤波修正 INS 速度漂移
 *   3. 位置校正: 维护编码器独立 DR 位置 → gain=0.03 修正 INS 位置
 *   4. 防漂移回拉: 编码器 DR 位置以 0.01 速率向 INS 位置靠拢，防止航向漂移导致发散
 *
 * 坐标系: ENU (X=东, Y=北)。yaw 从 +X(东) 逆时针。
 *   vx = speed * cos(yaw_rad),  vy = speed * sin(yaw_rad)
 *
 * Author: JX116
 *********************************************************************************************************************/

#include <ICM42688/icm_attitude.h>
#include <ICM42688/icm_ins.h>
#include "encoder_odom.h"
#include "ins_enc_tune.h"
#include "rear_left_encoder.h"
#include <math.h>

/* 融合增益运行时可调：默认值定义在 ins_enc_tune.h，桌面调参工具通过
 * SET INSENC_L_* / SAVE INSENC 在线修改与持久化。 */

#ifndef M_PI
#define M_PI            3.14159265358979323846f
#endif
#define DEG2RAD(d)      ((d) * (M_PI / 180.0f))

/* ==== 内部状态 ============================================================ */
static float  eo_px_m   = 0.0f;    /* 编码器 DR 位置 X (ENU, m)   */
static float  eo_py_m   = 0.0f;    /* 编码器 DR 位置 Y (ENU, m)   */
static float  eo_spd_ms = 0.0f;    /* 当前帧编码器速度 (m/s)      */

static int32  eo_last_dist_mm    = 0;       /* 上次处理的编码器累计距离   */
static uint32 eo_last_process_ms = 0U;      /* 上次处理的 ENCL 时间戳    */
static uint8  eo_initialized     = 0U;      /* 首帧初始化标志             */
static uint8  eo_active          = 0U;      /* 当前帧是否执行了融合       */
static uint8  eo_enabled         = 1U;      /* 融合开关（默认开启）       */

/* ==== 初始化 ============================================================== */
void encoder_odom_init(void)
{
    eo_px_m   = 0.0f;
    eo_py_m   = 0.0f;
    eo_spd_ms = 0.0f;

    eo_last_dist_mm    = 0;
    eo_last_process_ms = 0U;
    eo_initialized     = 0U;
    eo_active          = 0U;
    eo_enabled         = 1U;
}

/* ==== 主循环任务 ========================================================== */
void encoder_odom_task(void)
{
    uint32 rx_ms;
    int32  cur_dist_mm;
    int32  delta_dist_mm;
    float  delta_dist_m;
    float  roll_deg, pitch_deg, yaw_deg;
    float  yaw_rad;
    float  enc_vx, enc_vy;
    float  ins_vx, ins_vy, ins_vz;
    float  ins_px, ins_py;

    /* ---- 帧率门控: 只在新 ENCL 帧到达时处理 ---- */
    rx_ms = rear_left_get_last_rx_ms();
    if (rx_ms == eo_last_process_ms)
    {
        return;                             /* 没有新帧，跳过 */
    }
    eo_last_process_ms = rx_ms;

    /* 采集编码器速度（即使融合关闭也更新，供菜单显示） */
    {
        int32 spd_mm_s = rear_left_get_spd_mm_s();
        eo_spd_ms = (float)spd_mm_s * 0.001f;
        /* 外部证据喂给 icm_ins，抑制 IMU-only ZUPT 的低速误触发。
         * 放在 enabled / online 前置检查之前，确保只要编码器还在产出速度，
         * 提示就持续刷新；若编码器掉线或融合关闭，TTL 自然过期回退纯 IMU。 */
        icm_ins_set_motion_hint_mm_s(spd_mm_s);
    }

    /* ---- 前置检查 ---- */
    if (0U == eo_enabled)
    {
        eo_active = 0U;
        return;                             /* 融合已关闭 */
    }
    if (0U == rear_left_is_online())
    {
        eo_active = 0U;
        return;                             /* 编码器离线 */
    }
    if (0U != icm_ins_is_stationary())
    {
        eo_active = 0U;
        return;                             /* ZUPT 静止态，不校正 */
    }

    /* ---- 获取当前 INS 状态 ---- */
    icm_ins_get_position(&ins_px, &ins_py);
    icm_ins_get_velocity(&ins_vx, &ins_vy, &ins_vz);
    icm_attitude_get_euler(&roll_deg, &pitch_deg, &yaw_deg);
    yaw_rad = DEG2RAD(yaw_deg);

    /* ---- 首帧初始化: 同步 DR 位置到 INS ---- */
    if (0U == eo_initialized)
    {
        eo_px_m         = ins_px;
        eo_py_m         = ins_py;
        eo_last_dist_mm = rear_left_get_dist_mm();
        eo_initialized  = 1U;
        eo_active       = 1U;
        return;                             /* 首帧不校正，仅同步 */
    }

    /* ---- 计算距离增量 ---- */
    cur_dist_mm  = rear_left_get_dist_mm();
    delta_dist_mm = cur_dist_mm - eo_last_dist_mm;
    eo_last_dist_mm = cur_dist_mm;
    delta_dist_m = (float)delta_dist_mm * 0.001f;

    /* ---- ENU 分解（eo_spd_ms 已在帧率门控后更新） ---- */
    enc_vx = eo_spd_ms * cosf(yaw_rad);
    enc_vy = eo_spd_ms * sinf(yaw_rad);

    /* ---- 更新编码器 DR 位置 ---- */
    eo_px_m += delta_dist_m * cosf(yaw_rad);
    eo_py_m += delta_dist_m * sinf(yaw_rad);

    /* 防漂移回拉: 缓慢向 INS 位置靠拢（~4 秒时间常数 @ 25Hz） */
    {
        float sync_rate = ins_enc_tune_get_l_sync_rate();
        float vel_gain  = ins_enc_tune_get_l_vel_gain();
        float pos_gain  = ins_enc_tune_get_l_pos_gain();

        eo_px_m += sync_rate * (ins_px - eo_px_m);
        eo_py_m += sync_rate * (ins_py - eo_py_m);

        /* ---- 速度校正 ---- */
        icm_ins_correct_velocity(vel_gain * (enc_vx - ins_vx),
                                 vel_gain * (enc_vy - ins_vy));

        /* ---- 位置校正 ---- */
        icm_ins_correct_position(pos_gain * (eo_px_m - ins_px),
                                 pos_gain * (eo_py_m - ins_py));
    }

    eo_active = 1U;
}

/* ==== 控制接口 ============================================================ */
void encoder_odom_reset(void)
{
    eo_px_m   = 0.0f;
    eo_py_m   = 0.0f;
    eo_spd_ms = 0.0f;
    eo_last_dist_mm    = 0;
    eo_last_process_ms = 0U;
    eo_initialized     = 0U;    /* 下帧自动重新同步 INS 位置 */
    eo_active          = 0U;
}

void encoder_odom_set_enable(uint8 en)
{
    eo_enabled = (0U != en) ? 1U : 0U;
    if (0U == eo_enabled)
    {
        eo_active = 0U;
    }
}

uint8 encoder_odom_is_enabled(void) { return eo_enabled; }

/* ==== 只读 Getter ========================================================= */
float encoder_odom_get_px_m(void)      { return eo_px_m; }
float encoder_odom_get_py_m(void)      { return eo_py_m; }
float encoder_odom_get_speed_ms(void)  { return eo_spd_ms; }
uint8 encoder_odom_is_active(void)     { return eo_active; }
