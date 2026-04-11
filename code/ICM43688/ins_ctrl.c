/**
 * @file ins_ctrl.c
 * @brief INS 轨迹复现控制骨架实现
 * @author JX116
 * @date 2026-04-07
 *
 * 计算逻辑（每次 task() 刷新）：
 *   1. 检查 ins_playback 是否 RUNNING；否则 drive_enable=0，清零输出
 *   2. 获取当前目标点（ins_playback_get_target）；无效则 drive_enable=0
 *   3. 计算 dx / dy / dist_to_target
 *   4. 计算 target_yaw = atan2f(dy, dx)，转换为度
 *   5. 计算 yaw_err = target_yaw - cur_yaw，归一化到 [-180, 180]
 *   6. 计算 speed_cmd：
 *      - dist > INS_CTRL_CRUISE_DIST_M → INS_CTRL_FULL_SPEED_MS（巡航速度）
 *      - dist ≤ INS_CTRL_CRUISE_DIST_M → INS_CTRL_FULL_SPEED_MS * INS_CTRL_SLOW_SPEED_RATIO（减速）
 */

#include "ins_ctrl.h"
#include "ins_playback.h"
#include "ins_record.h"
#include "icm_ins.h"
#include "icm_attitude.h"

#include <math.h>

/* ------------------------------------------------------------------ */
/* 内部状态                                                             */
/* ------------------------------------------------------------------ */
static ins_ctrl_output_t g_ctrl_out;

/* ------------------------------------------------------------------ */
/* 内部辅助                                                             */
/* ------------------------------------------------------------------ */

/**
 * 将角度归一化到 (-180, 180]。
 */
static float ctrl_normalize_angle_deg(float deg)
{
    while (deg >  180.0f) { deg -= 360.0f; }
    while (deg <= -180.0f) { deg += 360.0f; }
    return deg;
}

/** 清零输出并设 drive_enable=0 */
static void ctrl_clear_output(void)
{
    g_ctrl_out.tgt_dist_m   = -1.0f;
    g_ctrl_out.tgt_yaw_deg  = 0.0f;
    g_ctrl_out.yaw_err_deg  = 0.0f;
    g_ctrl_out.speed_cmd    = 0.0f;
    g_ctrl_out.drive_enable = 0U;
    g_ctrl_out.tgt_px_m     = 0.0f;
    g_ctrl_out.tgt_py_m     = 0.0f;
    /* cur_px/py/yaw 仍然刷新，方便调试显示 */
}

/* ------------------------------------------------------------------ */
/* 公开接口                                                             */
/* ------------------------------------------------------------------ */
void ins_ctrl_init(void)
{
    g_ctrl_out.cur_px_m     = 0.0f;
    g_ctrl_out.cur_py_m     = 0.0f;
    g_ctrl_out.cur_yaw_deg  = 0.0f;
    ctrl_clear_output();
}

void ins_ctrl_task(void)
{
    ins_record_point_t tgt;
    float roll_deg  = 0.0f;
    float pitch_deg = 0.0f;
    float cur_yaw   = 0.0f;
    float cur_px    = 0.0f;
    float cur_py    = 0.0f;
    float dx        = 0.0f;
    float dy        = 0.0f;
    float dist      = 0.0f;
    float tgt_yaw   = 0.0f;
    float yaw_err   = 0.0f;
    float spd_cmd   = 0.0f;

    /* 始终刷新当前姿态和位置，供调试显示 */
    icm_ins_get_position(&cur_px, &cur_py);
    icm_attitude_get_euler(&roll_deg, &pitch_deg, &cur_yaw);
    g_ctrl_out.cur_px_m    = cur_px;
    g_ctrl_out.cur_py_m    = cur_py;
    g_ctrl_out.cur_yaw_deg = cur_yaw;

    /* 检查 playback 是否运行 */
    if (!ins_playback_is_running())
    {
        ctrl_clear_output();
        return;
    }

    /* 获取当前目标点（内部已检查 RUNNING + 越界） */
    if (!ins_playback_get_target(&tgt))
    {
        ctrl_clear_output();
        return;
    }

    /* 几何量计算 */
    dx   = tgt.px_m - cur_px;
    dy   = tgt.py_m - cur_py;
    dist = sqrtf(dx * dx + dy * dy);

    /* 目标方位角（导航坐标系约定：x=East, y=North, z=Up，即标准 ENU）
     * atan2(dy, dx) 给出从 +X(East) 方向逆时针到目标的数学角，
     * 与四元数 yaw 定义一致（正值=逆时针），可直接相减得到航向误差。 */
    tgt_yaw = atan2f(dy, dx) * (180.0f / 3.14159265f);

    yaw_err = ctrl_normalize_angle_deg(tgt_yaw - cur_yaw);

    /* 速度建议：距离远时巡航，接近目标时减速 */
    if (dist > INS_CTRL_CRUISE_DIST_M)
    {
        spd_cmd = INS_CTRL_FULL_SPEED_MS;
    }
    else
    {
        spd_cmd = INS_CTRL_FULL_SPEED_MS * INS_CTRL_SLOW_SPEED_RATIO;
    }

    /* 写入输出 */
    g_ctrl_out.tgt_dist_m   = dist;
    g_ctrl_out.tgt_yaw_deg  = tgt_yaw;
    g_ctrl_out.yaw_err_deg  = yaw_err;
    g_ctrl_out.speed_cmd    = spd_cmd;
    g_ctrl_out.drive_enable = 1U;
    g_ctrl_out.tgt_px_m     = tgt.px_m;
    g_ctrl_out.tgt_py_m     = tgt.py_m;
}

const ins_ctrl_output_t *ins_ctrl_get_output(void)
{
    return &g_ctrl_out;
}
