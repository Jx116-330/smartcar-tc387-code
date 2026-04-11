/**
 * @file ins_playback.c
 * @brief INS 轨迹回放骨架实现
 * @author JX116
 * @date 2026-04-07
 *
 * 只做目标点推进，不含任何控制器。
 * 上层（steering / motor）通过 ins_playback_get_target() 取当前目标点后自行控制。
 */

#include "ins_playback.h"
#include "ins_record.h"
#include "icm_ins.h"

#include <math.h>

/* ------------------------------------------------------------------ */
/* 内部状态                                                             */
/* ------------------------------------------------------------------ */
typedef struct
{
    ins_play_state_t state;
    uint16           total_points;          /**< load 时快照的轨迹点数   */
    uint16           current_target_idx;    /**< 当前目标点索引          */
    float            reach_threshold_m;     /**< 到点切换距离阈值        */
    float            last_dist_m;           /**< 上次 task 计算的距离    */
} ins_play_ctx_t;

static ins_play_ctx_t g_play = {
    INS_PLAY_IDLE,
    0U,
    0U,
    INS_PLAY_REACH_THRESHOLD_M,
    -1.0f
};

/* ------------------------------------------------------------------ */
/* 内部辅助                                                             */
/* ------------------------------------------------------------------ */
static float play_dist2d(float dx, float dy)
{
    return sqrtf(dx * dx + dy * dy);
}

/* ------------------------------------------------------------------ */
/* 公开接口                                                             */
/* ------------------------------------------------------------------ */
void ins_playback_init(void)
{
    g_play.state              = INS_PLAY_IDLE;
    g_play.total_points       = 0U;
    g_play.current_target_idx = 0U;
    g_play.reach_threshold_m  = INS_PLAY_REACH_THRESHOLD_M;
    g_play.last_dist_m        = -1.0f;
}

uint8 ins_playback_load(void)
{
    uint16 count = ins_record_get_point_count();

    if (0U == count)
    {
        g_play.state = INS_PLAY_IDLE;
        return 0U;
    }

    g_play.total_points       = count;
    g_play.current_target_idx = 0U;
    g_play.last_dist_m        = -1.0f;
    g_play.state              = INS_PLAY_READY;
    return 1U;
}

void ins_playback_start(void)
{
    if (INS_PLAY_READY != g_play.state)
    {
        return;
    }

    g_play.current_target_idx = 0U;
    g_play.last_dist_m        = -1.0f;
    g_play.state              = INS_PLAY_RUNNING;
}

void ins_playback_stop(void)
{
    g_play.state       = INS_PLAY_IDLE;
    g_play.last_dist_m = -1.0f;
}

void ins_playback_task(void)
{
    const ins_record_point_t *tgt;
    float cur_px = 0.0f;
    float cur_py = 0.0f;
    float dist   = 0.0f;

    if (INS_PLAY_RUNNING != g_play.state)
    {
        return;
    }

    /* 安全检查：快照点数可能为 0 */
    if (0U == g_play.total_points)
    {
        ins_playback_stop();
        return;
    }

    /* 已到达最后一个点 */
    if (g_play.current_target_idx >= g_play.total_points)
    {
        ins_playback_stop();
        return;
    }

    tgt = ins_record_get_point_ptr(g_play.current_target_idx);
    if (NULL == tgt)
    {
        ins_playback_stop();
        return;
    }

    icm_ins_get_position(&cur_px, &cur_py);
    dist = play_dist2d(cur_px - tgt->px_m, cur_py - tgt->py_m);
    g_play.last_dist_m = dist;

    /* 到点：推进目标索引 */
    if (dist < g_play.reach_threshold_m)
    {
        g_play.current_target_idx++;

        if (g_play.current_target_idx >= g_play.total_points)
        {
            /* 所有点已遍历，回放结束 */
            ins_playback_stop();
        }
    }
}

uint8 ins_playback_is_running(void)
{
    return (INS_PLAY_RUNNING == g_play.state) ? 1U : 0U;
}

ins_play_state_t ins_playback_get_state(void)
{
    return g_play.state;
}

uint16 ins_playback_get_target_idx(void)
{
    return g_play.current_target_idx;
}

uint16 ins_playback_get_total_points(void)
{
    return g_play.total_points;
}

uint8 ins_playback_get_target(ins_record_point_t *out)
{
    if ((NULL == out) || (INS_PLAY_RUNNING != g_play.state))
    {
        return 0U;
    }

    return ins_record_get_point(g_play.current_target_idx, out);
}

float ins_playback_get_dist_to_target(void)
{
    if (INS_PLAY_RUNNING != g_play.state)
    {
        return -1.0f;
    }

    return g_play.last_dist_m;
}

void ins_playback_set_reach_threshold_m(float m)
{
    if (m < 0.05f)
    {
        m = 0.05f;
    }
    else if (m > 5.0f)
    {
        m = 5.0f;
    }

    g_play.reach_threshold_m = m;
}
