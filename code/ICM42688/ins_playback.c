/**
 * @file ins_playback.c
 * @brief INS 轨迹回放 — 里程驱动
 * @author JX116
 * @date 2026-04-07
 *
 * 改进：用编码器累计里程（enc_dist_mm）驱动目标点推进，
 * 比纯 INS 位置距离更稳定（不受惯导漂移影响）。
 *
 * 流程：
 *   1. load 时记录第一个点的 enc_dist_mm 作为里程基准
 *   2. start 时记录当前编码器里程作为回放起点
 *   3. task 中：回放里程 = |当前编码器里程 - 起点里程|
 *      当回放里程 ≥ 目标点相对里程时，推进到下一个目标
 *   4. 同时保留 INS 位置距离作为辅助（调试+ctrl用）
 */

#include <ICM42688/icm_ins.h>
#include <ICM42688/ins_playback.h>
#include <ICM42688/ins_record.h>
#include "board_comm.h"

#include <math.h>

/* ------------------------------------------------------------------ */
/* 内部状态                                                             */
/* ------------------------------------------------------------------ */
typedef struct
{
    ins_play_state_t state;
    uint16           total_points;
    uint16           current_target_idx;
    float            reach_threshold_m;
    float            last_dist_m;           /**< INS 位置距离（调试用）   */

    /* 里程驱动 */
    int32            base_enc_dist_mm;      /**< 第一个录制点的编码器里程 */
    int32            start_enc_dist_mm;     /**< 回放开始时的编码器里程   */
} ins_play_ctx_t;

static ins_play_ctx_t g_play = {
    INS_PLAY_IDLE,
    0U, 0U,
    INS_PLAY_REACH_THRESHOLD_M,
    -1.0f,
    0, 0
};

/* ------------------------------------------------------------------ */
/* 内部辅助                                                             */
/* ------------------------------------------------------------------ */
static float play_dist2d(float dx, float dy)
{
    return sqrtf(dx * dx + dy * dy);
}

/** 计算录制点相对于第一个点的里程（mm） */
static int32 play_point_mileage_mm(uint16 idx)
{
    const ins_record_point_t *pt = ins_record_get_point_ptr(idx);
    if (NULL == pt) return 0;
    return pt->enc_dist_mm - g_play.base_enc_dist_mm;
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
    g_play.base_enc_dist_mm   = 0;
    g_play.start_enc_dist_mm  = 0;
}

uint8 ins_playback_load(void)
{
    uint16 count = ins_record_get_point_count();
    const ins_record_point_t *first;

    if (0U == count)
    {
        g_play.state = INS_PLAY_IDLE;
        return 0U;
    }

    first = ins_record_get_point_ptr(0U);
    g_play.base_enc_dist_mm   = (NULL != first) ? first->enc_dist_mm : 0;
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
    g_play.start_enc_dist_mm  = board_comm_encl_get_dist_mm();
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
    int32 current_mileage_mm;
    int32 target_mileage_mm;
    uint8 reached_target = 0U;

    if (INS_PLAY_RUNNING != g_play.state)
    {
        return;
    }

    if (0U == g_play.total_points)
    {
        ins_playback_stop();
        return;
    }

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

    /* INS 位置距离（供 ins_ctrl 和调试用） */
    icm_ins_get_position(&cur_px, &cur_py);
    g_play.last_dist_m = play_dist2d(cur_px - tgt->px_m, cur_py - tgt->py_m);

    /* 里程驱动：当前回放里程 vs 目标点录制里程 */
    current_mileage_mm = board_comm_encl_get_dist_mm() - g_play.start_enc_dist_mm;
    target_mileage_mm  = play_point_mileage_mm(g_play.current_target_idx);

    /* 取绝对值（编码器可能倒转） */
    if (current_mileage_mm < 0) current_mileage_mm = -current_mileage_mm;
    if (target_mileage_mm  < 0) target_mileage_mm  = -target_mileage_mm;

    /* 里程已超过目标点 → 向前推进（可能跳多个点） */
    while (g_play.current_target_idx < g_play.total_points - 1U)
    {
        reached_target = (uint8)((current_mileage_mm >= target_mileage_mm) ||
                                 ((g_play.last_dist_m >= 0.0f) &&
                                  (g_play.last_dist_m <= g_play.reach_threshold_m)));
        if (!reached_target)
        {
            break;
        }
        g_play.current_target_idx++;
        tgt = ins_record_get_point_ptr(g_play.current_target_idx);
        if (NULL == tgt)
        {
            ins_playback_stop();
            return;
        }
        target_mileage_mm = play_point_mileage_mm(g_play.current_target_idx);
        if (target_mileage_mm < 0) target_mileage_mm = -target_mileage_mm;
        g_play.last_dist_m = play_dist2d(cur_px - tgt->px_m, cur_py - tgt->py_m);
    }

    /* 终点判定 */
    reached_target = (uint8)((current_mileage_mm >= target_mileage_mm) ||
                             ((g_play.last_dist_m >= 0.0f) &&
                              (g_play.last_dist_m <= g_play.reach_threshold_m)));
    if (g_play.current_target_idx >= g_play.total_points - 1U
        && reached_target)
    {
        ins_playback_stop();
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
