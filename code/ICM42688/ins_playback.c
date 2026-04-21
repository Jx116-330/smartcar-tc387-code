/**
 * @file ins_playback.c
 * @brief INS 轨迹回放 — 有向分段里程驱动
 * @author JX116
 * @date 2026-04-07
 *
 * 改进：用编码器累计里程（enc_dist_mm）驱动目标点推进，
 * 比纯 INS 位置距离更稳定（不受惯导漂移影响）。
 *
 * 分段有向里程（2026-04 修订）：
 *   旧版用 |cur-start| 与 |pt.enc-base| 比较，录制中若含反向段会在
 *   `abs()` 后破坏单调性、导致索引乱跳。现改为"按段比较有向进度"：
 *     prev_offset = pt[idx-1].enc - base      (idx==0 时视为 0)
 *     tgt_offset  = pt[idx].enc   - base
 *     cur_offset  = cur_enc - start_enc
 *     segment_delta = tgt_offset - prev_offset    (signed)
 *     progress      = cur_offset - prev_offset    (signed)
 *     到达：同号且 |progress| ≥ |segment_delta|
 *   对极短段（|segment_delta| ≤ 半个 reach 阈值）保留 INS 距离兜底。
 */

#include <ICM42688/icm_ins.h>
#include <ICM42688/ins_playback.h>
#include <ICM42688/ins_record.h>
#include "rear_left_encoder.h"

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

/** 计算录制点相对于第一个点（基准）的里程偏移（mm，有向） */
static int32 play_point_mileage_mm(uint16 idx)
{
    const ins_record_point_t *pt = ins_record_get_point_ptr(idx);
    if (NULL == pt) return 0;
    return pt->enc_dist_mm - g_play.base_enc_dist_mm;
}

/** 上一个点（段起点）的里程偏移；idx==0 时段起点即基准点，返回 0 */
static int32 play_prev_mileage_mm(uint16 idx)
{
    if (0U == idx)
    {
        return 0;
    }
    return play_point_mileage_mm((uint16)(idx - 1U));
}

/** 有向到达判定。
 *  segment_delta 为段相对位移（有向），progress 为当前回放已走过的相对位移。
 *  段长近零时返回 1（允许推进，交给距离兜底处理）。 */
static uint8 play_segment_reached(int32 progress, int32 segment_delta)
{
    if (segment_delta > 0)
    {
        return (uint8)(progress >= segment_delta);
    }
    if (segment_delta < 0)
    {
        return (uint8)(progress <= segment_delta);
    }
    /* 段长 0（idx==0 常见情形）：直接视为已到，外层 while 继续推进 */
    return 1U;
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
    g_play.start_enc_dist_mm  = rear_left_get_dist_mm();
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
    int32 cur_offset_mm;
    int32 tgt_offset_mm;
    int32 prev_offset_mm;
    int32 segment_delta_mm;
    int32 progress_mm;
    int32 abs_delta_mm;
    int32 dist_fallback_mm;
    uint8 reached_target;

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

    /* INS 位置距离（ins_ctrl / 调试 / 极短段兜底） */
    icm_ins_get_position(&cur_px, &cur_py);
    g_play.last_dist_m = play_dist2d(cur_px - tgt->px_m, cur_py - tgt->py_m);

    /* 当前累计回放里程（有向） */
    cur_offset_mm = rear_left_get_dist_mm() - g_play.start_enc_dist_mm;

    /* 距离兜底阈值（mm）— 只在极短段允许用 INS 距离补充到达判据，
     * 正常段必须靠有向里程达到段长，避免方向翻转瞬间的距离假到达 */
    dist_fallback_mm = (int32)(g_play.reach_threshold_m * 1000.0f * 0.5f);

    /* 有向分段推进：允许一个 tick 内跨多个点 */
    while (1)
    {
        prev_offset_mm   = play_prev_mileage_mm(g_play.current_target_idx);
        tgt_offset_mm    = play_point_mileage_mm(g_play.current_target_idx);
        segment_delta_mm = tgt_offset_mm - prev_offset_mm;
        progress_mm      = cur_offset_mm - prev_offset_mm;

        abs_delta_mm = (segment_delta_mm < 0) ? -segment_delta_mm : segment_delta_mm;

        reached_target = play_segment_reached(progress_mm, segment_delta_mm);

        /* 短段才允许距离兜底，防止长段在方向翻转时因瞬时贴近目标误判 */
        if ((0U == reached_target) &&
            (abs_delta_mm <= dist_fallback_mm) &&
            (g_play.last_dist_m >= 0.0f) &&
            (g_play.last_dist_m <= g_play.reach_threshold_m))
        {
            reached_target = 1U;
        }

        if (0U == reached_target)
        {
            break;
        }

        /* 已到当前目标；若还有下一个点就推进，否则终点停车 */
        if (g_play.current_target_idx >= (uint16)(g_play.total_points - 1U))
        {
            ins_playback_stop();
            return;
        }

        g_play.current_target_idx++;
        tgt = ins_record_get_point_ptr(g_play.current_target_idx);
        if (NULL == tgt)
        {
            ins_playback_stop();
            return;
        }
        g_play.last_dist_m = play_dist2d(cur_px - tgt->px_m, cur_py - tgt->py_m);
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
