/**
 * @file ins_record.c
 * @brief INS 轨迹记录模块实现
 * @author JX116
 * @date 2026-04-07
 *
 * 存点策略（航向自适应）：
 *   满足以下任一条件即存点（前提：位移 ≥ DIST_MIN 且非静止）
 *   1. 航向变化 ≥ YAW_THRESH_DEG  → 弯道密采
 *   2. 距离 ≥ DIST_MAX_M          → 直线兜底
 *   3. 时间 ≥ TIME_MAX_MS         → 极低速兜底
 *
 * 数据来源：icm_ins（位置/速度/静止）+ icm_attitude（yaw）
 */

#include "ins_record.h"
#include "icm_ins.h"
#include "icm_attitude.h"
#include "board_comm.h"
#include "zf_common_headfile.h"

#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* 运行时数据                                                           */
/* ------------------------------------------------------------------ */
ins_record_data_t ins_record_data;

/* ------------------------------------------------------------------ */
/* 内部辅助                                                             */
/* ------------------------------------------------------------------ */
static void ins_record_reset_runtime(void)
{
    memset(&ins_record_data, 0, sizeof(ins_record_data));
    ins_record_data.state = INS_REC_IDLE;
}

static float ins_record_dist2d(float dx, float dy)
{
    return sqrtf(dx * dx + dy * dy);
}

/** 计算两个角度之差的绝对值，归一化到 [0, 180] */
static float ins_record_yaw_delta(float a_deg, float b_deg)
{
    float d = a_deg - b_deg;
    while (d >  180.0f) { d -= 360.0f; }
    while (d < -180.0f) { d += 360.0f; }
    return (d < 0.0f) ? -d : d;
}

static uint8 ins_record_add_point(float px, float py, float yaw)
{
    ins_record_point_t *pt;
    float vx = 0.0f;
    float vy = 0.0f;

    if (ins_record_data.point_count >= INS_RECORD_MAX_POINTS)
    {
        return 0U;
    }

    icm_ins_get_velocity(&vx, &vy, NULL);

    pt = &ins_record_data.points[ins_record_data.point_count];
    pt->t_ms         = system_getval_ms();
    pt->px_m         = px;
    pt->py_m         = py;
    pt->vx_ms        = vx;
    pt->vy_ms        = vy;
    pt->yaw_deg      = yaw;
    pt->enc_spd_mm_s = board_comm_encl_get_spd_mm_s();
    pt->enc_dist_mm  = board_comm_encl_get_dist_mm();

    ins_record_data.point_count++;

    /* 更新"上次记录"状态 */
    ins_record_data.last_px_m      = px;
    ins_record_data.last_py_m      = py;
    ins_record_data.last_yaw_deg   = yaw;
    ins_record_data.last_record_ms = pt->t_ms;

    return 1U;
}

/* ------------------------------------------------------------------ */
/* 公开接口                                                             */
/* ------------------------------------------------------------------ */
void ins_record_init(void)
{
    ins_record_reset_runtime();
}

void ins_record_start(void)
{
    float px = 0.0f;
    float py = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    if (INS_REC_RECORDING == ins_record_data.state)
    {
        return;
    }

    icm_ins_get_position(&px, &py);
    icm_attitude_get_euler(&roll, &pitch, &yaw);

    ins_record_data.state          = INS_REC_RECORDING;
    ins_record_data.last_record_ms = system_getval_ms();
    ins_record_data.last_px_m      = px;
    ins_record_data.last_py_m      = py;
    ins_record_data.last_yaw_deg   = yaw;

    /* 立即存第一个点（起点），不等触发条件 */
    ins_record_add_point(px, py, yaw);
}

void ins_record_smooth(void)
{
    uint16 count = ins_record_data.point_count;
    uint16 pass;
    uint16 i;

    if (count < 3U)
    {
        return;
    }

    for (pass = 0U; pass < INS_RECORD_SMOOTH_PASSES; pass++)
    {
        for (i = 1U; i < (count - 1U); i++)
        {
            ins_record_point_t *prev = &ins_record_data.points[i - 1U];
            ins_record_point_t *cur  = &ins_record_data.points[i];
            ins_record_point_t *next = &ins_record_data.points[i + 1U];

            /* 只平滑弯道段：前后点的 yaw 差异超过阈值 */
            float dyaw = ins_record_yaw_delta(prev->yaw_deg, next->yaw_deg);
            if (dyaw < INS_RECORD_SMOOTH_YAW_THRESH)
            {
                continue;   /* 直道段，不动 */
            }

            /* 3 点加权平均：0.25 * prev + 0.5 * cur + 0.25 * next */
            cur->px_m = 0.25f * prev->px_m + 0.5f * cur->px_m + 0.25f * next->px_m;
            cur->py_m = 0.25f * prev->py_m + 0.5f * cur->py_m + 0.25f * next->py_m;
        }
    }
}

void ins_record_stop(void)
{
    ins_record_data.state = INS_REC_IDLE;

    /* 停止录制时自动平滑弯道 */
    ins_record_smooth();
}

void ins_record_clear(void)
{
    ins_record_reset_runtime();
}

void ins_record_task(void)
{
    uint32 now_ms;
    uint32 dt_ms;
    float  px = 0.0f;
    float  py = 0.0f;
    float  roll = 0.0f;
    float  pitch = 0.0f;
    float  yaw = 0.0f;
    float  dist;
    float  dyaw;
    uint8  should_record;

    if (INS_REC_RECORDING != ins_record_data.state)
    {
        return;
    }

    now_ms = system_getval_ms();

    /* 最小调用间隔，节省 CPU */
    if ((uint32)(now_ms - ins_record_data.last_record_ms) < INS_RECORD_POLL_MS)
    {
        return;
    }

    /* 静止时跳过 */
    if (icm_ins_is_stationary())
    {
        return;
    }

    /* 采集当前状态 */
    icm_ins_get_position(&px, &py);
    icm_attitude_get_euler(&roll, &pitch, &yaw);

    dist  = ins_record_dist2d(px - ins_record_data.last_px_m,
                               py - ins_record_data.last_py_m);
    dyaw  = ins_record_yaw_delta(yaw, ins_record_data.last_yaw_deg);
    dt_ms = (uint32)(now_ms - ins_record_data.last_record_ms);

    /* 最小位移门限：防止静止抖动堆点 */
    if (dist < INS_RECORD_DIST_MIN_M)
    {
        return;
    }

    /* 弯道/直道自动判别：用 yaw 变化率决定距离阈值 */
    {
        float yaw_rate_dps = (dt_ms > 0U) ? (dyaw * 1000.0f / (float)dt_ms) : 0.0f;
        float dist_thresh  = (yaw_rate_dps > INS_RECORD_TURN_RATE_THRESH)
                             ? INS_RECORD_DIST_TURN_M    /* 弯道：10cm 密采 */
                             : INS_RECORD_DIST_MAX_M;    /* 直道：50cm 稀采 */

        /* 判断是否需要存点 */
        should_record = 0U;

        /* 条件 1: 航向变化 ≥ 阈值（弯道密采） */
        if (dyaw >= INS_RECORD_YAW_THRESH_DEG)
        {
            should_record = 1U;
        }
        /* 条件 2: 距离 ≥ 动态阈值（弯道/直道自适应） */
        else if (dist >= dist_thresh)
        {
            should_record = 1U;
        }
        /* 条件 3: 时间 ≥ 上限（极低速兜底） */
        else if (dt_ms >= INS_RECORD_TIME_MAX_MS)
        {
            should_record = 1U;
        }
    }

    if (!should_record)
    {
        return;
    }

    if (!ins_record_add_point(px, py, yaw))
    {
        /* 缓冲区满，自动停止 */
        ins_record_stop();
    }
}

uint8 ins_record_is_recording(void)
{
    return (INS_REC_RECORDING == ins_record_data.state) ? 1U : 0U;
}

uint16 ins_record_get_point_count(void)
{
    return ins_record_data.point_count;
}

uint8 ins_record_get_point(uint16 index, ins_record_point_t *out)
{
    if ((NULL == out) || (index >= ins_record_data.point_count))
    {
        return 0U;
    }

    *out = ins_record_data.points[index];
    return 1U;
}

const ins_record_point_t *ins_record_get_point_ptr(uint16 index)
{
    if (index >= ins_record_data.point_count)
    {
        return NULL;
    }

    return &ins_record_data.points[index];
}

const ins_record_point_t *ins_record_get_last_point(void)
{
    if (0U == ins_record_data.point_count)
    {
        return NULL;
    }

    return &ins_record_data.points[ins_record_data.point_count - 1U];
}

const ins_record_point_t *ins_record_get_endpoint(void)
{
    return ins_record_get_last_point();
}
