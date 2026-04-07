/**
 * @file ins_record.c
 * @brief INS 轨迹记录模块实现
 * @author JX116
 * @date 2026-04-07
 *
 * 数据来源：icm_ins（位置/速度/静止）+ icm_attitude（yaw）
 * 设计风格参考 path_recorder.c，不重复实现 INS 核心。
 */

#include "ins_record.h"
#include "icm_ins.h"
#include "icm_attitude.h"
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
    ins_record_data.state       = INS_REC_IDLE;
    ins_record_data.interval_ms = INS_RECORD_INTERVAL_MS;
    ins_record_data.min_dist_m  = INS_RECORD_MIN_DIST_M;
}

static float ins_record_dist2d(float dx, float dy)
{
    return sqrtf(dx * dx + dy * dy);
}

static uint8 ins_record_add_point(void)
{
    ins_record_point_t *pt;
    float px = 0.0f;
    float py = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;

    if (ins_record_data.point_count >= INS_RECORD_MAX_POINTS)
    {
        return 0U;
    }

    icm_ins_get_position(&px, &py);
    icm_ins_get_velocity(&vx, &vy, NULL);
    icm_attitude_get_euler(&roll, &pitch, &yaw);

    pt = &ins_record_data.points[ins_record_data.point_count];
    pt->t_ms       = system_getval_ms();
    pt->px_m       = px;
    pt->py_m       = py;
    pt->vx_ms      = vx;
    pt->vy_ms      = vy;
    pt->yaw_deg    = yaw;
    pt->stationary = icm_ins_is_stationary();

    ins_record_data.point_count++;

    /* 更新"上次记录"位置 */
    ins_record_data.last_px_m      = px;
    ins_record_data.last_py_m      = py;
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
    if (INS_REC_RECORDING == ins_record_data.state)
    {
        return;
    }

    ins_record_data.state         = INS_REC_RECORDING;
    ins_record_data.last_record_ms = system_getval_ms();

    /* 以当前位置为起点 */
    icm_ins_get_position(&ins_record_data.last_px_m, &ins_record_data.last_py_m);
}

void ins_record_stop(void)
{
    ins_record_data.state = INS_REC_IDLE;
}

void ins_record_clear(void)
{
    ins_record_reset_runtime();
}

void ins_record_task(void)
{
    uint32 now_ms;
    float  px = 0.0f;
    float  py = 0.0f;
    float  dist = 0.0f;

    if (INS_REC_RECORDING != ins_record_data.state)
    {
        return;
    }

    now_ms = system_getval_ms();

    /* 时间门限 */
    if ((uint32)(now_ms - ins_record_data.last_record_ms) < ins_record_data.interval_ms)
    {
        return;
    }

    /* 静止时跳过：ZUPT 已将速度清零，位置不变，无需写重复点 */
    if (icm_ins_is_stationary())
    {
        ins_record_data.last_record_ms = now_ms;
        return;
    }

    /* 位移门限（仅当 min_dist_m > 0 且已有至少一个点时生效） */
    if ((ins_record_data.min_dist_m > 0.0f) && (ins_record_data.point_count > 0U))
    {
        icm_ins_get_position(&px, &py);
        dist = ins_record_dist2d(px - ins_record_data.last_px_m,
                                  py - ins_record_data.last_py_m);
        if (dist < ins_record_data.min_dist_m)
        {
            /* 时间窗口已过，但位移不够：更新时间戳避免连续跳过导致统计混乱 */
            ins_record_data.last_record_ms = now_ms;
            return;
        }
    }

    if (!ins_record_add_point())
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

void ins_record_set_interval_ms(uint32 interval_ms)
{
    if (interval_ms < 20U)
    {
        interval_ms = 20U;
    }
    else if (interval_ms > 2000U)
    {
        interval_ms = 2000U;
    }

    ins_record_data.interval_ms = interval_ms;
}

void ins_record_set_min_dist_m(float min_dist_m)
{
    if (min_dist_m < 0.0f)
    {
        min_dist_m = 0.0f;
    }
    else if (min_dist_m > 5.0f)
    {
        min_dist_m = 5.0f;
    }

    ins_record_data.min_dist_m = min_dist_m;
}
