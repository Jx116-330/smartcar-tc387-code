/**
 * @file ins_record.h
 * @brief INS 轨迹记录模块
 * @author JX116
 * @date 2026-04-07
 *
 * 基于已有 icm_ins 输出（位置、速度、静止状态、yaw），提供：
 *  - 固定周期轨迹记录（默认 100 ms）
 *  - 最小位移过滤（默认 0.05 m）
 *  - recording / idle 状态管理
 *  - 为后续轨迹复现预留的最小读取接口
 */

#ifndef __INS_RECORD_H__
#define __INS_RECORD_H__

#include "zf_common_typedef.h"

/* ------------------------------------------------------------------ */
/* 容量配置                                                             */
/* ------------------------------------------------------------------ */
#define INS_RECORD_MAX_POINTS       300U    /* 最多存储点数               */
#define INS_RECORD_INTERVAL_MS      100U    /* 默认采样周期 ms             */
#define INS_RECORD_MIN_DIST_M       0.05f   /* 最小位移门限 m（0 = 仅按时间）*/

/* ------------------------------------------------------------------ */
/* 数据类型                                                             */
/* ------------------------------------------------------------------ */
typedef enum
{
    INS_REC_IDLE = 0,
    INS_REC_RECORDING
} ins_rec_state_t;

/** 单条轨迹点，数据来自 icm_ins + icm_attitude */
typedef struct
{
    uint32 t_ms;        /**< 系统时间戳，ms                    */
    float  px_m;        /**< 导航系 X 位置，m                  */
    float  py_m;        /**< 导航系 Y 位置，m                  */
    float  vx_ms;       /**< 导航系 X 速度，m/s                */
    float  vy_ms;       /**< 导航系 Y 速度，m/s                */
    float  yaw_deg;     /**< 偏航角，°                         */
    uint8  stationary;  /**< 静止标志（ZUPT 输出）              */
} ins_record_point_t;

/** 轨迹记录运行时数据（可按需 extern 暴露给 display 等模块） */
typedef struct
{
    ins_record_point_t points[INS_RECORD_MAX_POINTS];
    uint16         point_count;
    ins_rec_state_t state;
    uint32         last_record_ms;
    float          last_px_m;
    float          last_py_m;
    uint32         interval_ms;     /**< 采样周期，可运行时修改 */
    float          min_dist_m;      /**< 最小位移门限，可运行时修改 */
} ins_record_data_t;

extern ins_record_data_t ins_record_data;

/* ------------------------------------------------------------------ */
/* 接口                                                                 */
/* ------------------------------------------------------------------ */

/** 初始化，上电后调用一次 */
void ins_record_init(void);

/** 开始记录；若已在 RECORDING 状态则忽略 */
void ins_record_start(void);

/** 停止记录（保留已记录点） */
void ins_record_stop(void);

/** 清空所有轨迹点并回到 IDLE */
void ins_record_clear(void);

/**
 * 主循环周期调用（建议每 ms 或每 10 ms 调一次）。
 * 内部自行管理时间窗口，不会每次都写点。
 */
void ins_record_task(void);

/** 是否正在记录 */
uint8 ins_record_is_recording(void);

/** 已记录点数 */
uint16 ins_record_get_point_count(void);

/** 获取指定索引的点（index 越界返回 0）。
 *  @param index  0 … point_count-1
 *  @param out    输出，不可为 NULL
 *  @return 1=成功，0=失败
 */
uint8 ins_record_get_point(uint16 index, ins_record_point_t *out);

/** 获取指定索引的指针（只读，NULL 表示越界） */
const ins_record_point_t *ins_record_get_point_ptr(uint16 index);

/** 获取最后一个记录点的指针（NULL 表示无点） */
const ins_record_point_t *ins_record_get_last_point(void);

/** 获取终点（同 get_last_point，为 playback 预留语义） */
const ins_record_point_t *ins_record_get_endpoint(void);

/** 修改采样周期 ms（钳位到 [20, 2000]） */
void ins_record_set_interval_ms(uint32 interval_ms);

/** 修改最小位移门限 m（0 = 仅按时间，钳位到 [0, 5.0]） */
void ins_record_set_min_dist_m(float min_dist_m);

#endif /* __INS_RECORD_H__ */
