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
#define INS_RECORD_MAX_POINTS       2000U   /* 最多存储点数（~64KB RAM，总 RAM 84+64=148KB/240KB） */
#define INS_RECORD_POLL_MS          20U     /* task 最小调用间隔 ms        */
#define INS_RECORD_YAW_THRESH_DEG   3.0f    /* 航向变化触发阈值（度）       */
#define INS_RECORD_DIST_MAX_M       0.50f   /* 最大距离触发（直线每 50cm）   */
#define INS_RECORD_DIST_MIN_M       0.02f   /* 最小距离门限（防静止堆点）    */
#define INS_RECORD_TIME_MAX_MS      500U    /* 最大时间兜底 ms（低速兜底）   */

/* ------------------------------------------------------------------ */
/* 数据类型                                                             */
/* ------------------------------------------------------------------ */
typedef enum
{
    INS_REC_IDLE = 0,
    INS_REC_RECORDING
} ins_rec_state_t;

/** 单条轨迹点，数据来自 icm_ins + icm_attitude + encoder */
typedef struct
{
    uint32 t_ms;        /**< 系统时间戳，ms                    */
    float  px_m;        /**< 导航系 X 位置，m                  */
    float  py_m;        /**< 导航系 Y 位置，m                  */
    float  vx_ms;       /**< 导航系 X 速度，m/s                */
    float  vy_ms;       /**< 导航系 Y 速度，m/s                */
    float  yaw_deg;     /**< 偏航角，°                         */
    int32  enc_spd_mm_s;/**< 编码器速度 mm/s（回放速度参考）     */
    int32  enc_dist_mm; /**< 编码器累计距离 mm（段间距计算）     */
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
    float          last_yaw_deg;    /**< 上次记录时的航向角（度） */
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

#endif /* __INS_RECORD_H__ */
