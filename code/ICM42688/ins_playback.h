/**
 * @file ins_playback.h
 * @brief INS 轨迹回放骨架
 * @author JX116
 * @date 2026-04-07
 *
 * 最小可用骨架：加载已记录轨迹 → 顺序推进目标点 → 提供当前目标给上层控制器。
 * 不含转向 / PID / 路径跟踪，只做"目标点管理"。
 */

#ifndef __INS_PLAYBACK_H__
#define __INS_PLAYBACK_H__

#include <ICM42688/ins_record.h>
#include "zf_common_typedef.h"

/* ------------------------------------------------------------------ */
/* 配置                                                                 */
/* ------------------------------------------------------------------ */
/** 默认到点切换距离阈值，m */
#define INS_PLAY_REACH_THRESHOLD_M  0.20f

/* ------------------------------------------------------------------ */
/* 类型                                                                 */
/* ------------------------------------------------------------------ */
typedef enum
{
    INS_PLAY_IDLE    = 0,   /**< 未加载轨迹，或已结束          */
    INS_PLAY_READY   = 1,   /**< 轨迹已加载，等待 start        */
    INS_PLAY_RUNNING = 2,   /**< 正在推进目标点                */
} ins_play_state_t;

/* ------------------------------------------------------------------ */
/* 接口                                                                 */
/* ------------------------------------------------------------------ */

/** 上电初始化，调用一次 */
void ins_playback_init(void);

/**
 * 从 ins_record 当前已记录轨迹加载快照。
 * 必须先 stop/record 完毕再 load。
 * @return 1=成功（有点可回放），0=失败（无轨迹）
 */
uint8 ins_playback_load(void);

/**
 * 开始回放（READY → RUNNING）。
 * 必须先调用 ins_playback_load() 成功。
 */
void ins_playback_start(void);

/** 停止回放，回到 IDLE */
void ins_playback_stop(void);

/**
 * 主循环周期调用。
 * 计算当前位置到目标点距离，小于阈值则推进到下一个目标点；
 * 到达最后一个点后自动停止。
 */
void ins_playback_task(void);

/** 是否正在回放 */
uint8 ins_playback_is_running(void);

/** 当前状态 */
ins_play_state_t ins_playback_get_state(void);

/** 当前目标点索引（RUNNING 时有效） */
uint16 ins_playback_get_target_idx(void);

/** 总点数快照（load 时确定） */
uint16 ins_playback_get_total_points(void);

/**
 * 获取当前目标点内容（拷贝到 out）。
 * @return 1=有效，0=无效（IDLE 或越界）
 */
uint8 ins_playback_get_target(ins_record_point_t *out);

/**
 * 获取上一次 task() 计算的当前位置到目标点距离，m。
 * 未 RUNNING 时返回 -1.0f。
 */
float ins_playback_get_dist_to_target(void);

/** 修改到点切换阈值 m（钳位到 [0.05, 5.0]） */
void ins_playback_set_reach_threshold_m(float m);

#endif /* __INS_PLAYBACK_H__ */
