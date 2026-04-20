/**
 * @file ins_ctrl.h
 * @brief INS 轨迹复现控制骨架
 * @author JX116
 * @date 2026-04-07
 *
 * 最小可用复现控制骨架：
 *   - 从 ins_playback 获取当前目标点
 *   - 从 icm_ins / icm_attitude 获取当前状态
 *   - 计算基础几何量（dist, target_yaw, yaw_err）
 *   - 输出最简控制建议量（drive_enable, speed_cmd）
 *
 * 不含 Stanley / Pure Pursuit / PID 等完整路径跟踪算法。
 * 上层控制链通过 ins_ctrl_get_output() 读取结果后自行驱动电机/舵机。
 */

#ifndef __INS_CTRL_H__
#define __INS_CTRL_H__

#include "zf_common_typedef.h"

/* ------------------------------------------------------------------ */
/* 配置                                                                 */
/* ------------------------------------------------------------------ */
/** 满速巡航距离阈值，m：距目标点超过此值时使用 full speed */
#define INS_CTRL_CRUISE_DIST_M      0.60f

/** 接近目标点时的减速速度（比例因子相对 full speed），巡航距离以内时使用 */
#define INS_CTRL_SLOW_SPEED_RATIO   0.50f

/** 满速巡航参考速度，m/s（供上层参考，实际由下层电机控制换算） */
#define INS_CTRL_FULL_SPEED_MS      0.50f

/* ------------------------------------------------------------------ */
/* 数据类型                                                             */
/* ------------------------------------------------------------------ */

/**
 * 控制骨架输出量，每次 ins_ctrl_task() 后更新。
 * 上层控制链读取此结构并换算为实际电机/舵机指令。
 */
typedef struct
{
    /* ----- 几何量（调试优先） ----- */
    float    tgt_dist_m;      /**< 当前位置到目标点的直线距离，m；未运行时 = -1 */
    float    tgt_yaw_deg;     /**< 从当前位置指向目标点的方位角，°（atan2），未运行时 = 0 */
    float    yaw_err_deg;     /**< 目标航向误差 = tgt_yaw - cur_yaw，归一化到 [-180, 180] */

    /* ----- 控制建议量 ----- */
    float    speed_cmd;       /**< 建议前进速度，m/s；drive_enable=0 时强制为 0 */
    uint8    drive_enable;    /**< 1=允许推进，0=停止（playback 未运行或目标无效） */

    /* ----- 当前状态快照（供上层显示/逻辑使用） ----- */
    float    cur_px_m;        /**< 当前 X 位置，m */
    float    cur_py_m;        /**< 当前 Y 位置，m */
    float    cur_yaw_deg;     /**< 当前 yaw，° */
    float    tgt_px_m;        /**< 目标点 X，m（未运行时 = 0） */
    float    tgt_py_m;        /**< 目标点 Y，m（未运行时 = 0） */
} ins_ctrl_output_t;

/* ------------------------------------------------------------------ */
/* 接口                                                                 */
/* ------------------------------------------------------------------ */

/** 上电初始化，调用一次 */
void ins_ctrl_init(void);

/**
 * 主循环周期调用。
 * 内部从 ins_playback / icm_ins / icm_attitude 取数，
 * 计算几何量并更新控制建议量。
 */
void ins_ctrl_task(void);

/**
 * 获取最新输出量（只读指针）。
 * 返回值永不为 NULL。
 */
const ins_ctrl_output_t *ins_ctrl_get_output(void);

#endif /* __INS_CTRL_H__ */
