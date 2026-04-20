/*********************************************************************************************************************
 * File: encoder_odom.h
 * Brief: 编码器里程计 → INS 融合模块
 *        读取本地 rear_left 编码器数据，以速度+位置双校正的方式融入 icm_ins。
 *        仅左后轮编码器，速度沿航向分解到 ENU 坐标系。
 *
 * 调用方式:
 *   - encoder_odom_init()  上电初始化
 *   - encoder_odom_task()  主循环中 menu_task() 之前调用
 *
 * Author: JX116
 *********************************************************************************************************************/

#ifndef __ENCODER_ODOM_H__
#define __ENCODER_ODOM_H__

#include "zf_common_typedef.h"

/* ---- 初始化 / 周期任务 ---- */
void  encoder_odom_init(void);
void  encoder_odom_task(void);

/* ---- 控制 ---- */
void  encoder_odom_reset(void);         /* 清零 DR 位置，下帧重新同步 INS */
void  encoder_odom_set_enable(uint8 en);/* 1=启用融合 0=仅采集不校正       */
uint8 encoder_odom_is_enabled(void);    /* 当前融合开关状态               */

/* ---- 只读 Getter（供菜单 / 调试） ---- */
float encoder_odom_get_px_m(void);      /* 编码器 DR 位置 X (ENU, m)   */
float encoder_odom_get_py_m(void);      /* 编码器 DR 位置 Y (ENU, m)   */
float encoder_odom_get_speed_ms(void);  /* 当前编码器速度 (m/s)        */
uint8 encoder_odom_is_active(void);     /* 正在融合 = 1                */

#endif /* __ENCODER_ODOM_H__ */
