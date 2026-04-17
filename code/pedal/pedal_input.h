/*********************************************************************************************************************
* File: pedal_input.h
* Brief: 踏板 ADC 采样模块
*        第一步：A45/A47 原始值读取（两路都采，供调试对比）
*        第二步：油门处理管线——现在的油门实测插在 A47
*                （filtered / percent / pressed），标定:
*                  THROTTLE_MIN=1000, THROTTLE_MAX=3000,
*                  PRESS_ON=1350, PRESS_OFF=1250
*                A45 现已不接油门，仅保留 raw 供观察
*                刹车：当前硬件未提供可用通道，brake_available 固定 0
*        第三步：控制层接口——throttle_cmd / throttle_valid / throttle_enable_request
*                本层只整理命令量，不直接驱动任何执行器
* Author: JX116
*********************************************************************************************************************/

#ifndef __PEDAL_INPUT_H__
#define __PEDAL_INPUT_H__

#include "zf_common_typedef.h"

/* ---- 初始化 / 任务 ------------------------------------------------------ */
void   pedal_input_init(void);
void   pedal_input_task(void);

/* ---- Raw（两通道都保留，用于调试对比） -------------------------------- */
uint16 pedal_input_get_a45(void);
uint16 pedal_input_get_a47(void);
uint16 pedal_input_get_a24(void);

/* ---- 油门通道（当前物理源 = A47） -------------------------------------- */
/* 经 adc 均值 + 一阶 IIR 后的滤波值（0~4095） */
uint16 pedal_input_get_throttle_filtered(void);
/* 线性映射到 0~1000 的百分比（含裁剪） */
uint16 pedal_input_get_throttle_percent(void);
/* 带回差的 pressed 状态：0 = 未踩下，1 = 已踩下 */
uint8  pedal_input_get_throttle_pressed(void);

/* ---- 刹车通道 ---------------------------------------------------------- */
/*
 * 当前硬件未提供可用的刹车输入通道，本函数固定返回 0。
 * 上层任何刹车相关逻辑都应据此跳过，不要接入控制回路。
 */
/* A24 brake input is available; active brake only releases throttle. */
uint8  pedal_input_is_brake_available(void);
uint16 pedal_input_get_brake_filtered(void);
uint16 pedal_input_get_brake_percent(void);
uint8  pedal_input_get_brake_pressed(void);
uint8  pedal_input_get_brake_valid(void);
uint8  pedal_input_get_brake_enable_request(void);
uint16 pedal_input_get_brake_cmd(void);

/* ---- 控制层接口层（Step 3） --------------------------------------------
 * 提供“上层控制可安全读取的标准命令量”：
 *   - throttle_valid           : 输入通道是否有效（范围合理 + 已完成首次采样）
 *   - throttle_enable_request  : 驾驶员是否明确请求出力（必须 valid 才可能为 1）
 *   - throttle_cmd             : 带死区 + 再拉伸的油门命令，0~1000
 *
 * 重要：本层只产出命令量，绝对不直接控制电机/舵机/执行器。
 *       上层控制器应当同时看 valid / enable / cmd 后再决定是否下发。
 * ---------------------------------------------------------------------- */
uint8  pedal_input_get_throttle_valid(void);
uint8  pedal_input_get_throttle_enable_request(void);
uint16 pedal_input_get_throttle_cmd(void);      /* 0~1000 */

/* ---- 驱动控制参数（菜单设置，影响 THR 发送到 TC264 的门控和限幅） ---- */
void   pedal_input_set_drive_enable(uint8 en);  /* 0=禁止驱动 1=允许 */
uint8  pedal_input_get_drive_enable(void);
void   pedal_input_set_pwm_limit(uint16 limit); /* 0~1000, PWM占空比上限 */
uint16 pedal_input_get_pwm_limit(void);

/* ---- 方向按钮（P20.7，按一次切换前进/倒车） ---- */
uint8  pedal_input_get_direction(void);          /* 0=前进, 1=倒车 */

#endif /* __PEDAL_INPUT_H__ */
