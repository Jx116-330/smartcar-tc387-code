/*********************************************************************************************************************
* File: pedal_input.h
* Brief: 踏板 ADC 采样模块
*        第一步：A45/A47 原始值读取
*        第二步：基于当前板级硬件实际情况，仅实现 A45 油门识别
*                （filtered / percent / pressed）
*                刹车（A47）受当前板级 ADC 复用问题影响，暂不可用
*        第三步：新增“控制层接口层”，给上层控制提供
*                throttle_cmd / throttle_valid / throttle_enable_request
*                —— 本层只整理命令量，不直接驱动任何执行器
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

/* ---- A45 油门通道 ------------------------------------------------------ */
/* 经 adc 均值 + 一阶 IIR 后的滤波值（0~4095） */
uint16 pedal_input_get_throttle_filtered(void);
/* 线性映射到 0~1000 的百分比（含裁剪） */
uint16 pedal_input_get_throttle_percent(void);
/* 带回差的 pressed 状态：0 = 未踩下，1 = 已踩下 */
uint8  pedal_input_get_throttle_pressed(void);

/* ---- A47 刹车通道 ------------------------------------------------------ */
/*
 * 受当前板级 ADC 复用问题影响，A47 刹车输入暂不可用。
 * 这里保留原始采样用于现场观察硬件表现，但不提供
 * filtered / percent / pressed 接口，避免上层误用。
 * 如果后续板子修好，再在这里补齐刹车接口。
 */
uint8  pedal_input_is_brake_available(void);  /* 固定返回 0，后续可扩展 */

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

#endif /* __PEDAL_INPUT_H__ */
