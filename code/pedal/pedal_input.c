/*********************************************************************************************************************
* File: pedal_input.c
* Brief: 踏板 ADC 采样模块
*        - A45：油门（均值 + 一阶 IIR 滤波、线性百分比、带回差 pressed）
*        - A47：刹车（当前板级 ADC 复用问题，暂不可用，仅保留 raw 观察）
*        - Step 3：新增控制层接口 throttle_valid / enable_request / cmd
*                  （带死区 + 再拉伸，本模块不直接驱动任何执行器）
* Author: JX116
*********************************************************************************************************************/

#include "pedal_input.h"
#include "zf_driver_adc.h"

/* ==== 采样与滤波参数 ================================================== */
#define PEDAL_FILTER_COUNT      4U          /* adc_mean_filter_convert 样本数 */

/*
 * 一阶 IIR：filt = filt*(1 - 1/4) + raw*(1/4)
 * 用位移实现，避免浮点；时间常数约等于 4 个 task 周期
 */
#define PEDAL_IIR_SHIFT         2U

/* ==== A45 油门标定（临时）============================================ */
/*
 * 来自现场实测：
 *   - 松开约 1000
 *   - 踩到底约 4000+
 *   - 数值随踩下增大
 * 后续若换踏板再重新标定
 */
#define THROTTLE_A45_MIN        1000U
#define THROTTLE_A45_MAX        4000U

/* 带回差的 pressed 判定 */
#define THROTTLE_PRESS_ON       1500U
#define THROTTLE_PRESS_OFF      1350U

/* ==== 控制层参数 ====================================================== */
/*
 * 输入有效性判定：
 *   filtered 必须落在合理范围内，太靠近 0 或 4095 一般是断线/短路/浮空。
 */
#define THROTTLE_VALID_LOW      100U
#define THROTTLE_VALID_HIGH     4090U

/*
 * 死区：A45 percent（0~1000）小于等于这个值时，命令强制归零；
 * 剩余 (DEADZONE, 1000] 区间再线性拉伸到 0~1000。
 */
#define THROTTLE_CMD_DEADZONE   30U      /* 3.0% */
#define THROTTLE_CMD_FULL       1000U

/* ==== 内部状态 ======================================================== */
static uint16 pedal_a45_raw      = 0U;
static uint16 pedal_a47_raw      = 0U;   /* 仅用于原始观察，硬件问题下不可信 */

static uint16 pedal_a45_filtered = 0U;
static uint16 pedal_a45_percent  = 0U;   /* 0~1000 */
static uint8  pedal_a45_pressed  = 0U;
static uint8  pedal_a45_filt_inited = 0U;

/* 控制层输出 */
static uint8  pedal_throttle_valid       = 0U;
static uint8  pedal_throttle_enable_req  = 0U;
static uint16 pedal_throttle_cmd         = 0U;   /* 0~1000 */

/* ==== 内部工具 ======================================================== */
static uint16 throttle_calc_percent(uint16 filtered)
{
    if (filtered <= THROTTLE_A45_MIN)
    {
        return 0U;
    }
    if (filtered >= THROTTLE_A45_MAX)
    {
        return 1000U;
    }
    /* (filtered - MIN) * 1000 / (MAX - MIN)，不需要浮点 */
    return (uint16)(((uint32)(filtered - THROTTLE_A45_MIN) * 1000U)
                    / (uint32)(THROTTLE_A45_MAX - THROTTLE_A45_MIN));
}

static uint8 throttle_update_pressed(uint8 prev, uint16 filtered)
{
    if (0U == prev)
    {
        /* 当前未踩下：需要超过 ON 阈值才算踩下 */
        if (filtered >= THROTTLE_PRESS_ON)
        {
            return 1U;
        }
        return 0U;
    }
    /* 当前已踩下：需要低于 OFF 阈值才算松开（回差防抖） */
    if (filtered <= THROTTLE_PRESS_OFF)
    {
        return 0U;
    }
    return 1U;
}

/* ==== 控制层内部工具 ================================================== */

/* 输入通道是否有效：落在合理范围 + 已完成首次采样 */
static uint8 throttle_check_valid(uint16 filtered, uint8 inited)
{
    if (0U == inited)               { return 0U; }
    if (filtered < THROTTLE_VALID_LOW)  { return 0U; }
    if (filtered > THROTTLE_VALID_HIGH) { return 0U; }
    return 1U;
}

/*
 * enable_request：驾驶员是否明确请求出力
 *   - 必须 valid = 1
 *   - 必须 pressed = 1（pressed 本身带回差防抖，已过滤抖动）
 * 任意一条不满足都强制为 0
 */
static uint8 throttle_check_enable(uint8 valid, uint8 pressed)
{
    if (0U == valid)   { return 0U; }
    if (0U == pressed) { return 0U; }
    return 1U;
}

/*
 * 油门命令：基于 percent(0~1000)
 *   - 非 valid 时强制 0（安全兜底）
 *   - <= 死区：0
 *   - >  死区：把 (DEADZONE, 1000] 重新线性拉伸到 0~1000
 */
static uint16 throttle_calc_command(uint16 percent, uint8 valid)
{
    uint32 span;
    uint32 mapped;

    if (0U == valid)                   { return 0U; }
    if (percent <= THROTTLE_CMD_DEADZONE) { return 0U; }

    span   = (uint32)(THROTTLE_CMD_FULL - THROTTLE_CMD_DEADZONE);
    mapped = ((uint32)(percent - THROTTLE_CMD_DEADZONE) * THROTTLE_CMD_FULL) / span;

    if (mapped > THROTTLE_CMD_FULL) { mapped = THROTTLE_CMD_FULL; }
    return (uint16)mapped;
}

/* ==== 公共接口 ======================================================== */
void pedal_input_init(void)
{
    adc_init(ADC8_CH13_A45, ADC_12BIT);
    adc_init(ADC8_CH15_A47, ADC_12BIT);

    pedal_a45_raw        = 0U;
    pedal_a47_raw        = 0U;
    pedal_a45_filtered   = 0U;
    pedal_a45_percent    = 0U;
    pedal_a45_pressed    = 0U;
    pedal_a45_filt_inited = 0U;

    pedal_throttle_valid      = 0U;
    pedal_throttle_enable_req = 0U;
    pedal_throttle_cmd        = 0U;
}

void pedal_input_task(void)
{
    uint16 a45_sample;
    uint16 a47_sample;

    /* 第一层：硬件均值滤波 */
    a45_sample = adc_mean_filter_convert(ADC8_CH13_A45, PEDAL_FILTER_COUNT);
    a47_sample = adc_mean_filter_convert(ADC8_CH15_A47, PEDAL_FILTER_COUNT);

    pedal_a45_raw = a45_sample;
    /*
     * A47 raw 保留，仅给 debug 页观察；由于当前板级 ADC 复用问题，
     * 这里的数值不做任何滤波/映射/pressed 判定。
     */
    pedal_a47_raw = a47_sample;

    /* 第二层：一阶 IIR，首帧直接用 raw 初始化避免起步爬升 */
    if (0U == pedal_a45_filt_inited)
    {
        pedal_a45_filtered    = a45_sample;
        pedal_a45_filt_inited = 1U;
    }
    else
    {
        /* filt += (raw - filt) >> PEDAL_IIR_SHIFT  —— 等价于轻量 IIR */
        int32 diff = (int32)a45_sample - (int32)pedal_a45_filtered;
        pedal_a45_filtered = (uint16)((int32)pedal_a45_filtered
                                      + (diff >> PEDAL_IIR_SHIFT));
    }

    /* 第三层：映射 + 带回差 pressed 判定 */
    pedal_a45_percent = throttle_calc_percent(pedal_a45_filtered);
    pedal_a45_pressed = throttle_update_pressed(pedal_a45_pressed,
                                                pedal_a45_filtered);

    /* 第四层：控制层命令量（本函数是唯一的写入点） */
    pedal_throttle_valid = throttle_check_valid(pedal_a45_filtered,
                                                pedal_a45_filt_inited);
    pedal_throttle_enable_req = throttle_check_enable(pedal_throttle_valid,
                                                      pedal_a45_pressed);
    pedal_throttle_cmd = throttle_calc_command(pedal_a45_percent,
                                               pedal_throttle_valid);
}

/* ---- Raw 访问 ---------------------------------------------------------- */
uint16 pedal_input_get_a45(void) { return pedal_a45_raw; }
uint16 pedal_input_get_a47(void) { return pedal_a47_raw; }

/* ---- A45 油门 ---------------------------------------------------------- */
uint16 pedal_input_get_throttle_filtered(void) { return pedal_a45_filtered; }
uint16 pedal_input_get_throttle_percent(void)  { return pedal_a45_percent; }
uint8  pedal_input_get_throttle_pressed(void)  { return pedal_a45_pressed; }

/* ---- A47 刹车 ---------------------------------------------------------- */
/*
 * 注意：受当前板级硬件问题影响，刹车输入暂不可用。
 * 此函数固定返回 0，上层任何刹车相关逻辑都应据此跳过。
 */
uint8 pedal_input_is_brake_available(void) { return 0U; }

/* ---- 控制层接口 -------------------------------------------------------- */
uint8  pedal_input_get_throttle_valid(void)          { return pedal_throttle_valid; }
uint8  pedal_input_get_throttle_enable_request(void) { return pedal_throttle_enable_req; }
uint16 pedal_input_get_throttle_cmd(void)            { return pedal_throttle_cmd; }
