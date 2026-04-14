/*********************************************************************************************************************
* File: pedal_input.c
* Brief: 踏板 ADC 采样模块
*        - A47：油门（实测当前实际插在 A47 上）
*                 均值 + 一阶 IIR 滤波、线性百分比、带回差 pressed
*                 方向：踩下时数值增大（约 1000 松 → 约 3000 踩到底）
*        - A45：历史上曾作为油门，现在改插 A47，A45 保留 raw 观察，不参与控制
*        - 刹车：当前硬件未提供可用通道，brake_available 固定 0
*        - 控制层接口：throttle_valid / enable_request / cmd
*                       （带死区 + 再拉伸，本模块不直接驱动任何执行器）
* Author: JX116
*********************************************************************************************************************/

#include "pedal_input.h"
#include "zf_driver_adc.h"
#include "zf_driver_gpio.h"

/* ==== 采样与滤波参数 ================================================== */
#define PEDAL_FILTER_COUNT      4U          /* adc_mean_filter_convert 样本数 */

/*
 * 一阶 IIR：filt = filt*(1 - 1/4) + raw*(1/4)
 * 用位移实现，避免浮点；时间常数约等于 4 个 task 周期
 */
#define PEDAL_IIR_SHIFT         2U

/* ==== A47 油门标定（临时）============================================ */
/*
 * 当前现场实测（油门实际插在 A47）：
 *   - 松开约 1000
 *   - 踩到底约 4000
 *   - 数值随踩下增大（方向与之前 A45 一致）
 * 后续若换踏板或重新标定，只改这几个宏即可
 */
#define THROTTLE_MIN            1000U
#define THROTTLE_MAX            4000U

/* 带回差的 pressed 判定 */
#define THROTTLE_PRESS_ON       1350U    /* 贴近 MIN 之上，较早触发 pressed */
#define THROTTLE_PRESS_OFF      1250U    /* 回差 100 counts */

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
static uint16 pedal_a45_raw      = 0U;   /* 仅作观察：A45 当前没接油门，保留采样便于调试 */
static uint16 pedal_a47_raw      = 0U;   /* 当前油门原始值：经控制管线后产出 filtered/percent/pressed */

static uint16 pedal_thr_filtered = 0U;
static uint16 pedal_thr_percent  = 0U;   /* 0~1000 */
static uint8  pedal_thr_pressed  = 0U;
static uint8  pedal_thr_filt_inited = 0U;

/* 控制层输出 */
static uint8  pedal_throttle_valid       = 0U;
static uint8  pedal_throttle_enable_req  = 0U;
static uint16 pedal_throttle_cmd         = 0U;   /* 0~1000 */

/* 驱动控制参数（菜单设置） */
static uint8  pedal_drive_enable         = 0U;    /* 总开关: 0=禁止驱动, 1=允许驱动 */
static uint16 pedal_pwm_limit            = 300U;  /* PWM上限: 0~1000, 默认30%安全限 */

/* 方向按钮 P20.7（上拉，按下=低电平） */
#define DIR_BTN_PIN             (P20_7)
#define DIR_BTN_DEBOUNCE_COUNT  3U      /* 3 次连续一致 = 30ms（10ms task 周期） */
static uint8  dir_btn_direction    = 0U;   /* 0=前进 1=倒车 */
static uint8  dir_btn_last_stable  = 1U;   /* 上次稳定电平（1=未按） */
static uint8  dir_btn_reading      = 1U;   /* 当前读数 */
static uint8  dir_btn_count        = 0U;   /* 去抖计数 */

/* ==== 内部工具 ======================================================== */
static uint16 throttle_calc_percent(uint16 filtered)
{
    if (filtered <= THROTTLE_MIN)
    {
        return 0U;
    }
    if (filtered >= THROTTLE_MAX)
    {
        return 1000U;
    }
    /* (filtered - MIN) * 1000 / (MAX - MIN)，不需要浮点 */
    return (uint16)(((uint32)(filtered - THROTTLE_MIN) * 1000U)
                    / (uint32)(THROTTLE_MAX - THROTTLE_MIN));
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
    /* P20_7 已由 MyKey 模块初始化，硬件有外部上拉+去抖电容，这里不重复 gpio_init */

    pedal_a45_raw        = 0U;
    pedal_a47_raw        = 0U;
    pedal_thr_filtered   = 0U;
    pedal_thr_percent    = 0U;
    pedal_thr_pressed    = 0U;
    pedal_thr_filt_inited = 0U;

    pedal_throttle_valid      = 0U;
    pedal_throttle_enable_req = 0U;
    pedal_throttle_cmd        = 0U;

    dir_btn_direction   = 0U;
    dir_btn_last_stable = 1U;
    dir_btn_count       = 0U;
}

void pedal_input_task(void)
{
    uint16 a45_sample;
    uint16 a47_sample;

    /* 第一层：硬件均值滤波（两路都采） */
    a45_sample = adc_mean_filter_convert(ADC8_CH13_A45, PEDAL_FILTER_COUNT);
    a47_sample = adc_mean_filter_convert(ADC8_CH15_A47, PEDAL_FILTER_COUNT);
    pedal_a45_raw = a45_sample;
    pedal_a47_raw = a47_sample;

    /* 自动选源：哪路值大用哪路（油门接哪个通道就哪个能到高值） */
    {
        uint16 active_sample = (a47_sample >= a45_sample) ? a47_sample : a45_sample;

        /* 第二层：一阶 IIR */
        if (0U == pedal_thr_filt_inited)
        {
            pedal_thr_filtered    = active_sample;
            pedal_thr_filt_inited = 1U;
        }
        else
        {
            int32 diff = (int32)active_sample - (int32)pedal_thr_filtered;
            pedal_thr_filtered = (uint16)((int32)pedal_thr_filtered
                                          + (diff >> PEDAL_IIR_SHIFT));
        }
    }

    /* 第三层：映射 + 带回差 pressed 判定（基于 A47 处理结果） */
    pedal_thr_percent = throttle_calc_percent(pedal_thr_filtered)   ;
    pedal_thr_pressed = throttle_update_pressed(pedal_thr_pressed,
                                                pedal_thr_filtered);

    /* 第四层：控制层命令量（本函数是唯一的写入点） */
    pedal_throttle_valid = throttle_check_valid(pedal_thr_filtered,
                                                pedal_thr_filt_inited);
    pedal_throttle_enable_req = throttle_check_enable(pedal_throttle_valid,
                                                      pedal_thr_pressed);
    pedal_throttle_cmd = throttle_calc_command(pedal_thr_percent,
                                               pedal_throttle_valid);

    /* 方向按钮去抖：每 10ms 读一次，连续 3 次一致才更新稳定值 */
    {
        uint8 cur = gpio_get_level(DIR_BTN_PIN) ? 1U : 0U;
        if (cur == dir_btn_reading)
        {
            if (dir_btn_count < DIR_BTN_DEBOUNCE_COUNT)
                dir_btn_count++;
        }
        else
        {
            dir_btn_reading = cur;
            dir_btn_count   = 0U;
        }

        if (dir_btn_count >= DIR_BTN_DEBOUNCE_COUNT && cur != dir_btn_last_stable)
        {
            dir_btn_last_stable = cur;
            /* 下降沿（按下瞬间）触发切换 */
            if (0U == cur)
            {
                dir_btn_direction = dir_btn_direction ? 0U : 1U;
            }
        }
    }
}

/* ---- Raw 访问 ---------------------------------------------------------- */
uint16 pedal_input_get_a45(void) { return pedal_a45_raw; }
uint16 pedal_input_get_a47(void) { return pedal_a47_raw; }

/* ---- 油门（当前来自 A47 处理管线） ------------------------------------ */
uint16 pedal_input_get_throttle_filtered(void) { return pedal_thr_filtered; }
uint16 pedal_input_get_throttle_percent(void)  { return pedal_thr_percent; }
uint8  pedal_input_get_throttle_pressed(void)  { return pedal_thr_pressed; }

/* ---- 刹车 ------------------------------------------------------------- */
/*
 * 当前硬件未提供可用的刹车输入通道。
 * 此函数固定返回 0，上层任何刹车相关逻辑都应据此跳过。
 */
uint8 pedal_input_is_brake_available(void) { return 0U; }

/* ---- 控制层接口 -------------------------------------------------------- */
uint8  pedal_input_get_throttle_valid(void)          { return pedal_throttle_valid; }
uint8  pedal_input_get_throttle_enable_request(void) { return pedal_throttle_enable_req; }
uint16 pedal_input_get_throttle_cmd(void)            { return pedal_throttle_cmd; }

/* ---- 驱动控制参数（菜单设置） ------------------------------------------ */
void   pedal_input_set_drive_enable(uint8 en)  { pedal_drive_enable = (0U != en) ? 1U : 0U; }
uint8  pedal_input_get_drive_enable(void)      { return pedal_drive_enable; }
void   pedal_input_set_pwm_limit(uint16 limit) { pedal_pwm_limit = (limit > 1000U) ? 1000U : limit; }
uint16 pedal_input_get_pwm_limit(void)         { return pedal_pwm_limit; }

/* ---- 方向按钮 ------------------------------------------------------------ */
uint8  pedal_input_get_direction(void)         { return dir_btn_direction; }
