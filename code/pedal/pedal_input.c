/*********************************************************************************************************************
* File: pedal_input.c
* Brief: 踏板 ADC 采样模块
*        - A47：油门（实测当前实际插在 A47 上）
*                 均值 + 一阶 IIR 滤波、线性百分比、带回差 pressed
*                 方向：踩下时数值增大（约 1000 松 → 约 3000 踩到底）
*        - A45：历史上曾作为油门，现在改插 A47，A45 保留 raw 观察，不参与控制
*        - 刹车（A47 -> A24）：读 A24 (ADC3_CH0_A24)，管线同油门，
*                 brake_pressed && valid 时强制油门命令归零（刹车优先）。
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
 * 一阶 IIR：filt = filt*(1 - 1/8) + raw*(1/8)
 * 用位移实现，避免浮点；时间常数约等于 8 个 task 周期
 */
#define PEDAL_IIR_SHIFT         3U

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
#define THROTTLE_CMD_RAMP_UP    50U      /* 每周期最多 +50，0->1000 约 200ms */
#define THROTTLE_CMD_RAMP_DOWN  200U     /* 松油门快速回落 */

#define REVERSE_THROTTLE_LIMIT  300U     /* 倒车时油门 cmd 上限 30% */

/* ==== A24 brake calibration, placeholder before vehicle calibration ==== */
#define BRAKE_MIN               1000U
#define BRAKE_MAX               4000U
#define BRAKE_PRESS_ON          1350U
#define BRAKE_PRESS_OFF         1250U
#define BRAKE_VALID_LOW         100U
#define BRAKE_VALID_HIGH        4090U
#define BRAKE_CMD_DEADZONE      30U
#define BRAKE_CMD_FULL          1000U
#define BRAKE_CMD_RAMP_UP       80U
#define BRAKE_CMD_RAMP_DOWN     200U

/* ==== 内部状态 ======================================================== */
static uint16 pedal_a45_raw      = 0U;   /* 仅作观察：A45 当前没接油门，保留采样便于调试 */
static uint16 pedal_a47_raw      = 0U;   /* 当前油门原始值：经控制管线后产出 filtered/percent/pressed */

static uint16 pedal_a24_raw      = 0U;   /* Brake pedal raw value */

static uint16 pedal_thr_filtered = 0U;
static uint16 pedal_thr_percent  = 0U;   /* 0~1000 */
static uint8  pedal_thr_pressed  = 0U;
static uint8  pedal_thr_filt_inited = 0U;

static uint16 pedal_brk_filtered = 0U;
static uint16 pedal_brk_percent  = 0U;   /* 0~1000 */
static uint8  pedal_brk_pressed  = 0U;
static uint8  pedal_brk_filt_inited = 0U;

/* 控制层输出 */
static uint8  pedal_throttle_valid       = 0U;
static uint8  pedal_throttle_enable_req  = 0U;
static uint16 pedal_throttle_cmd         = 0U;   /* 0~1000 */
static uint16 pedal_throttle_cmd_last    = 0U;   /* 上次输出的 cmd */

static uint8  pedal_brake_valid       = 0U;
static uint8  pedal_brake_enable_req  = 0U;
static uint16 pedal_brake_cmd         = 0U;
static uint16 pedal_brake_cmd_last    = 0U;

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
static uint16 throttle_calc_command_hys(uint16 percent, uint8 valid, uint16 last_cmd)
{
    uint32 span;
    uint32 mapped;
    uint16 deadzone;

    if (0U == valid)                   { return 0U; }
    deadzone = (0U != last_cmd) ? (THROTTLE_CMD_DEADZONE / 2U)
                                : THROTTLE_CMD_DEADZONE;
    if (percent <= deadzone) { return 0U; }

    span   = (uint32)(THROTTLE_CMD_FULL - deadzone);
    mapped = ((uint32)(percent - deadzone) * THROTTLE_CMD_FULL) / span;

    if (mapped > THROTTLE_CMD_FULL) { mapped = THROTTLE_CMD_FULL; }
    return (uint16)mapped;
}

static uint16 brake_calc_percent(uint16 filtered)
{
    if (filtered <= BRAKE_MIN)
    {
        return 0U;
    }
    if (filtered >= BRAKE_MAX)
    {
        return 1000U;
    }
    return (uint16)(((uint32)(filtered - BRAKE_MIN) * 1000U)
                    / (uint32)(BRAKE_MAX - BRAKE_MIN));
}

static uint8 brake_update_pressed(uint8 prev, uint16 filtered)
{
    if (0U == prev)
    {
        if (filtered >= BRAKE_PRESS_ON)
        {
            return 1U;
        }
        return 0U;
    }
    if (filtered <= BRAKE_PRESS_OFF)
    {
        return 0U;
    }
    return 1U;
}

static uint8 brake_check_valid(uint16 filtered, uint8 inited)
{
    if (0U == inited)             { return 0U; }
    if (filtered < BRAKE_VALID_LOW)  { return 0U; }
    if (filtered > BRAKE_VALID_HIGH) { return 0U; }
    return 1U;
}

static uint8 brake_check_enable(uint8 valid, uint8 pressed)
{
    if (0U == valid)   { return 0U; }
    if (0U == pressed) { return 0U; }
    return 1U;
}

static uint16 brake_calc_command_hys(uint16 percent, uint8 valid, uint16 last_cmd)
{
    uint32 span;
    uint32 mapped;
    uint16 deadzone;

    if (0U == valid)                   { return 0U; }
    deadzone = (0U != last_cmd) ? (BRAKE_CMD_DEADZONE / 2U)
                                : BRAKE_CMD_DEADZONE;
    if (percent <= deadzone) { return 0U; }

    span = (uint32)(BRAKE_CMD_FULL - deadzone);
    mapped = ((uint32)(percent - deadzone) * BRAKE_CMD_FULL) / span;

    if (mapped > BRAKE_CMD_FULL) { mapped = BRAKE_CMD_FULL; }
    return (uint16)mapped;
}

/* ==== 公共接口 ======================================================== */
void pedal_input_init(void)
{
    adc_init(ADC8_CH13_A45, ADC_12BIT);
    adc_init(ADC8_CH15_A47, ADC_12BIT);
    adc_init(ADC3_CH0_A24, ADC_12BIT);
    /* P20_7 已由 MyKey 模块初始化，硬件有外部上拉+去抖电容，这里不重复 gpio_init */

    pedal_a45_raw        = 0U;
    pedal_a47_raw        = 0U;
    pedal_a24_raw        = 0U;
    pedal_thr_filtered   = 0U;
    pedal_thr_percent    = 0U;
    pedal_thr_pressed    = 0U;
    pedal_thr_filt_inited = 0U;

    pedal_brk_filtered   = 0U;
    pedal_brk_percent    = 0U;
    pedal_brk_pressed    = 0U;
    pedal_brk_filt_inited = 0U;

    pedal_throttle_valid      = 0U;
    pedal_throttle_enable_req = 0U;
    pedal_throttle_cmd        = 0U;
    pedal_throttle_cmd_last   = 0U;

    pedal_brake_valid      = 0U;
    pedal_brake_enable_req = 0U;
    pedal_brake_cmd        = 0U;
    pedal_brake_cmd_last   = 0U;

    dir_btn_direction   = 0U;
    dir_btn_last_stable = 1U;
    dir_btn_count       = 0U;
}

void pedal_input_task(void)
{
    uint16 a45_sample;
    uint16 a47_sample;
    uint16 a24_sample;

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
    pedal_throttle_cmd = throttle_calc_command_hys(pedal_thr_percent,
                                                   pedal_throttle_valid,
                                                   pedal_throttle_cmd_last);
    /* Ramp limit */
    {
        uint16 target = pedal_throttle_cmd;
        int32  delta  = (int32)target - (int32)pedal_throttle_cmd_last;
        int32  step;

        if (delta > (int32)THROTTLE_CMD_RAMP_UP)
        {
            step = (int32)THROTTLE_CMD_RAMP_UP;
        }
        else if (delta < -(int32)THROTTLE_CMD_RAMP_DOWN)
        {
            step = -(int32)THROTTLE_CMD_RAMP_DOWN;
        }
        else
        {
            step = delta;
        }

        pedal_throttle_cmd = (uint16)((int32)pedal_throttle_cmd_last + step);
        pedal_throttle_cmd_last = pedal_throttle_cmd;
    }

    /* 倒车时油门 cmd 限到 30% 上限 */
    if (1U == dir_btn_direction)
    {
        if (pedal_throttle_cmd > REVERSE_THROTTLE_LIMIT)
        {
            pedal_throttle_cmd = REVERSE_THROTTLE_LIMIT;
            if (pedal_throttle_cmd_last > REVERSE_THROTTLE_LIMIT)
            {
                pedal_throttle_cmd_last = REVERSE_THROTTLE_LIMIT;
            }
        }
    }

    a24_sample = adc_mean_filter_convert(ADC3_CH0_A24, PEDAL_FILTER_COUNT);
    pedal_a24_raw = a24_sample;

    if (0U == pedal_brk_filt_inited)
    {
        pedal_brk_filtered = a24_sample;
        pedal_brk_filt_inited = 1U;
    }
    else
    {
        int32 diff = (int32)a24_sample - (int32)pedal_brk_filtered;
        pedal_brk_filtered = (uint16)((int32)pedal_brk_filtered
                                      + (diff >> PEDAL_IIR_SHIFT));
    }

    pedal_brk_percent = brake_calc_percent(pedal_brk_filtered);
    pedal_brk_pressed = brake_update_pressed(pedal_brk_pressed,
                                             pedal_brk_filtered);
    pedal_brake_valid = brake_check_valid(pedal_brk_filtered,
                                          pedal_brk_filt_inited);
    pedal_brake_enable_req = brake_check_enable(pedal_brake_valid,
                                                pedal_brk_pressed);
    pedal_brake_cmd = brake_calc_command_hys(pedal_brk_percent,
                                             pedal_brake_valid,
                                             pedal_brake_cmd_last);
    /* Ramp limit */
    {
        uint16 target = pedal_brake_cmd;
        int32  delta  = (int32)target - (int32)pedal_brake_cmd_last;
        int32  step;

        if (delta > (int32)BRAKE_CMD_RAMP_UP)
        {
            step = (int32)BRAKE_CMD_RAMP_UP;
        }
        else if (delta < -(int32)BRAKE_CMD_RAMP_DOWN)
        {
            step = -(int32)BRAKE_CMD_RAMP_DOWN;
        }
        else
        {
            step = delta;
        }

        pedal_brake_cmd = (uint16)((int32)pedal_brake_cmd_last + step);
        pedal_brake_cmd_last = pedal_brake_cmd;
    }

    if ((0U != pedal_brk_pressed) && (0U != pedal_brake_valid))
    {
        pedal_throttle_cmd = 0U;
        pedal_throttle_enable_req = 0U;
        pedal_throttle_cmd_last = 0U;
    }

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
uint16 pedal_input_get_a24(void) { return pedal_a24_raw; }

/* ---- 油门（当前来自 A47 处理管线） ------------------------------------ */
uint16 pedal_input_get_throttle_filtered(void) { return pedal_thr_filtered; }
uint16 pedal_input_get_throttle_percent(void)  { return pedal_thr_percent; }
uint8  pedal_input_get_throttle_pressed(void)  { return pedal_thr_pressed; }

/* ---- 刹车 ------------------------------------------------------------- */
/* A24 brake input is available; active brake only releases throttle. */
uint8 pedal_input_is_brake_available(void) { return 1U; }

uint16 pedal_input_get_brake_filtered(void) { return pedal_brk_filtered; }
uint16 pedal_input_get_brake_percent(void)  { return pedal_brk_percent; }
uint8  pedal_input_get_brake_pressed(void)  { return pedal_brk_pressed; }
uint8  pedal_input_get_brake_valid(void)    { return pedal_brake_valid; }
uint8  pedal_input_get_brake_enable_request(void) { return pedal_brake_enable_req; }
uint16 pedal_input_get_brake_cmd(void)      { return pedal_brake_cmd; }

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
