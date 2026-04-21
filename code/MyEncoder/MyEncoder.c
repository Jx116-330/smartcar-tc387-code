#include "MyEncoder.h"

/* 共享于 ISR 和主循环 —— 必须 volatile */
volatile int   switch_encoder_num                 = 0;
volatile int   switch_encoder_change_num          = 0;
volatile uint8 switch_encode_bring_flag           = 0;
volatile uint8 switch_encode_change_get_buff_flag = 0;

/*
 * 软件解码机械编码器：
 * 1. 直接轮询 A/B 两相信号，避开 GPT12 在当前硬件上的抖动问题
 * 2. 每个合法相位跃迁记 1 个计数
 * 3. 累计到一个完整 detent 后，再给菜单输出 1 步
 *
 * 运行上下文：
 *   Get_Switch_Num()            由 CCU61_CH1 (5 kHz) ISR 调用，每 200us 一次
 *   If_Switch_Encoder_Change()  由 menu_task 主循环调用，用中断禁用保护 RMW
 *
 * 阈值解释（重要，调菜单灵敏度时看这里）：
 *   COUNTS_PER_STEP = 一个菜单步需要的正交跳变次数。
 *     • 标准 EC11（full-detent）  ：每格 4 跳变 → 设 4  (当前)
 *     • Half-detent (EC11-HS 等) ：每格 2 跳变 → 设 2
 *     • 如果转 1 格菜单跳 2 项  → 阈值偏小，往上加
 *     • 如果转 2 格菜单才跳 1 项 → 阈值偏大，往下减
 */
#define SWITCH_ENCODER_COUNTS_PER_STEP   4
#define SWITCH_ENCODER_PENDING_LIMIT     8
#define SWITCH_ENCODER_DIR_SIGN         -1

static int8 switch_encoder_transition_table[16] =
{
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

/* pending_steps 由 ISR 累积、主循环消耗 → volatile */
static volatile int switch_encoder_pending_steps = 0;
/* raw_accum / prev_state 只在 ISR 上下文使用，不跨上下文 → 不需 volatile */
static int   switch_encoder_raw_accum  = 0;
static uint8 switch_encoder_prev_state = 0xFF;

static uint8 switch_encoder_read_state(void)
{
    uint8 a_level = (uint8)gpio_get_level(SWITCH_ENCODER_A_PIN);
    uint8 b_level = (uint8)gpio_get_level(SWITCH_ENCODER_B_PIN);
    return (uint8)((a_level << 1) | b_level);
}

void MyEncoder_Init(void)
{
    gpio_init(SWITCH_ENCODER_A_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH_ENCODER_B_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);

    switch_encoder_num = 0;
    switch_encoder_change_num = 0;
    switch_encode_bring_flag = 0;
    switch_encode_change_get_buff_flag = 0;
    switch_encoder_pending_steps = 0;
    switch_encoder_raw_accum = 0;
    switch_encoder_prev_state = switch_encoder_read_state();
}

/* ISR 上下文 (CCU61_CH1, 5 kHz)：状态机累计正交跳变 */
void Get_Switch_Num(void)
{
    uint8 current_state = switch_encoder_read_state();
    uint8 transition_index;
    int8 delta;

    if (switch_encoder_prev_state > 3U)
    {
        switch_encoder_prev_state = current_state;
        return;
    }

    if (current_state == switch_encoder_prev_state)
    {
        switch_encode_bring_flag = (switch_encoder_pending_steps != 0) ? 1U : 0U;
        return;
    }

    transition_index = (uint8)((switch_encoder_prev_state << 2) | current_state);
    delta = switch_encoder_transition_table[transition_index];
    switch_encoder_prev_state = current_state;

    if (0 == delta)
    {
        return;
    }

    delta = (int8)(delta * SWITCH_ENCODER_DIR_SIGN);
    switch_encoder_raw_accum += delta;

    /* pending_steps 按正负累加，不做反向清零：
     * 用户快转正向 +5 再反转 -1，pending 从 +5 变 +4，cursor 净走 +4，
     * 与物理位移一致。反向响应速度由上层 menu 抽干 (drain-all) 保证，
     * 不需要在 ISR 层抹掉对向 pending（那会丢掉用户输入）。 */
    if (switch_encoder_raw_accum >= SWITCH_ENCODER_COUNTS_PER_STEP)
    {
        if (switch_encoder_pending_steps < SWITCH_ENCODER_PENDING_LIMIT)
        {
            switch_encoder_pending_steps++;
        }
        switch_encoder_num++;
        switch_encoder_raw_accum = 0;
    }
    else if (switch_encoder_raw_accum <= -SWITCH_ENCODER_COUNTS_PER_STEP)
    {
        if (switch_encoder_pending_steps > -SWITCH_ENCODER_PENDING_LIMIT)
        {
            switch_encoder_pending_steps--;
        }
        switch_encoder_num--;
        switch_encoder_raw_accum = 0;
    }

    switch_encode_bring_flag = (switch_encoder_pending_steps != 0) ? 1U : 0U;
}

/* 主循环上下文：消耗 1 步 pending。临界区保护 RMW 不被 ISR 插队 */
uint8 If_Switch_Encoder_Change(void)
{
    uint32 irq_state;
    int local_pending;
    uint8 result = 0U;

    irq_state = interrupt_global_disable();
    local_pending = switch_encoder_pending_steps;
    if (local_pending > 0)
    {
        switch_encoder_pending_steps = local_pending - 1;
        switch_encoder_change_num = 1;
        switch_encode_change_get_buff_flag = 1;
        result = 1U;
    }
    else if (local_pending < 0)
    {
        switch_encoder_pending_steps = local_pending + 1;
        switch_encoder_change_num = -1;
        switch_encode_change_get_buff_flag = 1;
        result = 1U;
    }
    else
    {
        switch_encoder_change_num = 0;
        switch_encode_change_get_buff_flag = 0;
    }
    interrupt_global_enable(irq_state);

    return result;
}
