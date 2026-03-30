#include "MyEncoder.h"

int switch_encoder_num = 0;
int switch_encoder_change_num = 0;
uint8 switch_encode_bring_flag = 0;
uint8 switch_encode_change_get_buff_flag = 0;

/*
 * 软件解码机械编码器：
 * 1. 直接轮询 A/B 两相信号，避开 GPT12 在当前硬件上的抖动问题
 * 2. 每个合法相位跃迁记 1 个计数
 * 3. 累计到一个完整卡点后，再给菜单输出 1 步
 */
#define SWITCH_ENCODER_COUNTS_PER_STEP   2
#define SWITCH_ENCODER_PENDING_LIMIT     8
#define SWITCH_ENCODER_DIR_SIGN         -1

static int8 switch_encoder_transition_table[16] =
{
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static int switch_encoder_pending_steps = 0;
static int switch_encoder_raw_accum = 0;
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

uint8 If_Switch_Encoder_Change(void)
{
    if (switch_encoder_pending_steps > 0)
    {
        switch_encoder_change_num = 1;
        switch_encoder_pending_steps--;
        switch_encode_change_get_buff_flag = 1;
        return 1;
    }
    else if (switch_encoder_pending_steps < 0)
    {
        switch_encoder_change_num = -1;
        switch_encoder_pending_steps++;
        switch_encode_change_get_buff_flag = 1;
        return 1;
    }

    switch_encoder_change_num = 0;
    switch_encode_change_get_buff_flag = 0;
    return 0;
}
