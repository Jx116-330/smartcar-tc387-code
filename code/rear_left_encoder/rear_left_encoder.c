#include "rear_left_encoder.h"

#include "zf_driver_gpio.h"
#include "zf_driver_timer.h"
#include "zf_common_clock.h"

static volatile int32  rl_count            = 0;
static volatile int32  rl_dist_mm          = 0;
static volatile int32  rl_spd_mm_s         = 0;
static volatile uint32 rl_last_rx_ms       = 0U;
static volatile int32  rl_last_speed_count = 0;
static volatile uint32 rl_speed_tick_ms    = 0U;
static volatile uint8  rl_last_pulse_level = 0U;

void rear_left_encoder_init(void)
{
    gpio_init(RLEAR_PULSE_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(RLEAR_DIR_PIN,   GPI, GPIO_HIGH, GPI_PULL_UP);

    rl_count = 0;
    rl_dist_mm = 0;
    rl_spd_mm_s = 0;
    rl_last_rx_ms = 0U;
    rl_last_speed_count = 0;
    rl_speed_tick_ms = 0U;
    rl_last_pulse_level = (uint8)gpio_get_level(RLEAR_PULSE_PIN);

    /* CCU61_CH1 is armed by rear_right_encoder_init; we share that ISR. */
}

void rear_left_encoder_poll_isr(void)
{
    uint8 now;
    uint8 edge;

    now = (uint8)gpio_get_level(RLEAR_PULSE_PIN);

#if RLEAR_PULSE_ACTIVE_RISING
    edge = ((0U == rl_last_pulse_level) && (0U != now)) ? 1U : 0U;
#else
    edge = ((0U != rl_last_pulse_level) && (0U == now)) ? 1U : 0U;
#endif

    if (edge)
    {
        uint8 dir = (uint8)gpio_get_level(RLEAR_DIR_PIN);
        int32 step = (dir == RLEAR_DIR_FORWARD_LEVEL) ? 1 : -1;
        rl_count += (int32)RLEAR_SIGN * step;
    }
    rl_last_pulse_level = now;
}

void rear_left_encoder_task(void)
{
    rl_dist_mm = rl_count / (int32)(RLEAR_PPR / RLEAR_MM_PER_REV);

    rl_speed_tick_ms += 10U;
    if (rl_speed_tick_ms >= RLEAR_SPEED_SAMPLE_MS)
    {
        int32 delta = rl_count - rl_last_speed_count;
        rl_last_speed_count = rl_count;
        rl_speed_tick_ms = 0U;

        rl_spd_mm_s = delta * (int32)(1000U / RLEAR_SPEED_SAMPLE_MS)
                            * (int32)RLEAR_MM_PER_REV
                            / (int32)RLEAR_PPR;
    }
    rl_last_rx_ms = system_getval_ms();
}

int32  rear_left_get_count(void)       { return rl_count; }
int32  rear_left_get_dist_mm(void)     { return rl_dist_mm; }
int32  rear_left_get_spd_mm_s(void)    { return rl_spd_mm_s; }
uint32 rear_left_get_last_rx_ms(void)  { return rl_last_rx_ms; }
uint8  rear_left_is_online(void)       { return 1U; }

void rear_left_encoder_reset(void)
{
    rl_count = 0;
    rl_dist_mm = 0;
    rl_spd_mm_s = 0;
    rl_last_speed_count = 0;
    rl_speed_tick_ms = 0U;
    rl_last_pulse_level = (uint8)gpio_get_level(RLEAR_PULSE_PIN);
}
