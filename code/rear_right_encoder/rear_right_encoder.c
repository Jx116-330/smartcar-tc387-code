#include "rear_right_encoder.h"

#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"
#include "zf_driver_timer.h"
#include "zf_common_clock.h"
#include "isr_config.h"

static volatile int32  rr_count            = 0;
static volatile int32  rr_dist_mm          = 0;
static volatile int32  rr_spd_mm_s         = 0;
static volatile uint32 rr_last_rx_ms       = 0U;
static volatile int32  rr_last_speed_count = 0;
static volatile uint32 rr_speed_tick_ms    = 0U;
static volatile uint8  rr_last_pulse_level = 0U;

void rear_right_encoder_init(void)
{
    gpio_init(RREAR_PULSE_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(RREAR_DIR_PIN,   GPI, GPIO_HIGH, GPI_PULL_UP);

    rr_count = 0;
    rr_dist_mm = 0;
    rr_spd_mm_s = 0;
    rr_last_rx_ms = 0U;
    rr_last_speed_count = 0;
    rr_speed_tick_ms = 0U;
    rr_last_pulse_level = (uint8)gpio_get_level(RREAR_PULSE_PIN);

    pit_us_init(CCU61_CH1, RREAR_POLL_US);
}

void rear_right_encoder_poll_isr(void)
{
    uint8 now;
    uint8 edge;

    now = (uint8)gpio_get_level(RREAR_PULSE_PIN);

#if RREAR_PULSE_ACTIVE_RISING
    edge = ((0U == rr_last_pulse_level) && (0U != now)) ? 1U : 0U;
#else
    edge = ((0U != rr_last_pulse_level) && (0U == now)) ? 1U : 0U;
#endif

    if (edge)
    {
        uint8 dir = (uint8)gpio_get_level(RREAR_DIR_PIN);
        int32 step = (dir == RREAR_DIR_FORWARD_LEVEL) ? 1 : -1;
        rr_count += (int32)RREAR_SIGN * step;
    }
    rr_last_pulse_level = now;
}

void rear_right_encoder_task(void)
{
    rr_dist_mm = rr_count / (int32)(RREAR_PPR / RREAR_MM_PER_REV);

    rr_speed_tick_ms += 10U;
    if (rr_speed_tick_ms >= RREAR_SPEED_SAMPLE_MS)
    {
        int32 delta = rr_count - rr_last_speed_count;
        rr_last_speed_count = rr_count;
        rr_speed_tick_ms = 0U;

        rr_spd_mm_s = delta * (int32)(1000U / RREAR_SPEED_SAMPLE_MS)
                            * (int32)RREAR_MM_PER_REV
                            / (int32)RREAR_PPR;
    }
    rr_last_rx_ms = system_getval_ms();
}

int32 rear_right_get_count(void)      { return rr_count; }
int32 rear_right_get_dist_mm(void)    { return rr_dist_mm; }
int32 rear_right_get_spd_mm_s(void)   { return rr_spd_mm_s; }
uint32 rear_right_get_last_rx_ms(void){ return rr_last_rx_ms; }
uint8 rear_right_is_online(void)      { return 1U; }

void rear_right_encoder_reset(void)
{
    rr_count = 0;
    rr_dist_mm = 0;
    rr_spd_mm_s = 0;
    rr_last_speed_count = 0;
    rr_speed_tick_ms = 0U;
    rr_last_pulse_level = (uint8)gpio_get_level(RREAR_PULSE_PIN);
}
