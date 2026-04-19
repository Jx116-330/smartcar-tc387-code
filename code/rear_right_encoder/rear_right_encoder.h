#ifndef __REAR_RIGHT_ENCODER_H__
#define __REAR_RIGHT_ENCODER_H__

#include "zf_common_typedef.h"

/* ==== Hardware ==== */
#define RREAR_PULSE_PIN             P21_7
#define RREAR_DIR_PIN               P21_6

/* ==== Parameters, align with TC264 left rear encoder before vehicle calibration ==== */
#define RREAR_PPR                   1000       /* single-phase pulses per revolution */
#define RREAR_MM_PER_REV            200        /* mm per revolution */
#define RREAR_SPEED_SAMPLE_MS       20         /* speed window, same as left rear */
#define RREAR_POLL_US               200        /* 5 kHz polling */

/* ==== Polarity, tune on vehicle ==== */
#define RREAR_PULSE_ACTIVE_RISING   1          /* 1=rising edge, 0=falling edge */
#define RREAR_DIR_FORWARD_LEVEL     1          /* DIR=1 means forward; set 0 if reversed */
#define RREAR_SIGN                  -1         /* vehicle test: invert final sign to match forward motion */

/* ==== Lifecycle ==== */
void   rear_right_encoder_init(void);
void   rear_right_encoder_poll_isr(void);      /* CCU61_CH1 ISR, 200 us */
void   rear_right_encoder_task(void);          /* CCU60_CH1 ISR, 10 ms */

/* ==== Data interface, field names align with board_comm_encl_get_* ==== */
int32  rear_right_get_count(void);
int32  rear_right_get_dist_mm(void);
int32  rear_right_get_spd_mm_s(void);
uint32 rear_right_get_last_rx_ms(void);
uint8  rear_right_is_online(void);             /* local acquisition is always online */

/* ==== Control ==== */
void   rear_right_encoder_reset(void);

#endif /* __REAR_RIGHT_ENCODER_H__ */
