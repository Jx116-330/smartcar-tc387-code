#ifndef __REAR_LEFT_ENCODER_H__
#define __REAR_LEFT_ENCODER_H__

#include "zf_common_typedef.h"

/* ==== Hardware (U53 left rear, GPIO-polled DIR+pulse) ====
 * TIM5 is taken by Turn; TIM2 could work for the left, but we keep left/right
 * symmetric by using the same GPIO-poll scheme as rear_right.
 * PULSE = P33.7, DIR = P33.6.  Poll ISR is shared with rear_right (CCU61_CH1).
 */
#define RLEAR_PULSE_PIN             P33_7
#define RLEAR_DIR_PIN               P33_6

/* ==== Parameters, aligned with rear_right ==== */
#define RLEAR_PPR                   1000       /* single-phase pulses per revolution */
#define RLEAR_MM_PER_REV            200        /* mm per revolution */
#define RLEAR_SPEED_SAMPLE_MS       20         /* speed window, same as rear_right */

/* ==== Polarity, tune on vehicle ==== */
#define RLEAR_PULSE_ACTIVE_RISING   1          /* 1=rising edge, 0=falling edge */
#define RLEAR_DIR_FORWARD_LEVEL     1          /* DIR=1 means forward; set 0 if reversed */
#define RLEAR_SIGN                  -1         /* invert final sign to match forward motion */

/* ==== Lifecycle ==== */
void   rear_left_encoder_init(void);
void   rear_left_encoder_poll_isr(void);       /* CCU61_CH1 ISR, shared w/ rear_right */
void   rear_left_encoder_task(void);           /* CCU60_CH1 ISR, 10 ms */

/* ==== Data interface, mirrors rear_right_get_* ==== */
int32  rear_left_get_count(void);
int32  rear_left_get_dist_mm(void);
int32  rear_left_get_spd_mm_s(void);
uint32 rear_left_get_last_rx_ms(void);
uint8  rear_left_is_online(void);              /* local acquisition is always online */

/* ==== Control ==== */
void   rear_left_encoder_reset(void);

#endif /* __REAR_LEFT_ENCODER_H__ */
