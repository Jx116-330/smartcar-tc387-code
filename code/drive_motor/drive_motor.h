#ifndef __DRIVE_MOTOR_H__
#define __DRIVE_MOTOR_H__

#include "zf_common_typedef.h"

/*********************************************************************************************************************
 * Drive motor (rear wheels) local driver.
 *
 * Hardware: U61 → 4x EG2104 half-bridge drivers (U1-U4) → 2x full H-bridge → 2 motors.
 * Each EG2104's SD# is pulled high (always enabled); each IN is one MCU signal:
 *
 *   Motor L  leg1 IN -> ATOM2_CH4_P33_12
 *   Motor L  leg2 IN -> ATOM2_CH5_P33_13
 *   Motor R  leg1 IN -> ATOM2_CH1_P33_5
 *   Motor R  leg2 IN -> ATOM0_CH2_P33_11
 *
 * Drive rule per motor:
 *   forward : leg1 PWM(duty), leg2 0
 *   reverse : leg1 0,          leg2 PWM(duty)
 *   stop    : leg1 0,          leg2 0            (both LO on = low-side short brake)
 *
 * duty range 0 ~ PWM_DUTY_MAX (10000).  dir: 0 = forward, 1 = reverse.
 *********************************************************************************************************************/

void drive_motor_init(void);
void drive_motor_stop(void);

/* Low-level: write both wheels directly (differential-ready). */
void drive_motor_apply(uint16 duty_left, uint16 duty_right,
                       uint8  dir_left,  uint8  dir_right,
                       uint8  enable);

/* Wraps pedal_input + menu drive_enable / pwm_limit, 1:1 mirror of the
 * old TC264 ApplyThrCommand logic.  Call at 10 ms ISR. */
void drive_motor_apply_from_pedal(void);

#endif /* __DRIVE_MOTOR_H__ */
