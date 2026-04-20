#include "drive_motor.h"

#include "zf_driver_pwm.h"
#include "pedal_input.h"

/* ==== PWM channels for 4 EG2104 IN pins (U25 connector) ==== */
#define DM_PWM_L_LEG1           ATOM2_CH4_P33_12   /* Motor L, leg 1 IN */
#define DM_PWM_L_LEG2           ATOM2_CH5_P33_13   /* Motor L, leg 2 IN */
#define DM_PWM_R_LEG1           ATOM2_CH1_P33_5    /* Motor R, leg 1 IN */
#define DM_PWM_R_LEG2           ATOM0_CH2_P33_11   /* Motor R, leg 2 IN */

#define DM_PWM_FREQ_HZ          1000U              /* match TC264 PWM_OUT_FREQ */
#define DM_THROTTLE_THRESHOLD   20U                /* match TC264 PEDAL_MAP_THRESHOLD */

/* Per-motor direction polarity.  Set to 1 if the wheel turns opposite to the
 * requested direction (mechanical mirror-mounting or reversed motor wiring). */
#define DM_MOTOR_L_INVERTED     0U
#define DM_MOTOR_R_INVERTED     1U                 /* vehicle test: right wheel ran reversed */

/* ==== Smooth throttle / brake via duty slew-rate limiter ===========================
 * Called at 10 ms cadence from drive_motor_apply_from_pedal.  Actual duty slews
 * toward the pedal's target duty at the rates below.  As duty ramps down to 0,
 * EG2104 LS-short brake engages progressively (smooth brake), not instantly.
 */
#define DM_SLEW_UP_PER_TICK      500U    /* 0 → PWM_DUTY_MAX in ~200 ms (throttle) */
#define DM_SLEW_DOWN_PER_TICK    200U    /* PWM_DUTY_MAX → 0 in ~500 ms (brake)    */

static uint16 dm_cur_duty_l = 0U;
static uint16 dm_cur_duty_r = 0U;
static uint8  dm_last_dir_l = 0U;
static uint8  dm_last_dir_r = 0U;

static uint16 dm_slew_toward(uint16 cur, uint16 target)
{
    if (cur < target)
    {
        uint16 diff = (uint16)(target - cur);
        return (diff > DM_SLEW_UP_PER_TICK) ? (uint16)(cur + DM_SLEW_UP_PER_TICK) : target;
    }
    if (cur > target)
    {
        uint16 diff = (uint16)(cur - target);
        return (diff > DM_SLEW_DOWN_PER_TICK) ? (uint16)(cur - DM_SLEW_DOWN_PER_TICK) : target;
    }
    return cur;
}

static void dm_write_bridge(pwm_channel_enum leg1, pwm_channel_enum leg2,
                            uint16 duty, uint8 dir)
{
    if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;

    if (0U == dir)                  /* forward: leg1 PWM, leg2 low */
    {
        pwm_set_duty(leg1, duty);
        pwm_set_duty(leg2, 0U);
    }
    else                            /* reverse: leg1 low, leg2 PWM */
    {
        pwm_set_duty(leg1, 0U);
        pwm_set_duty(leg2, duty);
    }
}

void drive_motor_init(void)
{
    /* All four IN pins init as PWM @ 1 kHz, duty 0 (motor braked low side). */
    pwm_init(DM_PWM_L_LEG1, DM_PWM_FREQ_HZ, 0U);
    pwm_init(DM_PWM_L_LEG2, DM_PWM_FREQ_HZ, 0U);
    pwm_init(DM_PWM_R_LEG1, DM_PWM_FREQ_HZ, 0U);
    pwm_init(DM_PWM_R_LEG2, DM_PWM_FREQ_HZ, 0U);

    dm_cur_duty_l = 0U;
    dm_cur_duty_r = 0U;
    dm_last_dir_l = 0U;
    dm_last_dir_r = 0U;
}

void drive_motor_stop(void)
{
    /* Emergency stop: snap to LS short-brake and reset slew state so the next
     * apply_from_pedal call starts ramping up from 0, not from the stale duty. */
    pwm_set_duty(DM_PWM_L_LEG1, 0U);
    pwm_set_duty(DM_PWM_L_LEG2, 0U);
    pwm_set_duty(DM_PWM_R_LEG1, 0U);
    pwm_set_duty(DM_PWM_R_LEG2, 0U);
    dm_cur_duty_l = 0U;
    dm_cur_duty_r = 0U;
}

void drive_motor_apply(uint16 duty_left, uint16 duty_right,
                       uint8  dir_left,  uint8  dir_right,
                       uint8  enable)
{
    if (0U == enable)
    {
        drive_motor_stop();
        return;
    }

    dm_write_bridge(DM_PWM_L_LEG1, DM_PWM_L_LEG2, duty_left,  dir_left);
    dm_write_bridge(DM_PWM_R_LEG1, DM_PWM_R_LEG2, duty_right, dir_right);
}

void drive_motor_apply_from_pedal(void)
{
    uint16 cmd   = pedal_input_get_throttle_cmd();            /* 0~1000 */
    uint8  en    = pedal_input_get_throttle_enable_request()
                   && pedal_input_get_drive_enable();
    uint8  vld   = pedal_input_get_throttle_valid();
    uint16 limit = pedal_input_get_pwm_limit();               /* 0~1000 */
    uint8  dir   = pedal_input_get_direction();               /* 0=fwd 1=rev */

    uint8  dir_l = (uint8)(dir ^ DM_MOTOR_L_INVERTED);
    uint8  dir_r = (uint8)(dir ^ DM_MOTOR_R_INVERTED);
    uint16 target_duty = 0U;

    if (en && vld && (cmd >= DM_THROTTLE_THRESHOLD))
    {
        uint32 effective = (uint32)cmd * (uint32)limit / 1000U;
        if (effective > 1000U) effective = 1000U;
        target_duty = (uint16)(effective * (uint32)PWM_DUTY_MAX / 1000U);
    }

    /* Direction flip while spinning is unsafe under slew: drive would still be
     * pushing in the old direction as the motor reverses.  Snap current duty to
     * 0 on dir change so the motor brakes first, then ramps up in the new dir. */
    if (dir_l != dm_last_dir_l) { dm_cur_duty_l = 0U; }
    if (dir_r != dm_last_dir_r) { dm_cur_duty_r = 0U; }
    dm_last_dir_l = dir_l;
    dm_last_dir_r = dir_r;

    dm_cur_duty_l = dm_slew_toward(dm_cur_duty_l, target_duty);
    dm_cur_duty_r = dm_slew_toward(dm_cur_duty_r, target_duty);

    drive_motor_apply(dm_cur_duty_l, dm_cur_duty_r, dir_l, dir_r, 1U);
}
