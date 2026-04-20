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
}

void drive_motor_stop(void)
{
    /* Both legs low => EG2104 LO on => motor shorted through LS FETs = brake. */
    pwm_set_duty(DM_PWM_L_LEG1, 0U);
    pwm_set_duty(DM_PWM_L_LEG2, 0U);
    pwm_set_duty(DM_PWM_R_LEG1, 0U);
    pwm_set_duty(DM_PWM_R_LEG2, 0U);
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

    if (en && vld && (cmd >= DM_THROTTLE_THRESHOLD))
    {
        uint32 effective = (uint32)cmd * (uint32)limit / 1000U;
        uint32 duty;
        if (effective > 1000U) effective = 1000U;
        duty = effective * (uint32)PWM_DUTY_MAX / 1000U;

        drive_motor_apply((uint16)duty, (uint16)duty, dir, dir, 1U);
    }
    else
    {
        drive_motor_stop();
    }
}
