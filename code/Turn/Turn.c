/*********************************************************************************************************************
* File: Turn.c
* Brief: Steering motor closed-loop control adapted from ZUST_KADING_TC387.
*
* Known risks kept explicit by design:
* - P14.5/P14.6 are boot-related pins. H-bridge levels during power-up can affect boot mode.
* - If the H-bridge has no hardware deadtime, fast direction reversal can damage the bridge.
*   The current implementation preserves the upstream behavior and relies on H-bridge internal protection.
* - Turn_STARTUP_HOLD_CYCLES = 120 is coupled to the 10ms control period. Recalculate it if the period changes.
* - Turn_ENCODER_COUNT_PER_DEG = 1.0f is a placeholder and must be calibrated on the vehicle.
*********************************************************************************************************************/

#include "Turn.h"

#include "pid_runtime.h"

volatile float turn_target_angle_deg = 0.0f;
volatile float turn_current_angle_deg = 0.0f;

typedef struct
{
    volatile int32 encoder_count;
    volatile int32 encoder_zero;
    uint8 dir_level;
    uint16 startup_hold_cnt;
    uint8 motor_enable;
} Turn_RuntimeState;

typedef struct
{
    float target_angle_deg;
    float encoder_value;
    float angle_deg;
    uint8 motor_enable;
} Turn_MenuState;

static pid_controller_t turn_pid;
static pid_param_t turn_pid_param = {
    Turn_PID_KP,
    Turn_PID_KI,
    Turn_PID_KD,
    Turn_PID_I_MAX,
    (float)Turn_Duty_LIMIT
};

static Turn_RuntimeState turn_state = {
    0,
    0,
    Turn_DIR_POSITIVE_LEVEL,
    0,
    0
};

static Turn_MenuState turn_menu_state = {
    0.0f,
    0.0f,
    0.0f,
    0U
};

static float Turn_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Turn_EncToMenuAngleDeg(float enc_val)
{
    return enc_val * (Turn_MENU_ANGLE_FULL_SCALE / Turn_MENU_ENC_FULL_SCALE);
}

static float Turn_MenuAngleDegToEnc(float angle_val)
{
    return angle_val * (Turn_MENU_ENC_FULL_SCALE / Turn_MENU_ANGLE_FULL_SCALE);
}

static uint32 Turn_GetDutyFromOutput(float output)
{
    float abs_output = Turn_AbsFloat(output);

    if (abs_output <= Turn_CTRL_OUTPUT_DEADBAND)
    {
        return 0U;
    }
    if (abs_output >= (float)Turn_Duty_LIMIT)
    {
        return (uint32)Turn_Duty_LIMIT;
    }
    return (uint32)abs_output;
}

static void Turn_ApplyMotorOutput(float output)
{
    uint32 duty = Turn_GetDutyFromOutput(output);
    uint8 target_dir_level = Turn_DIR_POSITIVE_LEVEL;

    if (0U == duty)
    {
        pwm_set_duty(Turn_PWM, 0U);
        return;
    }

    if (output < 0.0f)
    {
        target_dir_level = (uint8)!Turn_DIR_POSITIVE_LEVEL;
    }

    if (target_dir_level != turn_state.dir_level)
    {
        turn_state.dir_level = target_dir_level;
        gpio_set_level(Turn_DIR_PIN, turn_state.dir_level);
    }

    pwm_set_duty(Turn_PWM, duty);
}

void Turn_MenuTargetAngleSync(void)
{
    Turn_SetTargetAngleDeg(Turn_MenuAngleDegToEnc(turn_menu_state.target_angle_deg));
}

void Turn_MenuPidSync(void)
{
    turn_pid_param.integral_limit = Turn_PID_I_MAX;
    turn_pid_param.output_limit = (float)Turn_Duty_LIMIT;
    pid_set_param(&turn_pid, &turn_pid_param);
}

void Turn_MenuRuntimeUpdate(void)
{
    uint8 new_enable;

    turn_menu_state.encoder_value = (float)Turn_GetEncoderCount();
    turn_menu_state.angle_deg = Turn_EncToMenuAngleDeg(turn_menu_state.encoder_value);

    new_enable = (turn_menu_state.motor_enable != 0U) ? 1U : 0U;
    if (new_enable != turn_state.motor_enable)
    {
        Turn_SetMotorEnable(new_enable);
    }
}

void Turn_Init(void)
{
    gpio_init(Turn_PWM_SAFE_PIN, GPO, Turn_PWM_SAFE_LEVEL, GPO_PUSH_PULL);
    pwm_init(Turn_PWM, Turn_MOTOR_FREQ, 0U);
    pwm_set_duty(Turn_PWM, 0U);

    turn_state.dir_level = Turn_DIR_POSITIVE_LEVEL;
    gpio_init(Turn_DIR_PIN, GPO, turn_state.dir_level, GPO_PUSH_PULL);

    encoder_dir_init(Turn_ENCODER_INDEX, Turn_ENCODER_PULSE_PIN, Turn_ENCODER_DIR_INPUT_PIN);
    encoder_clear_count(Turn_ENCODER_INDEX);

    Turn_MenuPidSync();
    pid_reset(&turn_pid);

    turn_state.encoder_count = 0;
    turn_state.encoder_zero = 0;
    turn_state.startup_hold_cnt = Turn_STARTUP_HOLD_CYCLES;
    turn_state.motor_enable = 0U;

    turn_target_angle_deg = 0.0f;
    turn_current_angle_deg = 0.0f;

    turn_menu_state.target_angle_deg = 0.0f;
    turn_menu_state.encoder_value = 0.0f;
    turn_menu_state.angle_deg = 0.0f;
    turn_menu_state.motor_enable = 0U;
}

void Turn_ControlTask(void)
{
    int16 delta_count = encoder_get_count(Turn_ENCODER_INDEX);

    encoder_clear_count(Turn_ENCODER_INDEX);
    turn_state.encoder_count += (int32)(Turn_ENCODER_SIGN * delta_count);
    turn_current_angle_deg = (float)(turn_state.encoder_count - turn_state.encoder_zero) / Turn_ENCODER_COUNT_PER_DEG;

    if (turn_state.startup_hold_cnt > 0U)
    {
        turn_state.startup_hold_cnt--;
        turn_state.encoder_zero = turn_state.encoder_count;
        turn_current_angle_deg = 0.0f;
        pwm_set_duty(Turn_PWM, 0U);
        return;
    }

    if (!turn_state.motor_enable)
    {
        Turn_Stop();
        return;
    }

    Turn_ApplyMotorOutput(pid_calculate(&turn_pid, turn_target_angle_deg, turn_current_angle_deg));
}

void Turn_SetTargetAngleDeg(float target_deg)
{
    turn_target_angle_deg = target_deg;
}

void Turn_SetMenuTargetAngleDeg(float target_deg)
{
    turn_menu_state.target_angle_deg = target_deg;
    Turn_MenuTargetAngleSync();
}

float *Turn_GetMenuTargetAngleDegPtr(void)
{
    return &turn_menu_state.target_angle_deg;
}

float *Turn_GetMenuEncoderValuePtr(void)
{
    return &turn_menu_state.encoder_value;
}

float *Turn_GetMenuAngleDegPtr(void)
{
    return &turn_menu_state.angle_deg;
}

uint8 *Turn_GetMenuMotorEnablePtr(void)
{
    return &turn_menu_state.motor_enable;
}

float *Turn_GetMenuKpPtr(void)
{
    return &turn_pid_param.kp;
}

float *Turn_GetMenuKiPtr(void)
{
    return &turn_pid_param.ki;
}

float *Turn_GetMenuKdPtr(void)
{
    return &turn_pid_param.kd;
}

void Turn_SetCurrentAngleAsZero(void)
{
    turn_state.encoder_zero = turn_state.encoder_count;
    turn_current_angle_deg = 0.0f;
    turn_target_angle_deg = 0.0f;
    turn_menu_state.target_angle_deg = 0.0f;
    pid_reset(&turn_pid);
}

void Turn_Stop(void)
{
    pid_reset(&turn_pid);
    pwm_set_duty(Turn_PWM, 0U);
}

void Turn_SetMotorEnable(uint8 enable)
{
    turn_state.motor_enable = (enable != 0U) ? 1U : 0U;
    turn_menu_state.motor_enable = turn_state.motor_enable;

    if (!turn_state.motor_enable)
    {
        Turn_Stop();
    }
}

uint8 Turn_GetMotorEnable(void)
{
    return turn_state.motor_enable;
}

int32 Turn_GetEncoderCount(void)
{
    return (turn_state.encoder_count - turn_state.encoder_zero);
}

float Turn_GetCurrentAngleDeg(void)
{
    return turn_current_angle_deg;
}
