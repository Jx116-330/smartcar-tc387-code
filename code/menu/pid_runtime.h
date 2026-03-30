#ifndef __PID_RUNTIME_H__
#define __PID_RUNTIME_H__

#include "zf_common_typedef.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} pid_param_t;

typedef struct
{
    pid_param_t param;
    float integral;
    float last_error;
    float output;
} pid_controller_t;

static inline float pid_limit_value(float value, float limit)
{
    if (limit <= 0.0f)
    {
        return value;
    }
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static inline void pid_set_param(pid_controller_t *controller, const pid_param_t *param)
{
    if ((NULL == controller) || (NULL == param))
    {
        return;
    }
    controller->param = *param;
}

static inline void pid_reset(pid_controller_t *controller)
{
    if (NULL == controller)
    {
        return;
    }
    controller->integral = 0.0f;
    controller->last_error = 0.0f;
    controller->output = 0.0f;
}

static inline float pid_calculate(pid_controller_t *controller, float target, float feedback)
{
    float error;
    float derivative;

    if (NULL == controller)
    {
        return 0.0f;
    }

    error = target - feedback;
    controller->integral += error;
    controller->integral = pid_limit_value(controller->integral, controller->param.integral_limit);

    derivative = error - controller->last_error;
    controller->last_error = error;

    controller->output = controller->param.kp * error +
                         controller->param.ki * controller->integral +
                         controller->param.kd * derivative;
    controller->output = pid_limit_value(controller->output, controller->param.output_limit);
    return controller->output;
}

#endif
