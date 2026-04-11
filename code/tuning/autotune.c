#include "autotune.h"

#include <string.h>

#include "zf_common_headfile.h"

#define AUTOTUNE_DEFAULT_KP_STEP 0.10f
#define AUTOTUNE_DEFAULT_KI_STEP 0.005f
#define AUTOTUNE_DEFAULT_KD_STEP 0.05f
#define AUTOTUNE_CANDIDATE_TOTAL 7U

typedef struct
{
    float kp_mul;
    float ki_mul;
    float kd_mul;
} autotune_candidate_delta_t;

static const autotune_candidate_delta_t autotune_candidates[AUTOTUNE_CANDIDATE_TOTAL] = {
    { 0.0f,  0.0f,  0.0f},
    { 1.0f,  0.0f,  0.0f},
    {-1.0f,  0.0f,  0.0f},
    { 0.0f,  1.0f,  0.0f},
    { 0.0f, -1.0f,  0.0f},
    { 0.0f,  0.0f,  1.0f},
    { 0.0f,  0.0f, -1.0f},
};

static autotune_status_t autotune_status;

static float autotune_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void autotune_apply_candidate(uint16 index)
{
    pid_param_t next = autotune_status.base;

    if (index >= AUTOTUNE_CANDIDATE_TOTAL)
    {
        autotune_status.state = AUTOTUNE_STATE_DONE;
        return;
    }

    next.kp += autotune_candidates[index].kp_mul * autotune_status.kp_step;
    next.ki += autotune_candidates[index].ki_mul * autotune_status.ki_step;
    next.kd += autotune_candidates[index].kd_mul * autotune_status.kd_step;

    next.kp = autotune_clamp(next.kp, 0.0f, 50.0f);
    next.ki = autotune_clamp(next.ki, 0.0f, 10.0f);
    next.kd = autotune_clamp(next.kd, 0.0f, 20.0f);

    if (!menu_set_pid_param(&next, 0U))
    {
        autotune_status.state = AUTOTUNE_STATE_ERROR;
        return;
    }

    autotune_status.current = next;
    autotune_status.candidate_index = index;
    autotune_status.updated_ms = system_getval_ms();
    autotune_status.state = AUTOTUNE_STATE_WAIT_SCORE;
}

void autotune_init(void)
{
    memset(&autotune_status, 0, sizeof(autotune_status));
    autotune_status.kp_step = AUTOTUNE_DEFAULT_KP_STEP;
    autotune_status.ki_step = AUTOTUNE_DEFAULT_KI_STEP;
    autotune_status.kd_step = AUTOTUNE_DEFAULT_KD_STEP;
    autotune_status.candidate_total = AUTOTUNE_CANDIDATE_TOTAL;
    autotune_status.state = AUTOTUNE_STATE_IDLE;
}

void autotune_reset(void)
{
    pid_param_t current = {0};
    const pid_param_t *menu_param = menu_get_pid_param();
    float kp_step = autotune_status.kp_step;
    float ki_step = autotune_status.ki_step;
    float kd_step = autotune_status.kd_step;

    if (NULL != menu_param)
    {
        current = *menu_param;
    }

    memset(&autotune_status, 0, sizeof(autotune_status));
    autotune_status.base = current;
    autotune_status.current = current;
    autotune_status.kp_step = (kp_step > 0.0f) ? kp_step : AUTOTUNE_DEFAULT_KP_STEP;
    autotune_status.ki_step = (ki_step > 0.0f) ? ki_step : AUTOTUNE_DEFAULT_KI_STEP;
    autotune_status.kd_step = (kd_step > 0.0f) ? kd_step : AUTOTUNE_DEFAULT_KD_STEP;
    autotune_status.candidate_total = AUTOTUNE_CANDIDATE_TOTAL;
    autotune_status.state = AUTOTUNE_STATE_IDLE;
}

uint8 autotune_start(const pid_param_t *base)
{
    if (NULL == base)
    {
        return 0U;
    }

    autotune_reset();
    autotune_status.base = *base;
    autotune_status.current = *base;
    autotune_status.started_ms = system_getval_ms();
    autotune_status.updated_ms = autotune_status.started_ms;
    autotune_status.state = AUTOTUNE_STATE_RUNNING;
    autotune_apply_candidate(0U);
    return (AUTOTUNE_STATE_WAIT_SCORE == autotune_status.state) ? 1U : 0U;
}

void autotune_stop(void)
{
    if ((AUTOTUNE_STATE_IDLE != autotune_status.state) &&
        (AUTOTUNE_STATE_DONE != autotune_status.state))
    {
        autotune_status.state = AUTOTUNE_STATE_ABORTED;
        autotune_status.updated_ms = system_getval_ms();
    }
}

uint8 autotune_is_active(void)
{
    return (AUTOTUNE_STATE_RUNNING == autotune_status.state || AUTOTUNE_STATE_WAIT_SCORE == autotune_status.state) ? 1U : 0U;
}

autotune_state_t autotune_get_state(void)
{
    return autotune_status.state;
}

const autotune_status_t *autotune_get_status(void)
{
    return &autotune_status;
}

uint8 autotune_submit_score(float score)
{
    uint16 next_index;

    if (AUTOTUNE_STATE_WAIT_SCORE != autotune_status.state)
    {
        return 0U;
    }

    autotune_status.last_score = score;
    if ((!autotune_status.best_valid) || (score > autotune_status.best_score))
    {
        autotune_status.best = autotune_status.current;
        autotune_status.best_score = score;
        autotune_status.best_valid = 1U;
    }

    next_index = (uint16)(autotune_status.candidate_index + 1U);
    if (next_index >= autotune_status.candidate_total)
    {
        autotune_status.state = AUTOTUNE_STATE_DONE;
        autotune_status.updated_ms = system_getval_ms();
        return 1U;
    }

    autotune_status.state = AUTOTUNE_STATE_RUNNING;
    autotune_apply_candidate(next_index);
    return 1U;
}

uint8 autotune_apply_best(uint8 save_to_flash)
{
    if (!autotune_status.best_valid)
    {
        return 0U;
    }

    return menu_set_pid_param(&autotune_status.best, save_to_flash);
}

void autotune_set_steps(float kp_step, float ki_step, float kd_step)
{
    if (kp_step > 0.0f) autotune_status.kp_step = kp_step;
    if (ki_step > 0.0f) autotune_status.ki_step = ki_step;
    if (kd_step > 0.0f) autotune_status.kd_step = kd_step;
}
