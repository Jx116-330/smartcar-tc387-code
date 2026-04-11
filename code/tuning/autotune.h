#ifndef __AUTOTUNE_H__
#define __AUTOTUNE_H__

#include "menu.h"
#include "zf_common_typedef.h"

typedef enum
{
    AUTOTUNE_STATE_IDLE = 0U,
    AUTOTUNE_STATE_RUNNING,
    AUTOTUNE_STATE_WAIT_SCORE,
    AUTOTUNE_STATE_DONE,
    AUTOTUNE_STATE_ABORTED,
    AUTOTUNE_STATE_ERROR,
} autotune_state_t;

typedef struct
{
    pid_param_t base;
    pid_param_t current;
    pid_param_t best;
    float kp_step;
    float ki_step;
    float kd_step;
    float best_score;
    float last_score;
    uint16 candidate_index;
    uint16 candidate_total;
    uint8 best_valid;
    autotune_state_t state;
    uint32 started_ms;
    uint32 updated_ms;
} autotune_status_t;

void autotune_init(void);
void autotune_reset(void);
uint8 autotune_start(const pid_param_t *base);
void autotune_stop(void);
uint8 autotune_is_active(void);
autotune_state_t autotune_get_state(void);
const autotune_status_t *autotune_get_status(void);
uint8 autotune_submit_score(float score);
uint8 autotune_apply_best(uint8 save_to_flash);
void autotune_set_steps(float kp_step, float ki_step, float kd_step);

#endif
