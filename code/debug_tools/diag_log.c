#include "diag_log.h"

#include <stdio.h>

#include "zf_common_headfile.h"
#include "menu.h"
#include "path_recorder.h"
#include "mock_gnss.h"

#define DIAG_LOG_PERIOD_MS 1000U

static uint8 diag_log_enabled = 1U;
static uint32 diag_log_last_ms = 0U;

static void diag_log_dump_once(void)
{
    const pid_param_t *pid_param = menu_get_pid_param();
    const path_record_config_t *record_cfg = path_recorder_get_config();

    printf("[diag] ms=%lu mock=%u rec_state=%d points=%u fix=%d sat=%d spd=%.2f lat=%.6f lon=%.6f\r\n",
           (unsigned long)system_getval_ms(),
           (unsigned int)mock_gnss_is_enabled(),
           (int)path_recorder_get_state(),
           (unsigned int)path_recorder_get_point_count(),
           gnss.state,
           gnss.satellite_used,
           gnss.speed,
           gnss.latitude,
           gnss.longitude);

    if ((NULL != pid_param) && (NULL != record_cfg))
    {
        printf("[diag] pid(kp=%.3f,ki=%.3f,kd=%.3f,ilim=%.2f,olim=%.2f) rec(dist=%.2f,intv=%lu,sat=%u,max=%.1f)\r\n",
               pid_param->kp,
               pid_param->ki,
               pid_param->kd,
               pid_param->integral_limit,
               pid_param->output_limit,
               record_cfg->min_record_distance,
               (unsigned long)record_cfg->min_record_interval_ms,
               (unsigned int)record_cfg->min_satellites,
               record_cfg->max_record_speed_kph);
    }
}

void diag_log_init(void)
{
    diag_log_enabled = 1U;
    diag_log_last_ms = system_getval_ms();
}

void diag_log_task(void)
{
    uint32 now_ms;

    if (!diag_log_enabled)
    {
        return;
    }

    now_ms = system_getval_ms();
    if ((uint32)(now_ms - diag_log_last_ms) < DIAG_LOG_PERIOD_MS)
    {
        return;
    }

    diag_log_last_ms = now_ms;
    diag_log_dump_once();
}

void diag_log_force_dump(void)
{
    diag_log_dump_once();
}

uint8 diag_log_is_enabled(void)
{
    return diag_log_enabled;
}

void diag_log_set_enabled(uint8 enabled)
{
    diag_log_enabled = enabled ? 1U : 0U;
    diag_log_last_ms = system_getval_ms();
}
