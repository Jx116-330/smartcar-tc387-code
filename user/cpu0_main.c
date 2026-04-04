#include "myhead.h"
#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();
    Beep_Init();
    gnss_init(TAU1201);
    path_recorder_init();
    menu_init();
    mock_gnss_init();
    diag_log_init();
    cpu_wait_event_ready();

    /* 无板阶段默认打开 mock 轨迹回放，便于离线联调；上板后可改回 0U。 */
    mock_gnss_set_enabled(1U);
    diag_log_force_dump();

    while (TRUE)
    {
        mock_gnss_task();

        if (gnss_flag)
        {
            gnss_flag = 0;
            if (path_recorder_get_state() == PATH_STATE_RECORDING)
            {
                path_recorder_task();
            }
        }

        tuning_soft_task();
        diag_log_task();
        menu_task();
    }
}

#pragma section all restore
