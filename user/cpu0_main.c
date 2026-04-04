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
    cpu_wait_event_ready();

    while (TRUE)
    {
        if (gnss_flag)
        {
            gnss_flag = 0;
            if (path_recorder_get_state() == PATH_STATE_RECORDING)
            {
                path_recorder_task();
            }
        }

        tuning_soft_task();
        menu_task();
    }
}

#pragma section all restore
