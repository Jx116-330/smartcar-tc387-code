#include "myhead.h"
#include "icm_attitude.h"
#include "icm_ins.h"
#include "icm_gps_fusion.h"
#include "ins_record.h"
#include "ins_playback.h"
#include "ins_ctrl.h"
#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();
    Beep_Init();
    Init_ICM42688();
    icm_attitude_init();
    icm_ins_init();
    pit_ms_init(CCU60_CH0, 1);                      // 1ms 定时中断采样 ICM42688（1kHz）
    gnss_init(TAU1201);
    path_recorder_init();
    ins_record_init();
    ins_playback_init();
    ins_ctrl_init();
    icm_gps_fusion_init();
    menu_init();
    cpu_wait_event_ready();

    while (TRUE)
    {
        if (gnss_flag)
        {
            gnss_flag = 0;

            /* GPS+INS 融合：每次 GNSS 更新时校正 INS */
            icm_gps_fusion_update(gnss.latitude, gnss.longitude,
                                  gnss.speed, gnss.direction,
                                  gnss.state, gnss.satellite_used);

            if (path_recorder_get_state() == PATH_STATE_RECORDING)
            {
                path_recorder_task();
            }
        }

        ins_record_task();
        ins_playback_task();
        ins_ctrl_task();
        menu_task();   /* 内部调用 tuning_soft_task() */
    }
}

#pragma section all restore
