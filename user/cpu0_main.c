#include <ICM42688/icm_attitude.h>
#include <ICM42688/icm_gps_fusion.h>
#include <ICM42688/icm_ins.h>
#include <ICM42688/ins_ctrl.h>
#include <ICM42688/ins_playback.h>
#include <ICM42688/ins_record.h>
#include "myhead.h"
#include "pedal_input.h"
#include "Turn.h"
#include "rear_right_encoder.h"
#include "encoder_odom_right.h"
#include "rear_left_encoder.h" /* 本地左后编码器（TIM2 硬件 DIR 模式） */
#include "drive_motor.h"       /* 本地两路电机驱动（ATOM2_CH4/CH1 + DIR） */
#include "encoder_odom.h" /* 编码器里程计 → INS 融合 */
#include "ins_enc_tune.h" /* INS/编码器融合增益 运行时可调 + flash 持久化 */
#include "MyEncoder.h"    /* 菜单旋钮（5kHz CCU61_CH1 ISR 采样） */
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
    pedal_input_init();
    Turn_Init();
    drive_motor_init();   /* 本地两路电机 PWM + DIR */
    rear_left_encoder_init();
    ins_enc_tune_init();  /* 加载 INS/编码器融合增益（flash 失败回默认值） */
    encoder_odom_init();  /* 编码器里程计融合：初始化状态 */

    /* 启动 10ms 定时中断：把固定节拍任务从主循环搬到 ISR
     * CCU60_CH1 (prio 43): pedal_input_task — 保证油门采集 100Hz 确定性
     * CCU61_CH0 (prio 44): ins_record/playback/ctrl — 惯导记录控制 100Hz
     * 两者优先级均低于 1ms ICM (prio 50)，不会抢占惯导采样 */
    /* 菜单旋钮 GPIO + 状态必须在 CCU61_CH1 PIT 启动前就绪，
     * 因为 rear_right_encoder_init 会启动该 5kHz ISR，里面会调 Get_Switch_Num */
    MyEncoder_Init();
    rear_right_encoder_init();
    encoder_odom_right_init();
    pit_ms_init(CCU60_CH1, 10);
    pit_ms_init(CCU61_CH0, 10);

    menu_init();
    cpu_wait_event_ready();

    while (TRUE)
    {
        if (gnss_flag)
        {
            gnss_flag = 0;

            /* GPS+INS 融合暂时关闭（不用 GPS 校正 INS，也不触发首帧锁原点 +
             * encoder_odom_reset 的副作用）。重新启用时恢复下面这段调用即可。 */
            /* icm_gps_fusion_update(gnss.latitude, gnss.longitude,
                                     gnss.speed, gnss.direction,
                                     gnss.state, gnss.satellite_used); */

            if (path_recorder_get_state() == PATH_STATE_RECORDING)
            {
                path_recorder_task();
            }
        }

        /* pedal_input_task / ins_record / ins_playback / ins_ctrl / drive_motor_apply
         * 已搬入 10ms 定时中断 (CCU60_CH1 / CCU61_CH0)，不再在主循环调用 */

        encoder_odom_task(); /* 编码器里程计 → INS 速度+位置双校正 (25Hz) */
        encoder_odom_right_task();
        menu_task();         /* 菜单 + tuning_soft_task() */
    }
}

#pragma section all restore
