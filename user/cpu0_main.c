#include "myhead.h"
#include "icm_attitude.h"
#include "icm_ins.h"
#include "icm_gps_fusion.h"
#include "ins_record.h"
#include "ins_playback.h"
#include "ins_ctrl.h"
#include "pedal_input.h"
#include "Turn.h"
#include "rear_right_encoder.h"
#include "encoder_odom_right.h"
#include "board_comm.h"   /* TC264 板间通信（接收诊断链路，用于收 ENC） */
#include "encoder_odom.h" /* 编码器里程计 → INS 融合 */
#pragma section all "cpu0_dsram"

/* ==== UART1 发送 THR 命令 ====
 * 主板作为"命令源"：每周期把 pedal_input 产出的控制层快照打成 ASCII：
 *     THR,<cmd 0~1000>,<enable 0/1>,<valid 0/1>\r\n
 * 经 UART1 → MAX3232 → 对端 hq 板。
 * UART1 已由 board_comm_init() 完成 115200 P33_12/P33_13 初始化，
 * 此处只调用 uart_write_string 发送，不动初始化。
 */
#define TXTHR_UART              UART_1
#define TXTHR_PERIOD_MS         20U    /* 与 hq 侧 ENC 上行节拍一致 */

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
    board_comm_init();    /* TC264 板间通信：初始化 UART1 115200 P33_12/P33_13 */
    encoder_odom_init();  /* 编码器里程计融合：初始化状态 */

    /* 启动 10ms 定时中断：把固定节拍任务从主循环搬到 ISR
     * CCU60_CH1 (prio 43): pedal_input_task — 保证油门采集 100Hz 确定性
     * CCU61_CH0 (prio 44): ins_record/playback/ctrl — 惯导记录控制 100Hz
     * 两者优先级均低于 1ms ICM (prio 50)，不会抢占惯导采样 */
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

            /* GPS+INS 融合：每次 GNSS 更新时校正 INS */
            icm_gps_fusion_update(gnss.latitude, gnss.longitude,
                                  gnss.speed, gnss.direction,
                                  gnss.state, gnss.satellite_used);

            if (path_recorder_get_state() == PATH_STATE_RECORDING)
            {
                path_recorder_task();
            }
        }

        /* pedal_input_task / ins_record / ins_playback / ins_ctrl
         * 已搬入 10ms 定时中断 (CCU60_CH1 / CCU61_CH0)，不再在主循环调用 */

        /* ==== UART1 发送 THR,<cmd>,<enable>,<valid>,<limit>\r\n（20ms 一次）====
         * enable 已叠加菜单 drive_enable 总开关：只有菜单允许 AND 踏板请求才 = 1
         * limit 是菜单设定的 PWM 占空比上限 (0~1000)
         * TC264 侧用 cmd*limit/1000 算最终 duty，实现百分比出力控制
         */
        {
            static uint32 txthr_last_ms = 0U;
            uint32 now_ms = system_getval_ms();
            if ((now_ms - txthr_last_ms) >= TXTHR_PERIOD_MS)
            {
                uint16 cmd   = pedal_input_get_throttle_cmd();
                uint8  en    = pedal_input_get_throttle_enable_request()
                               && pedal_input_get_drive_enable();    /* 菜单总开关门控 */
                uint8  vld   = pedal_input_get_throttle_valid();
                uint16 limit = pedal_input_get_pwm_limit();
                char   txthr_buf[48];

                txthr_last_ms = now_ms;
                {
                    uint8 dir = pedal_input_get_direction();
                    sprintf(txthr_buf, "THR,%u,%u,%u,%u,%u\r\n",
                            (unsigned)cmd, (unsigned)en, (unsigned)vld, (unsigned)limit, (unsigned)dir);
                    uart_write_string(TXTHR_UART, txthr_buf);
                }
                /* debug printf 降频 */
                {
                    static uint32 txthr_dbg_skip = 0U;
                    if (++txthr_dbg_skip >= 50U)
                    {
                        txthr_dbg_skip = 0U;
                        printf("[TXTHR] THR,%u,%u,%u,%u,%u\r\n",
                               (unsigned)cmd, (unsigned)en, (unsigned)vld, (unsigned)limit,
                               (unsigned)pedal_input_get_direction());
                    }
                }
            }
        }

        board_comm_task();   /* 从 UART1 RX 收 ENCL/HQ 并解析 */
        encoder_odom_task(); /* 编码器里程计 → INS 速度+位置双校正 (25Hz) */
        encoder_odom_right_task();
        menu_task();         /* 菜单 + tuning_soft_task() */
    }
}

#pragma section all restore
