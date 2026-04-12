#include "myhead.h"
#include "icm_attitude.h"
#include "icm_ins.h"
#include "icm_gps_fusion.h"
#include "ins_record.h"
#include "ins_playback.h"
#include "ins_ctrl.h"
#include "pedal_input.h"
#include "board_comm.h"   /* TC264 板间通信（接收诊断链路，用于收 ENC） */
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
    board_comm_init();    /* TC264 板间通信：初始化 UART1 115200 P33_12/P33_13 */
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
        pedal_input_task();

        /* ==== UART1 发送 THR,<cmd>,<enable>,<valid>\r\n（非阻塞，20ms 一次） ====
         * 数据来源是 pedal_input 控制层快照（A47 油门处理链路的产物）：
         *   - cmd     0~1000，带死区+拉伸后的油门命令
         *   - enable  0/1，   驾驶员出力请求（必须 valid=1 且 pressed=1）
         *   - valid   0/1，   输入通道是否在合理范围
         * 三个字段都是已经过 pedal_input_task 一次性产出的只读快照，
         * 这里只做读+格式化+发送，不做任何决策。
         */
        {
            static uint32 txthr_last_ms = 0U;
            uint32 now_ms = system_getval_ms();
            if ((now_ms - txthr_last_ms) >= TXTHR_PERIOD_MS)
            {
                uint16 cmd = pedal_input_get_throttle_cmd();
                uint8  en  = pedal_input_get_throttle_enable_request();
                uint8  vld = pedal_input_get_throttle_valid();
                char   txthr_buf[32];

                txthr_last_ms = now_ms;
                sprintf(txthr_buf, "THR,%u,%u,%u\r\n",
                        (unsigned)cmd, (unsigned)en, (unsigned)vld);
                uart_write_string(TXTHR_UART, txthr_buf);
                printf("[TXTHR] THR,%u,%u,%u\r\n",
                       (unsigned)cmd, (unsigned)en, (unsigned)vld);
            }
        }

        board_comm_task();  /* 保留：从 UART1 RX 收 hq 上来的 ENC 并解析/统计 */
        menu_task();   /* 内部调用 tuning_soft_task() */
    }
}

#pragma section all restore
