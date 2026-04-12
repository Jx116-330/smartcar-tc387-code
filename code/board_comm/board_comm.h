/*********************************************************************************************************************
* File: board_comm.h
* Brief: TC264 板间通信接收模块（最小验证版）
*        硬件：UART1，TX=P33_12，RX=P33_13，115200，经 MAX3232 与 TC264 相连
*        本模块只做：接收 → 组行 → debug 串口打印，不解析协议，不控制执行器
*
* 注意：本模块接管 UART1 的 RX 中断处理。
*       isr.c 中 uart1_rx_isr 原来调用 camera_uart_handler()，
*       现改为调用 board_comm_uart1_rx_handler()。
*       相机功能如需恢复，请重新将该 ISR 指向 camera_uart_handler()。
*
* Author: JX116
*********************************************************************************************************************/

#ifndef __BOARD_COMM_H__
#define __BOARD_COMM_H__

#include "zf_common_typedef.h"

/* ---- 初始化 / 任务（在 cpu0_main 中调用） ----------------------------- */
void board_comm_init(void);
void board_comm_task(void);

/* ---- ISR 入口（仅在 isr.c 中 uart1_rx_isr 内调用） ------------------- */
void board_comm_uart1_rx_handler(void);

/* ---- 只读 Getter（供菜单页显示，不直接操作 UART） -------------------- */
/*
 * 最近一次收到的完整行内容（不含换行符，以 '\0' 结尾）。
 * 指针指向内部静态缓冲区，调用方只读，不要修改或长期持有。
 * 若尚未收到任何行，返回 ""。
 */
const char *board_comm_get_latest_line(void);

/* 累计收到的完整行数（含重复行，只要 \n/\r 结束就计一次） */
uint32 board_comm_get_rx_count(void);

/* system_getval_ms() 时刻：最近一次收到完整行的时间戳（ms）。
 * 若尚未收到任何行，返回 0。 */
uint32 board_comm_get_last_rx_ms(void);

/* 因环形缓冲区满而被丢弃的字节数（溢出计数，可用于判断通信健康度） */
uint16 board_comm_get_overflow_count(void);

/* 诊断用：ISR 层累计取到的字节总数（uart_query_byte 返回非0的次数）
 * 若上板后此值为 0，说明 UART1 硬件层完全没有收到任何字节。 */
uint32 board_comm_get_rx_byte_count(void);

/* ---- ISR vs polling 二分诊断 ---------------------------------------- */
/*
 * ISR 命中计数：每次 user/isr.c 中 uart1_rx_isr 被触发就 +1。
 * 直接由 isr.c 递增，避免通过函数调用引入额外开销/耦合。
 */
extern volatile uint32 uart1_rx_isr_hit_count;

/* ISR 命中计数 getter */
uint32 board_comm_get_uart1_isr_hit_count(void);

/* board_comm_task 中 polling 路径累计取到的字节数
 * 若 IsrHit=0 而 PollBy>0：硬件收到了字节，但 ISR 链路没工作 */
uint32 board_comm_get_poll_byte_count(void);

/* polling 路径最近一次取到的字节原值（十六进制显示用） */
uint8  board_comm_get_last_poll_byte(void);

/*
 * online 判定：
 *   - rx_count > 0  AND  (now_ms - last_rx_ms) <= 500ms  →  1 (online)
 *   - 否则  →  0
 */
uint8 board_comm_is_online(void);

/* ---- 最小 ENC 报文解析（只识别 "ENC,<int>,<int>" 前缀） ---------------
 * 在 board_comm_task 里每完成一行就尝试解析：
 *   - 若解析成功：更新 tim2/tim3 最新值、设置 has_enc_frame=1
 *   - 若解析失败：保留上一次的值，不清零
 * 不发 ACK、不做滤波（派生速度为整数裸值）。
 */
int16 board_comm_get_enc_tim2(void);
int16 board_comm_get_enc_tim3(void);

/* 是否已至少收到过一帧合法的 ENC 报文 */
uint8 board_comm_has_enc_frame(void);

/* ---- 派生速度：counts per second ------------------------------------
 * 换算假设发送板当前周期 = BC_ENC_PERIOD_MS (20ms)：
 *     spd_cps = delta * (1000 / period_ms) = delta * 50
 * 每解析成功一帧就重算一次；接收中断后快照保持最后一次值。
 * 若需要改成滤波版，只需在 board_comm.c 内部加一阶 IIR。
 */
int32 board_comm_get_spd2_cps(void);
int32 board_comm_get_spd3_cps(void);

/* ---- HQ 状态帧解析（hq 板周期上报的整机状态快照） ---------------------
 * 协议：HQ,<arm>,<fresh>,<valid>,<en>,<drv>,<cmd>,<out>,<duty>\r\n
 *   arm   0/1   hq 侧 debug_arm
 *   fresh 0/1   hq 是否在 200ms 内收到过新鲜 THR
 *   valid 0/1   主板油门 valid 标志，经 hq 透传回来
 *   en    0/1   主板油门 enable 标志，经 hq 透传回来
 *   drv   0/1   hq 最终驱动门控结果
 *   cmd   0~65535  hq 收到的原始 THR cmd（未限幅）
 *   out   0~65535  hq 限幅+门控后的目标命令
 *   duty  0~10000  hq 实际写到 PWM 的占空比
 *
 * 每次收到合法 HQ 帧就刷新全部字段 + 更新 last_rx 时间戳；
 * 坏帧保留上一次快照，不清零。
 */
uint8  board_comm_hq_get_arm(void);
uint8  board_comm_hq_get_fresh(void);
uint8  board_comm_hq_get_valid(void);
uint8  board_comm_hq_get_en(void);
uint8  board_comm_hq_get_drv(void);
uint16 board_comm_hq_get_cmd(void);
uint16 board_comm_hq_get_out(void);
uint16 board_comm_hq_get_duty(void);

/* system_getval_ms() 时刻：最近一次成功解析 HQ 帧的时间戳。
 * 若从未收到过任何合法 HQ 帧，返回 0。 */
uint32 board_comm_hq_get_last_rx_ms(void);

/*
 * 主板对 hq 的在线判定：
 *   - 曾经收到过至少一帧合法 HQ，且距最近一次 ≤ BC_HQ_TIMEOUT_MS (250ms) → 1
 *   - 否则 → 0
 *
 * 注意：本判定与 HQ 帧里的 fresh 字段不是一个概念：
 *   fresh  = hq 是否收到主板 THR（下行链路）
 *   online = 主板是否收到 hq HQ 状态帧（上行链路）
 *
 * 本函数在每次被调用时动态重算，不需要周期任务去维护一个全局标志。
 */
uint8  board_comm_hq_is_online(void);

#endif /* __BOARD_COMM_H__ */
