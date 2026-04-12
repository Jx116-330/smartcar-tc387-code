/*********************************************************************************************************************
* File: board_comm.c
* Brief: TC264 板间通信接收模块（最小验证版）
*
* 实现逻辑：
*   1. uart1_rx_isr 每收到一字节，调用 board_comm_uart1_rx_handler()
*      → 用 uart_query_byte 取出字节，写入 256 字节环形缓冲区
*      → 缓冲满时丢字节并计入 bc_overflow_count
*
*   2. 主循环 board_comm_task() 每次调用时：
*      → 从环形缓冲区读出字节，组装行（以 \r 或 \n 作为行尾）
*      → 收到完整一行后：
*          a. 向 debug 串口（UART0）打印 [BC264] <内容>
*          b. 拷贝到 bc_latest_display（供菜单只读）
*          c. 更新 bc_rx_count / bc_last_rx_ms
*
* 安全性：
*   - 环形缓冲区写索引（bc_tail）只在 ISR 修改
*   - 读索引（bc_head）、bc_latest_display 等只在主循环修改
*   - TC3xx 单字节读写原子，无需加锁
*
* Author: JX116
*********************************************************************************************************************/

#include "board_comm.h"
#include "zf_driver_uart.h"
#include "zf_driver_timer.h"   /* system_getval_ms */
#include "zf_common_debug.h"

/* ==== 硬件配置 ======================================================== */
#define BC_UART         UART_1
#define BC_BAUD         115200U
#define BC_TX_PIN       UART1_TX_P33_12
#define BC_RX_PIN       UART1_RX_P33_13

/* ==== 环形缓冲区（必须是 2 的幂） ===================================== */
#define BC_RXBUF_SIZE   256U
#define BC_RXBUF_MASK   (BC_RXBUF_SIZE - 1U)

static uint8   bc_rxbuf[BC_RXBUF_SIZE];
static uint8   bc_head = 0U;    /* 主循环读指针，只有 board_comm_task 修改 */
static uint8   bc_tail = 0U;    /* ISR 写指针，只有 board_comm_uart1_rx_handler 修改 */

/* ==== 行组装缓冲区 ==================================================== */
#define BC_LINE_SIZE    80U

static char    bc_line[BC_LINE_SIZE];         /* 当前正在组装的行 */
static uint8   bc_line_pos = 0U;

/* ==== 菜单只读状态（只在主循环 board_comm_task 里修改） =============== */
static char    bc_latest_display[BC_LINE_SIZE]; /* 最近完整行 */
static uint32  bc_rx_count     = 0U;            /* 累计完整行数 */
static uint32  bc_last_rx_ms   = 0U;            /* 最近完整行时间戳 */
static uint16  bc_overflow_count = 0U;          /* 环形缓冲区满丢字节计数 */
static uint32  bc_rx_byte_count  = 0U;          /* 诊断：字节级累计（ISR 路径 + polling 路径） */

/* ==== ISR vs polling 二分诊断计数 ====================================
 * uart1_rx_isr_hit_count：ISR 命中计数，由 user/isr.c 直接 ++
 * bc_poll_byte_count    ：board_comm_task 中 polling 路径取到的字节总数
 * bc_last_poll_byte     ：polling 路径最近一次取到的字节原值
 */
volatile uint32 uart1_rx_isr_hit_count = 0U;
static   uint32 bc_poll_byte_count     = 0U;
static   uint8  bc_last_poll_byte      = 0U;

/* ==== 最小 ENC 报文解析状态 ===========================================
 * 只在主循环 board_comm_task 里写，菜单只读访问。 */
static int16  bc_enc_tim2       = 0;
static int16  bc_enc_tim3       = 0;
static uint8  bc_enc_has_frame  = 0U;

/* ==== 派生速度（counts per second） ==================================
 * 发送板当前周期 BC_ENC_PERIOD_MS = 20ms，所以每秒 = 50 帧。
 * 速度换算：spd_cps = delta * (1000 / period_ms) = delta * BC_CPS_SCALE
 *
 *   delta int16 最大 ~32767  → spd_cps 最大 ~1.6M，放得下 int32
 *
 * 当前实现是纯整数裸值，不做滤波，需要滤波时把 raw_spd 喂进一阶 IIR 即可。
 */
#define BC_ENC_PERIOD_MS        20                              /* 与发送板 TXENC_PERIOD_MS 保持一致 */
#define BC_CPS_SCALE            (1000 / BC_ENC_PERIOD_MS)       /* = 50 */

static int32  bc_spd2_cps       = 0;
static int32  bc_spd3_cps       = 0;

/* ==== HQ 状态帧解析（hq 板 10Hz 周期上报） ============================
 * 协议：HQ,<arm>,<fresh>,<valid>,<en>,<drv>,<cmd>,<out>,<duty>\r\n
 *
 * 存储策略：
 *   - 每次成功解析就整体覆盖全部字段 + 更新 last_rx_ms
 *   - 解析失败 → 保留上一次快照、不更新时间戳，下一次 is_online() 自然会因超时判 0
 *   - is_online() 每次被调用时动态重算，不需要周期任务维护全局布尔
 *
 * BC_HQ_TIMEOUT_MS 设为 250ms：hq 侧发送周期 100ms，允许丢 1 帧仍在线，丢 2 帧判离线。
 */
#define BC_HQ_TIMEOUT_MS        250U

static uint8  bc_hq_arm         = 0U;
static uint8  bc_hq_fresh       = 0U;
static uint8  bc_hq_valid       = 0U;
static uint8  bc_hq_en          = 0U;
static uint8  bc_hq_drv         = 0U;
static uint16 bc_hq_cmd         = 0U;
static uint16 bc_hq_out         = 0U;
static uint16 bc_hq_duty        = 0U;
static uint32 bc_hq_last_rx_ms  = 0U;    /* 最近一次成功解析的时间戳 */
static uint8  bc_hq_seen        = 0U;    /* 是否收到过至少一帧合法 HQ */

/* ==== 内部工具 ======================================================== */
static uint8 bc_is_full(void)
{
    return ((uint8)((bc_tail + 1U) & BC_RXBUF_MASK)) == bc_head;
}

static uint8 bc_is_empty(void)
{
    return bc_head == bc_tail;
}

/* ==== 最小 ENC 报文解析 ==============================================
 * 识别格式：  "ENC,<int>,<int>"（行末换行符已在拼行阶段被剥掉）
 * 解析成功返回 1 并写 *out_t2 / *out_t3；否则返回 0 不修改 out。
 * 不使用 sscanf / strtol / string.h，仅做手写状态机。
 */
static uint8 bc_parse_enc_line(const char *s, int16 *out_t2, int16 *out_t3)
{
    int32 v;
    int8  sign;
    uint8 i;
    uint8 have_digit;

    if ((s[0] != 'E') || (s[1] != 'N') || (s[2] != 'C') || (s[3] != ','))
    {
        return 0U;
    }
    i = 4U;

    /* --- 第一个整数：tim2 --- */
    sign = 1;
    if (s[i] == '-')      { sign = -1; i++; }
    else if (s[i] == '+') {             i++; }

    v = 0;
    have_digit = 0U;
    while ((s[i] >= '0') && (s[i] <= '9'))
    {
        v = (v * 10) + (int32)(s[i] - '0');
        have_digit = 1U;
        i++;
    }
    if ((0U == have_digit) || (s[i] != ','))
    {
        return 0U;
    }
    *out_t2 = (int16)((int32)sign * v);
    i++;

    /* --- 第二个整数：tim3 --- */
    sign = 1;
    if (s[i] == '-')      { sign = -1; i++; }
    else if (s[i] == '+') {             i++; }

    v = 0;
    have_digit = 0U;
    while ((s[i] >= '0') && (s[i] <= '9'))
    {
        v = (v * 10) + (int32)(s[i] - '0');
        have_digit = 1U;
        i++;
    }
    if (0U == have_digit)
    {
        return 0U;
    }
    *out_t3 = (int16)((int32)sign * v);

    /* 行尾必须是 '\0'（board_comm_task 拼行后已加）或其他非数字字符——都视为合法收尾 */
    return 1U;
}

/* ==== 最小 HQ 状态帧解析 ==============================================
 * 识别格式： "HQ,<arm>,<fresh>,<valid>,<en>,<drv>,<cmd>,<out>,<duty>"
 *            （行末换行符已在拼行阶段被剥掉）
 * 8 个字段全部是非负整数，字段 1~5 只取 0/1，字段 6~8 是无符号。
 * 成功返回 1 并写全部 out；任一字段格式错/少逗号 → 返回 0，out 不修改。
 *
 * 与 bc_parse_enc_line 同款手写状态机，不使用 sscanf / strtol / string.h。
 */
static uint8 bc_parse_hq_line(const char *s,
                              uint8  *out_arm,
                              uint8  *out_fresh,
                              uint8  *out_valid,
                              uint8  *out_en,
                              uint8  *out_drv,
                              uint16 *out_cmd,
                              uint16 *out_out,
                              uint16 *out_duty)
{
    uint32 v;
    uint8  i;
    uint8  have_digit;

    /* 前缀必须是 "HQ," */
    if ((s[0] != 'H') || (s[1] != 'Q') || (s[2] != ','))
    {
        return 0U;
    }
    i = 3U;

    /* --- Field 1: arm (0/1，但用通用整数扫法) --- */
    v = 0U;
    have_digit = 0U;
    while ((s[i] >= '0') && (s[i] <= '9'))
    {
        v = v * 10U + (uint32)(s[i] - '0');
        have_digit = 1U;
        i++;
    }
    if ((0U == have_digit) || (s[i] != ',')) { return 0U; }
    *out_arm = (0U != v) ? 1U : 0U;
    i++;

    /* --- Field 2: fresh --- */
    v = 0U;
    have_digit = 0U;
    while ((s[i] >= '0') && (s[i] <= '9'))
    {
        v = v * 10U + (uint32)(s[i] - '0');
        have_digit = 1U;
        i++;
    }
    if ((0U == have_digit) || (s[i] != ',')) { return 0U; }
    *out_fresh = (0U != v) ? 1U : 0U;
    i++;

    /* --- Field 3: valid --- */
    v = 0U;
    have_digit = 0U;
    while ((s[i] >= '0') && (s[i] <= '9'))
    {
        v = v * 10U + (uint32)(s[i] - '0');
        have_digit = 1U;
        i++;
    }
    if ((0U == have_digit) || (s[i] != ',')) { return 0U; }
    *out_valid = (0U != v) ? 1U : 0U;
    i++;

    /* --- Field 4: en --- */
    v = 0U;
    have_digit = 0U;
    while ((s[i] >= '0') && (s[i] <= '9'))
    {
        v = v * 10U + (uint32)(s[i] - '0');
        have_digit = 1U;
        i++;
    }
    if ((0U == have_digit) || (s[i] != ',')) { return 0U; }
    *out_en = (0U != v) ? 1U : 0U;
    i++;

    /* --- Field 5: drv --- */
    v = 0U;
    have_digit = 0U;
    while ((s[i] >= '0') && (s[i] <= '9'))
    {
        v = v * 10U + (uint32)(s[i] - '0');
        have_digit = 1U;
        i++;
    }
    if ((0U == have_digit) || (s[i] != ',')) { return 0U; }
    *out_drv = (0U != v) ? 1U : 0U;
    i++;

    /* --- Field 6: cmd (uint16) --- */
    v = 0U;
    have_digit = 0U;
    while ((s[i] >= '0') && (s[i] <= '9'))
    {
        v = v * 10U + (uint32)(s[i] - '0');
        have_digit = 1U;
        i++;
    }
    if ((0U == have_digit) || (s[i] != ',')) { return 0U; }
    if (v > 65535U) { v = 65535U; }
    *out_cmd = (uint16)v;
    i++;

    /* --- Field 7: out (uint16) --- */
    v = 0U;
    have_digit = 0U;
    while ((s[i] >= '0') && (s[i] <= '9'))
    {
        v = v * 10U + (uint32)(s[i] - '0');
        have_digit = 1U;
        i++;
    }
    if ((0U == have_digit) || (s[i] != ',')) { return 0U; }
    if (v > 65535U) { v = 65535U; }
    *out_out = (uint16)v;
    i++;

    /* --- Field 8: duty (uint16，0~10000) --- */
    v = 0U;
    have_digit = 0U;
    while ((s[i] >= '0') && (s[i] <= '9'))
    {
        v = v * 10U + (uint32)(s[i] - '0');
        have_digit = 1U;
        i++;
    }
    if (0U == have_digit) { return 0U; }
    if (v > 65535U) { v = 65535U; }
    *out_duty = (uint16)v;

    /* 行尾允许 '\0' 或其他非数字字符，与 ENC 解析器同款 */
    return 1U;
}

/* ==== 共用：单字节喂入器 =============================================
 * ISR 路径和 polling 路径都通过它把字节送进环形缓冲区 / 字节诊断计数。
 * 抽成静态函数以避免两条路径的处理逻辑漂移。
 *
 * 注意：ISR 与 polling 理论上可能并发调用本函数，
 *       极端情况下 bc_tail 可能被并发修改丢一个字节，
 *       这对 ISR-vs-polling 二分诊断结论没有影响，可接受。
 */
static void bc_feed_byte(uint8 byte)
{
    bc_rx_byte_count++;

    if (!bc_is_full())
    {
        bc_rxbuf[bc_tail] = byte;
        bc_tail = (uint8)((bc_tail + 1U) & BC_RXBUF_MASK);
    }
    else
    {
        /* 缓冲满：丢弃字节并计数 */
        if (bc_overflow_count < 0xFFFFU)
        {
            bc_overflow_count++;
        }
    }
}

/* ==== ISR 入口 ======================================================== */
/*
 * 由 isr.c 中的 uart1_rx_isr 调用（已替换原来的 camera_uart_handler()）。
 * 如需恢复相机功能，将 isr.c 中该行改回 camera_uart_handler(); 即可。
 */
void board_comm_uart1_rx_handler(void)
{
    uint8 byte;
    if (0U != uart_query_byte(BC_UART, &byte))
    {
        bc_feed_byte(byte);
    }
}

/* ==== 初始化 ========================================================== */
void board_comm_init(void)
{
    bc_head           = 0U;
    bc_tail           = 0U;
    bc_line_pos       = 0U;
    bc_rx_count       = 0U;
    bc_last_rx_ms     = 0U;
    bc_overflow_count = 0U;
    bc_rx_byte_count  = 0U;

    /* ISR vs polling 诊断计数归零 */
    uart1_rx_isr_hit_count = 0U;
    bc_poll_byte_count     = 0U;
    bc_last_poll_byte      = 0U;

    /* ENC 解析状态归零 */
    bc_enc_tim2      = 0;
    bc_enc_tim3      = 0;
    bc_enc_has_frame = 0U;

    /* 派生速度归零 */
    bc_spd2_cps      = 0;
    bc_spd3_cps      = 0;

    /* HQ 状态帧归零 */
    bc_hq_arm        = 0U;
    bc_hq_fresh      = 0U;
    bc_hq_valid      = 0U;
    bc_hq_en         = 0U;
    bc_hq_drv        = 0U;
    bc_hq_cmd        = 0U;
    bc_hq_out        = 0U;
    bc_hq_duty       = 0U;
    bc_hq_last_rx_ms = 0U;
    bc_hq_seen       = 0U;

    bc_line[0]            = '\0';
    bc_latest_display[0]  = '\0';

    uart_init(BC_UART, BC_BAUD, BC_TX_PIN, BC_RX_PIN);

    uart_write_string(DEBUG_UART_INDEX,
                      "[BC] board_comm_init: UART1 115200 P33_12/P33_13 ready\r\n");
}

/* ==== 主循环任务 ======================================================= */
void board_comm_task(void)
{
    uint8  byte;
    uint8  i;
    uint8  ch;

    /* === 诊断 polling 路径 ===
     * 即使 ISR 链路没工作，也主动从 UART1 FIFO 里把字节拉出来：
     *   - 统计 polling 命中字节数 bc_poll_byte_count
     *   - 记录最近一个字节的原值 bc_last_poll_byte
     *   - 喂入与 ISR 相同的 bc_feed_byte()，不改变下游拼行逻辑
     * 只要 PollBy 往上涨，就能证明 UART1 硬件层其实收到了字节。
     */
    while (0U != uart_query_byte(BC_UART, &ch))
    {
        bc_poll_byte_count++;
        bc_last_poll_byte = ch;
        bc_feed_byte(ch);
    }

    while (!bc_is_empty())
    {
        byte    = bc_rxbuf[bc_head];
        bc_head = (uint8)((bc_head + 1U) & BC_RXBUF_MASK);

        if ((byte == (uint8)'\n') || (byte == (uint8)'\r'))
        {
            if (bc_line_pos > 0U)
            {
                bc_line[bc_line_pos] = '\0';

                /* a. debug 串口打印（原始行，不分类） */
                uart_write_string(DEBUG_UART_INDEX, "[BC264] ");
                uart_write_string(DEBUG_UART_INDEX, bc_line);
                uart_write_string(DEBUG_UART_INDEX, "\r\n");

                /* a2. 前缀分类打印（现场联调用，和 [BC264] 并存，信息量最大）
                 *     不论后面 parse 成功/失败，这一条都先打，让你能在串口滚动里
                 *     一眼看出"本次收到的整行到底是什么类型":
                 *       - THR,...    → [BCRX-THR]
                 *       - HQ,...     → [BCRX-HQ]
                 *       - ENC,...    → [BCRX-ENC]
                 *       - 其它       → [BCRX-OTHER]
                 *     只做 uart_write_string 多段拼装，不 snprintf，不修改 bc_line。
                 */
                {
                    const char *tag;
                    if      ((bc_line[0] == 'T') && (bc_line[1] == 'H') &&
                             (bc_line[2] == 'R') && (bc_line[3] == ','))
                    {
                        tag = "[BCRX-THR] ";
                    }
                    else if ((bc_line[0] == 'H') && (bc_line[1] == 'Q') &&
                             (bc_line[2] == ','))
                    {
                        tag = "[BCRX-HQ] ";
                    }
                    else if ((bc_line[0] == 'E') && (bc_line[1] == 'N') &&
                             (bc_line[2] == 'C') && (bc_line[3] == ','))
                    {
                        tag = "[BCRX-ENC] ";
                    }
                    else
                    {
                        tag = "[BCRX-OTHER] ";
                    }
                    uart_write_string(DEBUG_UART_INDEX, tag);
                    uart_write_string(DEBUG_UART_INDEX, bc_line);
                    uart_write_string(DEBUG_UART_INDEX, "\r\n");
                }

                /* b. 更新菜单只读状态（逐字节拷贝，避免使用 string.h） */
                for (i = 0U; i < bc_line_pos; i++)
                {
                    bc_latest_display[i] = bc_line[i];
                }
                bc_latest_display[bc_line_pos] = '\0';

                /* c. 计数 + 时间戳 */
                bc_rx_count++;
                bc_last_rx_ms = system_getval_ms();

                /* d. 最小 ENC 解析：成功则更新只读快照 + 派生速度
                 *    （hq 已停发 ENC，但 ENC 解析分支保留，兼容旧流量或回退） */
                {
                    int16 t2;
                    int16 t3;
                    if (0U != bc_parse_enc_line(bc_line, &t2, &t3))
                    {
                        bc_enc_tim2      = t2;
                        bc_enc_tim3      = t3;
                        bc_enc_has_frame = 1U;

                        /* 派生速度：整数裸值，spd = delta * 50 (counts/sec) */
                        bc_spd2_cps = (int32)t2 * (int32)BC_CPS_SCALE;
                        bc_spd3_cps = (int32)t3 * (int32)BC_CPS_SCALE;
                    }
                }

                /* e. 最小 HQ 状态帧解析：hq 周期上报的整机状态快照
                 *    成功 → 整体覆盖 8 个字段 + 更新 last_rx_ms + 打印 [HQRX]
                 *    失败 → 静默丢弃（保留上次快照，超时后由 is_online 自然判离线）
                 */
                {
                    uint8  h_arm;
                    uint8  h_fresh;
                    uint8  h_valid;
                    uint8  h_en;
                    uint8  h_drv;
                    uint16 h_cmd;
                    uint16 h_out;
                    uint16 h_duty;
                    if (0U != bc_parse_hq_line(bc_line,
                                               &h_arm, &h_fresh, &h_valid,
                                               &h_en,  &h_drv,
                                               &h_cmd, &h_out, &h_duty))
                    {
                        bc_hq_arm        = h_arm;
                        bc_hq_fresh      = h_fresh;
                        bc_hq_valid      = h_valid;
                        bc_hq_en         = h_en;
                        bc_hq_drv        = h_drv;
                        bc_hq_cmd        = h_cmd;
                        bc_hq_out        = h_out;
                        bc_hq_duty       = h_duty;
                        bc_hq_last_rx_ms = system_getval_ms();
                        bc_hq_seen       = 1U;

                        /* [HQRX] debug 打印（和 [BC264] 同款多段写法，避开 snprintf） */
                        uart_write_string(DEBUG_UART_INDEX, "[HQRX] ");
                        uart_write_string(DEBUG_UART_INDEX, bc_line);
                        uart_write_string(DEBUG_UART_INDEX, " online=1\r\n");
                    }
                }

                bc_line_pos = 0U;
            }
        }
        else
        {
            if (bc_line_pos < (uint8)(BC_LINE_SIZE - 1U))
            {
                bc_line[bc_line_pos] = (char)byte;
                bc_line_pos++;
            }
            /* 行过长：后续字节被丢弃直到遇到换行，防止越界 */
        }
    }
}

/* ==== 只读 Getter ==================================================== */
const char *board_comm_get_latest_line(void)
{
    return bc_latest_display;
}

uint32 board_comm_get_rx_count(void)
{
    return bc_rx_count;
}

uint32 board_comm_get_last_rx_ms(void)
{
    return bc_last_rx_ms;
}

uint16 board_comm_get_overflow_count(void)
{
    return bc_overflow_count;
}

uint32 board_comm_get_rx_byte_count(void)
{
    return bc_rx_byte_count;
}

uint32 board_comm_get_uart1_isr_hit_count(void)
{
    return uart1_rx_isr_hit_count;
}

uint32 board_comm_get_poll_byte_count(void)
{
    return bc_poll_byte_count;
}

uint8 board_comm_get_last_poll_byte(void)
{
    return bc_last_poll_byte;
}

/* ---- ENC 解析结果 ---------------------------------------------------- */
int16 board_comm_get_enc_tim2(void)
{
    return bc_enc_tim2;
}

int16 board_comm_get_enc_tim3(void)
{
    return bc_enc_tim3;
}

uint8 board_comm_has_enc_frame(void)
{
    return bc_enc_has_frame;
}

/* ---- 派生速度 getter ------------------------------------------------- */
int32 board_comm_get_spd2_cps(void)
{
    return bc_spd2_cps;
}

int32 board_comm_get_spd3_cps(void)
{
    return bc_spd3_cps;
}

/* ---- HQ 状态帧 getter ------------------------------------------------ */
uint8  board_comm_hq_get_arm(void)          { return bc_hq_arm;   }
uint8  board_comm_hq_get_fresh(void)        { return bc_hq_fresh; }
uint8  board_comm_hq_get_valid(void)        { return bc_hq_valid; }
uint8  board_comm_hq_get_en(void)           { return bc_hq_en;    }
uint8  board_comm_hq_get_drv(void)          { return bc_hq_drv;   }
uint16 board_comm_hq_get_cmd(void)          { return bc_hq_cmd;   }
uint16 board_comm_hq_get_out(void)          { return bc_hq_out;   }
uint16 board_comm_hq_get_duty(void)         { return bc_hq_duty;  }
uint32 board_comm_hq_get_last_rx_ms(void)   { return bc_hq_last_rx_ms; }

/*
 * 主板对 hq 的在线判定：
 *   - 收到过至少一帧合法 HQ AND (now_ms - last_rx_ms) <= BC_HQ_TIMEOUT_MS → 1
 *   - 否则 → 0
 *
 * 每次被调用都动态重算，不维护全局标志；任何周期任务（菜单刷新/诊断打印/
 * 安全联动）只要调这个函数就能拿到实时状态。
 */
uint8 board_comm_hq_is_online(void)
{
    uint32 now_ms;
    if (0U == bc_hq_seen)
    {
        return 0U;
    }
    now_ms = system_getval_ms();
    if ((now_ms - bc_hq_last_rx_ms) <= BC_HQ_TIMEOUT_MS)
    {
        return 1U;
    }
    return 0U;
}

/*
 * online 判定：
 *   曾经收到过数据，且距上次收包不超过 500ms
 */
uint8 board_comm_is_online(void)
{
    uint32 now_ms;
    if (0U == bc_rx_count)
    {
        return 0U;
    }
    now_ms = system_getval_ms();
    if ((now_ms - bc_last_rx_ms) <= 500U)
    {
        return 1U;
    }
    return 0U;
}
