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
#include "zf_common_interrupt.h"

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

/* ==== ENCL 左后轮编码器帧（TC264 处理后上报） ===========================
 * 协议：ENCL,<count>,<dist_mm>,<spd_mm_s>\r\n
 * 三个有符号 int32，TC264 每 20ms 发一帧。仅左后轮有效。
 */
#define BC_ENCL_TIMEOUT_MS      2000U   /* 容忍偶尔丢帧，从 500 放宽到 2000 */
#define BC_ENCL_SIGN            (-1)    /* vehicle test: invert left rear ENCL sign to match forward motion */

static int32  bc_encl_count      = 0;
static int32  bc_encl_dist_mm    = 0;
static int32  bc_encl_spd_mm_s   = 0;
static uint32 bc_encl_last_rx_ms = 0U;
static uint8  bc_encl_seen       = 0U;
static uint32 bc_encl_ok_count   = 0U;  /* ENCL 解析成功计数 */
static uint32 bc_encl_fail_count = 0U;  /* 前缀匹配但解析失败计数 */

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

/* ==== ENCL 左后轮编码器帧解析 =========================================
 * 识别格式：  "ENCL,<count>,<dist_mm>,<spd_mm_s>"
 * 三个有符号 int32，逗号分隔。手写状态机，与 bc_parse_enc_line 同款。
 * 成功返回 1 并写 out；否则返回 0。
 */
static uint8 bc_parse_encl_line(const char *s,
                                int32 *out_count,
                                int32 *out_dist,
                                int32 *out_spd)
{
    int32 v;
    int8  sign;
    uint8 i;
    uint8 have_digit;

    /* 前缀: "ENCL," (5 字符) */
    if ((s[0] != 'E') || (s[1] != 'N') || (s[2] != 'C') ||
        (s[3] != 'L') || (s[4] != ','))
    {
        return 0U;
    }
    i = 5U;

    /* --- field 1: count (signed int32) --- */
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
    if ((0U == have_digit) || (s[i] != ',')) { return 0U; }
    *out_count = (int32)sign * v;
    i++;

    /* --- field 2: dist_mm (signed int32) --- */
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
    if ((0U == have_digit) || (s[i] != ',')) { return 0U; }
    *out_dist = (int32)sign * v;
    i++;

    /* --- field 3: spd_mm_s (signed int32) --- */
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
    if (0U == have_digit) { return 0U; }
    *out_spd = (int32)sign * v;

    /* --- 可选 XOR 校验: *XX（兼容无校验的旧固件） --- */
    if (s[i] == '*')
    {
        uint8 expected_xor = 0U;
        uint8 rx_xor = 0U;
        uint8 hi;
        uint8 lo;
        uint8 j;

        /* 算 'E' 到 '*' 之前所有字节的 XOR */
        for (j = 0U; j < i; j++)
            expected_xor ^= (uint8)s[j];

        /* 读两位十六进制 */
        i++;
        hi = (uint8)s[i];
        if      (hi >= '0' && hi <= '9') { hi = (uint8)(hi - '0'); }
        else if (hi >= 'A' && hi <= 'F') { hi = (uint8)(hi - 'A' + 10U); }
        else if (hi >= 'a' && hi <= 'f') { hi = (uint8)(hi - 'a' + 10U); }
        else { return 0U; }

        i++;
        lo = (uint8)s[i];
        if      (lo >= '0' && lo <= '9') { lo = (uint8)(lo - '0'); }
        else if (lo >= 'A' && lo <= 'F') { lo = (uint8)(lo - 'A' + 10U); }
        else if (lo >= 'a' && lo <= 'f') { lo = (uint8)(lo - 'a' + 10U); }
        else { return 0U; }

        rx_xor = (uint8)((hi << 4U) | lo);
        if (rx_xor != expected_xor) { return 0U; }  /* 校验失败 → 丢弃 */
    }
    /* else: 无 '*' → 旧固件无校验，照常接受 */

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
/* ISR and polling both feed this ring buffer; protect the write index. */
static void bc_feed_byte(uint8 byte)
{
    uint32 irq_state = interrupt_global_disable();

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

    interrupt_global_enable(irq_state);
}

/* ==== ISR 入口 ======================================================== */
/*
 * 由 isr.c 中的 uart1_rx_isr 调用（已替换原来的 camera_uart_handler()）。
 * 如需恢复相机功能，将 isr.c 中该行改回 camera_uart_handler(); 即可。
 */
void board_comm_uart1_rx_handler(void)
{
    uint8 byte;
    while (0U != uart_query_byte(BC_UART, &byte))
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

    /* ENCL 解析状态归零 */
    bc_encl_count      = 0;
    bc_encl_dist_mm    = 0;
    bc_encl_spd_mm_s   = 0;
    bc_encl_last_rx_ms = 0U;
    bc_encl_seen       = 0U;
    bc_encl_ok_count   = 0U;
    bc_encl_fail_count = 0U;

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
    uart_rx_interrupt(BC_UART, 1U);

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

                /* a. debug 串口打印（节流版：高频帧降频，避免阻塞主循环）
                 *    ENCL 帧 50Hz 到达，逐字节阻塞发送 (~4ms/帧) 是主循环的最大瓶颈。
                 *    改为：ENCL 每 50 帧打 1 次 (~1Hz)；其它帧照常打。
                 */
                {
                    static uint32 bc_dbg_encl_skip = 0U;
                    const char *tag = NULL;
                    uint8 is_encl = 0U;

                    if      ((bc_line[0] == 'E') && (bc_line[1] == 'N') &&
                             (bc_line[2] == 'C') && (bc_line[3] == 'L') &&
                             (bc_line[4] == ','))
                    {
                        is_encl = 1U;
                        bc_dbg_encl_skip++;
                        if (bc_dbg_encl_skip >= 50U)
                        {
                            bc_dbg_encl_skip = 0U;
                            tag = "[BCRX-ENCL] ";
                        }
                    }
                    else if ((bc_line[0] == 'T') && (bc_line[1] == 'H') &&
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

                    if (NULL != tag)
                    {
                        uart_write_string(DEBUG_UART_INDEX, tag);
                        uart_write_string(DEBUG_UART_INDEX, bc_line);
                        uart_write_string(DEBUG_UART_INDEX, "\r\n");
                    }
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

                /* d. 前缀路由 — 按首字母分发到对应 parser，避免每行盲扫全部 */
                if (bc_line[0] == 'E' && bc_line[1] == 'N' && bc_line[2] == 'C')
                {
                    if (bc_line[3] == 'L' && bc_line[4] == ',')
                    {
                        /* ENCL 左后轮处理后编码器帧 */
                        int32 el_count;
                        int32 el_dist;
                        int32 el_spd;
                        if (0U != bc_parse_encl_line(bc_line, &el_count, &el_dist, &el_spd))
                        {
                            bc_encl_count      = el_count;
                            bc_encl_dist_mm    = el_dist;
                            bc_encl_spd_mm_s   = el_spd;
                            bc_encl_last_rx_ms = system_getval_ms();
                            bc_encl_seen       = 1U;
                            if (bc_encl_ok_count < 0xFFFFFFFFU) { bc_encl_ok_count++; }
                        }
                        else
                        {
                            if (bc_encl_fail_count < 0xFFFFFFFFU) { bc_encl_fail_count++; }
                        }
                    }
                    else if (bc_line[3] == ',')
                    {
                        /* ENC 旧双编码器帧（兼容保留） */
                        int16 t2;
                        int16 t3;
                        if (0U != bc_parse_enc_line(bc_line, &t2, &t3))
                        {
                            bc_enc_tim2      = t2;
                            bc_enc_tim3      = t3;
                            bc_enc_has_frame = 1U;
                            bc_spd2_cps = (int32)t2 * (int32)BC_CPS_SCALE;
                            bc_spd3_cps = (int32)t3 * (int32)BC_CPS_SCALE;
                        }
                    }
                }
                else if (bc_line[0] == 'H' && bc_line[1] == 'Q' && bc_line[2] == ',')
                {
                    /* HQ 状态帧 */
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

                        /* [HQRX] debug 打印：每 10 帧打 1 次 (~1Hz) */
                        {
                            static uint32 bc_hq_dbg_skip = 0U;
                            if (++bc_hq_dbg_skip >= 10U)
                            {
                                bc_hq_dbg_skip = 0U;
                                uart_write_string(DEBUG_UART_INDEX, "[HQRX] ");
                                uart_write_string(DEBUG_UART_INDEX, bc_line);
                                uart_write_string(DEBUG_UART_INDEX, " online=1\r\n");
                            }
                        }
                    }
                }
                /* 其它前缀: 不做 parser 尝试 */

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

/* ---- ENCL 左后轮编码器 getter ---------------------------------------- */
int32  board_comm_encl_get_count(void)       { return BC_ENCL_SIGN * bc_encl_count; }
int32  board_comm_encl_get_dist_mm(void)     { return BC_ENCL_SIGN * bc_encl_dist_mm; }
int32  board_comm_encl_get_spd_mm_s(void)    { return BC_ENCL_SIGN * bc_encl_spd_mm_s; }
uint32 board_comm_encl_get_last_rx_ms(void)  { return bc_encl_last_rx_ms; }

uint8 board_comm_encl_has_frame(void)
{
    return bc_encl_seen;
}

uint32 board_comm_encl_get_ok_count(void)   { return bc_encl_ok_count; }
uint32 board_comm_encl_get_fail_count(void) { return bc_encl_fail_count; }

uint8 board_comm_encl_is_online(void)
{
    uint32 now_ms;
    if (0U == bc_encl_seen) { return 0U; }
    now_ms = system_getval_ms();
    if ((now_ms - bc_encl_last_rx_ms) <= BC_ENCL_TIMEOUT_MS) { return 1U; }
    return 0U;
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
