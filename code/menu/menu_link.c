/*********************************************************************************************************************
* File: menu_link.c
* Brief: TC264 板间通信 Link Debug 菜单页实现
*        只读 board_comm getter，不直接读 UART，不碰 ISR，不解析协议
*
* 页面布局（8×16 字体，ips200 240x320，footer 安全线 = height_max-16 = 304）：
*   y= 2   "TC264 Link"            黄色标题
*   y=16   ─────────────────────  灰色分割线
*   y=22   "UART1 Link"            青色分组头
*   y=40   "Last: <latest_line>"   白色，最近收到的完整行
*   y=56   "RxCnt: <n>"            白色，累计完整行数
*   y=72   "AgeMs: <n>"            白色，距上次收包毫秒
*   y=88   "Online: YES/NO"        绿/红色
*   y=108  "Drop : <n>"            灰色，溢出字节数
*   y=124  "RxByte: <n>"           白色，字节级累计
*   y=140  "IsrHit: <n>"           白色，ISR 命中
*   y=156  "PollBy: <n>"           白色，polling 字节
*   y=172  "LastBy: 0xNN"          白色，最近字节原值
*   y=188  "TIM2: <n>"             青色，最新 ENC.tim2 (raw 20ms 增量)
*   y=204  "SPD2: <n>"             黄色，派生 cps = TIM2 * 50
*   y=220  "TIM3: <n>"             青色，最新 ENC.tim3 (raw 20ms 增量)
*   y=236  "SPD3: <n>"             黄色，派生 cps = TIM3 * 50
*   y=252  "ENC : OK/--"           绿/灰，是否已收到合法 ENC
*   y=304  "LONG:Exit"             底部灰色提示 (= ips200_height_max - 16)
*
* Author: JX116
*********************************************************************************************************************/

#include "menu_link.h"

#include <stdio.h>

#include "zf_common_headfile.h"
#include "zf_device_ips200.h"
#include "board_comm.h"

#define LINK_VIEW_EXIT_HOLD_MS  250U
#define LINK_VIEW_EXIT_KEY_PIN  P20_2
#define LINK_REFRESH_MS         150U

/* ---- 屏幕辅助工具（与 menu_pedal 同款，加 link_ 前缀避免重复符号） -- */

static void link_fill_rect(uint16 x0, uint16 y0, uint16 x1, uint16 y1, uint16 color)
{
    uint16 y;
    for (y = y0; y <= y1; y++)
    {
        ips200_draw_line(x0, y, x1, y, color);
    }
}

/*
 * 固定宽度字符串输出：用空格补齐到 max_width 像素，防止残字
 * max_width 按像素，除以 8 得到字符数（8px 宽字体）
 */
static void link_show_pad(uint16 x, uint16 y, uint16 max_width, const char *s)
{
    char    buf[64];
    int     max_chars = (int)(max_width / 8U);
    int     i = 0;
    if (max_chars > (int)(sizeof(buf) - 1)) { max_chars = (int)(sizeof(buf) - 1); }
    while ((i < max_chars) && (NULL != s) && (s[i] != '\0')) { buf[i] = s[i]; i++; }
    while (i < max_chars) { buf[i++] = ' '; }
    buf[i] = '\0';
    ips200_show_string(x, y, buf);
}

/* ---- 退出检测：P20_2 长按 250ms ------------------------------------ */

static void link_consume_key1_press(void)
{
    my_key_clear_state(MY_KEY_1);
    key_long_press_flag[MY_KEY_1] = close_status;
}

static uint8 link_is_exit_hold_triggered(void)
{
    static uint32 key_press_start_ms = 0U;
    uint32 now_ms = system_getval_ms();

    if (MY_KEY_RELEASE_LEVEL != gpio_get_level(LINK_VIEW_EXIT_KEY_PIN))
    {
        if (0U == key_press_start_ms)
        {
            key_press_start_ms = now_ms;
        }
        else if ((now_ms - key_press_start_ms) >= LINK_VIEW_EXIT_HOLD_MS)
        {
            key_press_start_ms = 0U;
            return 1U;
        }
    }
    else
    {
        key_press_start_ms = 0U;
    }
    return 0U;
}

/* ---- 核心绘制函数 --------------------------------------------------- */

static void link_draw_debug_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char     line[64];
    uint32   now_ms   = system_getval_ms();
    uint16   col_w    = (uint16)(ips200_width_max - 20U);
    uint16   y;

    /* 限速：不需要重绘时直接返回 */
    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        (now_ms - last_refresh_ms < LINK_REFRESH_MS))
    {
        return;
    }
    last_refresh_ms = now_ms;

    /* 全屏重绘：仅在首次进入或强制刷新时执行 */
    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 2, "TC264 Link");
        ips200_draw_line(0, 16, ips200_width_max - 1, 16, RGB565_GRAY);
        *menu_full_redraw = 0U;
    }

    /* --- 分组头 --- */
    y = 22U;
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(10, y, "UART1 Link");

    /* --- Last line --- */
    y = 40U;
    {
        const char *latest = board_comm_get_latest_line();
        /* 截取前 18 个字符防止超宽；snprintf 会加 '\0' */
        snprintf(line, sizeof(line), "Last: %.18s",
                 (latest[0] != '\0') ? latest : "--");
        link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        link_show_pad(10, y, col_w, line);
    }

    /* --- RxCnt --- */
    y = 56U;
    snprintf(line, sizeof(line), "RxCnt: %lu",
             (unsigned long)board_comm_get_rx_count());
    link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    link_show_pad(10, y, col_w, line);

    /* --- AgeMs --- */
    y = 72U;
    {
        uint32 rx_count = board_comm_get_rx_count();
        if (0U == rx_count)
        {
            snprintf(line, sizeof(line), "AgeMs: ----");
        }
        else
        {
            uint32 age_ms = now_ms - board_comm_get_last_rx_ms();
            snprintf(line, sizeof(line), "AgeMs: %lu", (unsigned long)age_ms);
        }
        link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        link_show_pad(10, y, col_w, line);
    }

    /* --- Online --- */
    y = 88U;
    {
        uint8 online = board_comm_is_online();
        snprintf(line, sizeof(line), "Online: %s", (0U != online) ? "YES" : "NO ");
        link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        if (0U != online)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        }
        link_show_pad(10, y, col_w, line);
    }

    /* --- Drop (overflow bytes) --- */
    y = 108U;
    snprintf(line, sizeof(line), "Drop : %u",
             (unsigned int)board_comm_get_overflow_count());
    link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    link_show_pad(10, y, col_w, line);

    /* --- RxByte（ISR 层字节诊断计数） --- */
    y = 124U;
    snprintf(line, sizeof(line), "RxByte: %lu",
             (unsigned long)board_comm_get_rx_byte_count());
    link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    link_show_pad(10, y, col_w, line);

    /* --- IsrHit（ISR 命中次数） --- */
    y = 140U;
    snprintf(line, sizeof(line), "IsrHit: %lu",
             (unsigned long)board_comm_get_uart1_isr_hit_count());
    link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    link_show_pad(10, y, col_w, line);

    /* --- PollBy（polling 路径取到字节数） --- */
    y = 156U;
    snprintf(line, sizeof(line), "PollBy: %lu",
             (unsigned long)board_comm_get_poll_byte_count());
    link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    link_show_pad(10, y, col_w, line);

    /* --- LastBy（polling 路径最近一个字节原值，16 进制） --- */
    y = 172U;
    snprintf(line, sizeof(line), "LastBy: 0x%02X",
             (unsigned int)board_comm_get_last_poll_byte());
    link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    link_show_pad(10, y, col_w, line);

    /* --- TIM2（最新 ENC.tim2 raw 增量） --- */
    y = 188U;
    snprintf(line, sizeof(line), "TIM2: %d",
             (int)board_comm_get_enc_tim2());
    link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    link_show_pad(10, y, col_w, line);

    /* --- SPD2（派生速度 cps = TIM2 * 50） --- */
    y = 204U;
    snprintf(line, sizeof(line), "SPD2: %ld",
             (long)board_comm_get_spd2_cps());
    link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    link_show_pad(10, y, col_w, line);

    /* --- TIM3（最新 ENC.tim3 raw 增量） --- */
    y = 220U;
    snprintf(line, sizeof(line), "TIM3: %d",
             (int)board_comm_get_enc_tim3());
    link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    link_show_pad(10, y, col_w, line);

    /* --- SPD3（派生速度 cps = TIM3 * 50） --- */
    y = 236U;
    snprintf(line, sizeof(line), "SPD3: %ld",
             (long)board_comm_get_spd3_cps());
    link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    link_show_pad(10, y, col_w, line);

    /* --- ENC 是否已至少解析成功过一次 --- */
    y = 252U;
    {
        uint8 has_enc = board_comm_has_enc_frame();
        snprintf(line, sizeof(line), "ENC : %s", (0U != has_enc) ? "OK" : "--");
        link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        if (0U != has_enc)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        }
        link_show_pad(10, y, col_w, line);
    }

    /* --- 底部提示，y 必须 <= ips200_height_max - 16 --- */
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    link_show_pad(5U, (uint16)(ips200_height_max - 16U),
                  (uint16)(ips200_width_max - 10U), "LONG:Exit");
}

/* ---- HQ Status 页面（只读展示 board_comm 的 HQ 帧解析结果） ---------
 *
 * 页面布局（8×16 字体，ips200 240x320，footer = height_max-16 = 304）：
 *   y= 2   "HQ Status"             黄色标题
 *   y=16   ─────────────────────  灰色分割线
 *   y=22   "HQ Frame"              青色分组头
 *   y=40   "ONLINE: YES/NO"        绿/红
 *   y=56   "ARM   : <0/1>"         ARM=1 黄色 (警示已武装), =0 灰色
 *   y=72   "FRESH : <0/1>"         FRESH=1 绿, =0 红
 *   y=88   "VALID : <0/1>"         VALID=1 绿, =0 红
 *   y=108  "EN    : <0/1>"         EN=1 绿, =0 白
 *   y=124  "DRV   : <0/1>"         DRV=1 绿, =0 白
 *   y=140  "CMD   : <n>"           白色
 *   y=156  "OUT   : <n>"           白色
 *   y=172  "DUTY  : <n>"           白色
 *   y=188  "AGEms : <n> | --"      从未收到 → "--"；否则 now - last_rx_ms
 *   y=304  "LONG:Exit"             底部灰色提示
 *
 * 刷新策略：跟随菜单刷新节拍（LINK_REFRESH_MS），独立的 last_refresh_ms 静态，
 *         不影响 Link Debug 页。只读 board_comm 的 HQ getter，不碰任何写操作。
 */
static void link_draw_hq_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms_hq = 0U;
    char     line[64];
    uint32   now_ms   = system_getval_ms();
    uint16   col_w    = (uint16)(ips200_width_max - 20U);
    uint16   y;

    /* 限速：不需要重绘时直接返回 */
    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        (now_ms - last_refresh_ms_hq < LINK_REFRESH_MS))
    {
        return;
    }
    last_refresh_ms_hq = now_ms;

    /* 全屏重绘：仅在首次进入或强制刷新时执行 */
    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 2, "HQ Status");
        ips200_draw_line(0, 16, ips200_width_max - 1, 16, RGB565_GRAY);
        *menu_full_redraw = 0U;
    }

    /* --- 分组头 --- */
    y = 22U;
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(10, y, "HQ Frame");

    /* --- ONLINE（主板是否最近收到 HQ 帧） --- */
    y = 40U;
    {
        uint8 online = board_comm_hq_is_online();
        snprintf(line, sizeof(line), "ONLINE: %s", (0U != online) ? "YES" : "NO ");
        link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        if (0U != online)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        }
        link_show_pad(10, y, col_w, line);
    }

    /* --- ARM（hq 侧 debug_arm） --- */
    y = 56U;
    {
        uint8 arm = board_comm_hq_get_arm();
        snprintf(line, sizeof(line), "ARM   : %u", (unsigned)arm);
        link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        /* arm=1 用黄色示警：电机已武装 */
        if (0U != arm)
        {
            ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        }
        link_show_pad(10, y, col_w, line);
    }

    /* --- FRESH（hq 是否最近收到主板 THR） --- */
    y = 72U;
    {
        uint8 fresh = board_comm_hq_get_fresh();
        snprintf(line, sizeof(line), "FRESH : %u", (unsigned)fresh);
        link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        if (0U != fresh)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        }
        link_show_pad(10, y, col_w, line);
    }

    /* --- VALID（主板 throttle valid，经 hq 透传） --- */
    y = 88U;
    {
        uint8 valid = board_comm_hq_get_valid();
        snprintf(line, sizeof(line), "VALID : %u", (unsigned)valid);
        link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        if (0U != valid)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        }
        link_show_pad(10, y, col_w, line);
    }

    /* --- EN（主板 throttle enable/pressed，经 hq 透传） --- */
    y = 108U;
    {
        uint8 en = board_comm_hq_get_en();
        snprintf(line, sizeof(line), "EN    : %u", (unsigned)en);
        link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        if (0U != en)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        }
        link_show_pad(10, y, col_w, line);
    }

    /* --- DRV（hq 最终四条件门控结果） --- */
    y = 124U;
    {
        uint8 drv = board_comm_hq_get_drv();
        snprintf(line, sizeof(line), "DRV   : %u", (unsigned)drv);
        link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        if (0U != drv)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        }
        link_show_pad(10, y, col_w, line);
    }

    /* --- CMD（hq 收到的原始 THR cmd，未限幅） --- */
    y = 140U;
    snprintf(line, sizeof(line), "CMD   : %u",
             (unsigned)board_comm_hq_get_cmd());
    link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    link_show_pad(10, y, col_w, line);

    /* --- OUT（hq 限幅 + 门控兜底后的命令） --- */
    y = 156U;
    snprintf(line, sizeof(line), "OUT   : %u",
             (unsigned)board_comm_hq_get_out());
    link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    link_show_pad(10, y, col_w, line);

    /* --- DUTY（hq 实际写到 PWM 硬件的 duty，0~10000） --- */
    y = 172U;
    snprintf(line, sizeof(line), "DUTY  : %u",
             (unsigned)board_comm_hq_get_duty());
    link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    link_show_pad(10, y, col_w, line);

    /* --- AGEms（距最近一次成功解析 HQ 帧的毫秒） --- */
    y = 188U;
    {
        uint32 last = board_comm_hq_get_last_rx_ms();
        if (0U == last)
        {
            /* 从未收到过任何合法 HQ 帧 */
            snprintf(line, sizeof(line), "AGEms : --");
        }
        else
        {
            uint32 age = now_ms - last;
            snprintf(line, sizeof(line), "AGEms : %lu", (unsigned long)age);
        }
        link_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        link_show_pad(10, y, col_w, line);
    }

    /* --- 底部提示，y 必须 <= ips200_height_max - 16 --- */
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    link_show_pad(5U, (uint16)(ips200_height_max - 16U),
                  (uint16)(ips200_width_max - 10U), "LONG:Exit");
}

/* ---- 公共接口 ------------------------------------------------------- */

void menu_link_action_debug(link_view_mode_t *link_mode,
                            uint8 *menu_full_redraw,
                            void (*drain_encoder_events)(void),
                            void (*request_redraw)(uint8 full_redraw),
                            void (*reset_dynamic_region)(void))
{
    *link_mode = LINK_VIEW_DEBUG;
    if (NULL != drain_encoder_events) { drain_encoder_events(); }
    link_consume_key1_press();
    if (NULL != reset_dynamic_region) { reset_dynamic_region(); }
    if (NULL != menu_full_redraw)     { *menu_full_redraw = 1U; }
    if (NULL != request_redraw)       { request_redraw(1U); }
}

void menu_link_action_hq_status(link_view_mode_t *link_mode,
                                uint8 *menu_full_redraw,
                                void (*drain_encoder_events)(void),
                                void (*request_redraw)(uint8 full_redraw),
                                void (*reset_dynamic_region)(void))
{
    *link_mode = LINK_VIEW_HQ_STATUS;
    if (NULL != drain_encoder_events) { drain_encoder_events(); }
    link_consume_key1_press();
    if (NULL != reset_dynamic_region) { reset_dynamic_region(); }
    if (NULL != menu_full_redraw)     { *menu_full_redraw = 1U; }
    if (NULL != request_redraw)       { request_redraw(1U); }
}

uint8 menu_link_handle_view(link_view_mode_t *link_mode,
                            uint8 *menu_full_redraw,
                            void (*drain_encoder_events)(void),
                            void (*request_redraw)(uint8 full_redraw),
                            void (*reset_dynamic_region)(void))
{
    if ((NULL == link_mode) || (LINK_VIEW_NONE == *link_mode)) { return 0U; }

    if ((MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1)) || link_is_exit_hold_triggered())
    {
        link_consume_key1_press();
        *link_mode = LINK_VIEW_NONE;
        if (NULL != drain_encoder_events) { drain_encoder_events(); }
        if (NULL != reset_dynamic_region) { reset_dynamic_region(); }
        if (NULL != request_redraw)       { request_redraw(1U); }
        return 1U;
    }

    if (LINK_VIEW_DEBUG == *link_mode)
    {
        link_draw_debug_page(menu_full_redraw);
    }
    else if (LINK_VIEW_HQ_STATUS == *link_mode)
    {
        link_draw_hq_page(menu_full_redraw);
    }

    return 1U;
}
