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
#include "menu_ui_utils.h"

#define LINK_REFRESH_MS             150U
#define BC_ENCL_DISPLAY_WARN_MS     1000U   /* AGEms 超过此值显示红色 */

static uint32 link_exit_hold_timer = 0U;

/* ---- 核心绘制函数 --------------------------------------------------- */

static void link_draw_debug_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char     val[64];
    uint32   now_ms   = system_getval_ms();
    uint16   end_x    = (uint16)(ips200_width_max - 10U);
    uint16   val_w;

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

        /* --- 标题 + 分割线 --- */
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 2, "TC264 Link");
        ips200_draw_line(0, 16, ips200_width_max - 1, 16, RGB565_GRAY);

        /* --- 分组头 --- */
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 22, "UART1 Link");

        /* --- 静态行标签（只画一次） --- */
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10,  40, "Last: ");
        ips200_show_string(10,  56, "RxCnt: ");
        ips200_show_string(10,  72, "AgeMs: ");
        ips200_show_string(10,  88, "Online: ");

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        ips200_show_string(10, 108, "Drop : ");

        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 124, "RxByte: ");
        ips200_show_string(10, 140, "IsrHit: ");
        ips200_show_string(10, 156, "PollBy: ");
        ips200_show_string(10, 172, "LastBy: ");

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 188, "TIM2: ");

        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 204, "SPD2: ");

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 220, "TIM3: ");

        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 236, "SPD3: ");

        /* ENC label 颜色在值变化时设置，但标签用白色 */
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 252, "ENC : ");

        /* --- 底部提示 --- */
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(5U, (uint16)(ips200_height_max - 16U),
                      (uint16)(ips200_width_max - 10U), "LONG:Exit");

        *menu_full_redraw = 0U;
    }

    /* ================================================================
     * 每帧刷新：只清除并重绘 VALUE 部分（标签区域不动）
     * val_x = label_chars * 8 + 10
     * ================================================================ */

    /* --- Last line --- val_x=58 (6 chars: "Last: ") */
    {
        const char *latest = board_comm_get_latest_line();
        snprintf(val, sizeof(val), "%.18s",
                 (latest[0] != '\0') ? latest : "--");
        val_w = (uint16)(end_x - 58U);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        menu_ui_show_pad(58, 40, val_w, val);
    }

    /* --- RxCnt --- val_x=66 (7 chars: "RxCnt: ") */
    snprintf(val, sizeof(val), "%lu",
             (unsigned long)board_comm_get_rx_count());
    val_w = (uint16)(end_x - 66U);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(66, 56, val_w, val);

    /* --- AgeMs --- val_x=66 (7 chars: "AgeMs: ") */
    {
        uint32 rx_count = board_comm_get_rx_count();
        if (0U == rx_count)
        {
            snprintf(val, sizeof(val), "----");
        }
        else
        {
            uint32 age_ms = now_ms - board_comm_get_last_rx_ms();
            snprintf(val, sizeof(val), "%lu", (unsigned long)age_ms);
        }
        val_w = (uint16)(end_x - 66U);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        menu_ui_show_pad(66, 72, val_w, val);
    }

    /* --- Online --- val_x=74 (8 chars: "Online: ") */
    {
        uint8 online = board_comm_is_online();
        snprintf(val, sizeof(val), "%s", (0U != online) ? "YES" : "NO ");
        val_w = (uint16)(end_x - 74U);
        if (0U != online)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        }
        menu_ui_show_pad(74, 88, val_w, val);
    }

    /* --- Drop --- val_x=66 (7 chars: "Drop : ") */
    snprintf(val, sizeof(val), "%u",
             (unsigned int)board_comm_get_overflow_count());
    val_w = (uint16)(end_x - 66U);
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    menu_ui_show_pad(66, 108, val_w, val);

    /* --- RxByte --- val_x=74 (8 chars: "RxByte: ") */
    snprintf(val, sizeof(val), "%lu",
             (unsigned long)board_comm_get_rx_byte_count());
    val_w = (uint16)(end_x - 74U);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(74, 124, val_w, val);

    /* --- IsrHit --- val_x=74 (8 chars: "IsrHit: ") */
    snprintf(val, sizeof(val), "%lu",
             (unsigned long)board_comm_get_uart1_isr_hit_count());
    val_w = (uint16)(end_x - 74U);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(74, 140, val_w, val);

    /* --- PollBy --- val_x=74 (8 chars: "PollBy: ") */
    snprintf(val, sizeof(val), "%lu",
             (unsigned long)board_comm_get_poll_byte_count());
    val_w = (uint16)(end_x - 74U);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(74, 156, val_w, val);

    /* --- LastBy --- val_x=74 (8 chars: "LastBy: ") */
    snprintf(val, sizeof(val), "0x%02X",
             (unsigned int)board_comm_get_last_poll_byte());
    val_w = (uint16)(end_x - 74U);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(74, 172, val_w, val);

    /* --- TIM2 --- val_x=58 (6 chars: "TIM2: ") */
    snprintf(val, sizeof(val), "%d",
             (int)board_comm_get_enc_tim2());
    val_w = (uint16)(end_x - 58U);
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    menu_ui_show_pad(58, 188, val_w, val);

    /* --- SPD2 --- val_x=58 (6 chars: "SPD2: ") */
    snprintf(val, sizeof(val), "%ld",
             (long)board_comm_get_spd2_cps());
    val_w = (uint16)(end_x - 58U);
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    menu_ui_show_pad(58, 204, val_w, val);

    /* --- TIM3 --- val_x=58 (6 chars: "TIM3: ") */
    snprintf(val, sizeof(val), "%d",
             (int)board_comm_get_enc_tim3());
    val_w = (uint16)(end_x - 58U);
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    menu_ui_show_pad(58, 220, val_w, val);

    /* --- SPD3 --- val_x=58 (6 chars: "SPD3: ") */
    snprintf(val, sizeof(val), "%ld",
             (long)board_comm_get_spd3_cps());
    val_w = (uint16)(end_x - 58U);
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    menu_ui_show_pad(58, 236, val_w, val);

    /* --- ENC --- val_x=58 (6 chars: "ENC : ") */
    {
        uint8 has_enc = board_comm_has_enc_frame();
        snprintf(val, sizeof(val), "%s", (0U != has_enc) ? "OK" : "--");
        val_w = (uint16)(end_x - 58U);
        if (0U != has_enc)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        }
        menu_ui_show_pad(58, 252, val_w, val);
    }
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
    char     val[64];
    uint32   now_ms   = system_getval_ms();
    uint16   end_x    = (uint16)(ips200_width_max - 10U);
    uint16   val_w    = (uint16)(end_x - 74U);   /* all HQ labels are 8 chars */

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

        /* --- 标题 + 分割线 --- */
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 2, "HQ Status");
        ips200_draw_line(0, 16, ips200_width_max - 1, 16, RGB565_GRAY);

        /* --- 分组头 --- */
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 22, "HQ Frame");

        /* --- 静态行标签（只画一次，全部白色） --- */
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10,  40, "ONLINE: ");
        ips200_show_string(10,  56, "ARM   : ");
        ips200_show_string(10,  72, "FRESH : ");
        ips200_show_string(10,  88, "VALID : ");
        ips200_show_string(10, 108, "EN    : ");
        ips200_show_string(10, 124, "DRV   : ");
        ips200_show_string(10, 140, "CMD   : ");
        ips200_show_string(10, 156, "OUT   : ");
        ips200_show_string(10, 172, "DUTY  : ");
        ips200_show_string(10, 188, "AGEms : ");

        /* --- 底部提示 --- */
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(5U, (uint16)(ips200_height_max - 16U),
                      (uint16)(ips200_width_max - 10U), "LONG:Exit");

        *menu_full_redraw = 0U;
    }

    /* ================================================================
     * 每帧刷新：只清除并重绘 VALUE 部分（标签区域不动）
     * 所有 HQ 标签 8 chars => val_x = 74
     * ================================================================ */

    /* --- ONLINE --- */
    {
        uint8 online = board_comm_hq_is_online();
        snprintf(val, sizeof(val), "%s", (0U != online) ? "YES" : "NO ");
        if (0U != online)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        }
        menu_ui_show_pad(74, 40, val_w, val);
    }

    /* --- ARM --- */
    {
        uint8 arm = board_comm_hq_get_arm();
        snprintf(val, sizeof(val), "%u", (unsigned)arm);
        if (0U != arm)
        {
            ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        }
        menu_ui_show_pad(74, 56, val_w, val);
    }

    /* --- FRESH --- */
    {
        uint8 fresh = board_comm_hq_get_fresh();
        snprintf(val, sizeof(val), "%u", (unsigned)fresh);
        if (0U != fresh)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        }
        menu_ui_show_pad(74, 72, val_w, val);
    }

    /* --- VALID --- */
    {
        uint8 valid = board_comm_hq_get_valid();
        snprintf(val, sizeof(val), "%u", (unsigned)valid);
        if (0U != valid)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        }
        menu_ui_show_pad(74, 88, val_w, val);
    }

    /* --- EN --- */
    {
        uint8 en = board_comm_hq_get_en();
        snprintf(val, sizeof(val), "%u", (unsigned)en);
        if (0U != en)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        }
        menu_ui_show_pad(74, 108, val_w, val);
    }

    /* --- DRV --- */
    {
        uint8 drv = board_comm_hq_get_drv();
        snprintf(val, sizeof(val), "%u", (unsigned)drv);
        if (0U != drv)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        }
        menu_ui_show_pad(74, 124, val_w, val);
    }

    /* --- CMD --- */
    snprintf(val, sizeof(val), "%u",
             (unsigned)board_comm_hq_get_cmd());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(74, 140, val_w, val);

    /* --- OUT --- */
    snprintf(val, sizeof(val), "%u",
             (unsigned)board_comm_hq_get_out());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(74, 156, val_w, val);

    /* --- DUTY --- */
    snprintf(val, sizeof(val), "%u",
             (unsigned)board_comm_hq_get_duty());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(74, 172, val_w, val);

    /* --- AGEms --- */
    {
        uint32 last = board_comm_hq_get_last_rx_ms();
        if (0U == last)
        {
            snprintf(val, sizeof(val), "--");
        }
        else
        {
            uint32 age = now_ms - last;
            snprintf(val, sizeof(val), "%lu", (unsigned long)age);
        }
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        menu_ui_show_pad(74, 188, val_w, val);
    }
}

/* ---- Encoder 页面（左后轮处理后编码器，只读展示 board_comm ENCL 帧） ---
 *
 * 页面布局（8×16 字体，ips200 240x320，footer = height_max-16 = 304）：
 *   y= 2   "Encoder LR"            黄色标题
 *   y=16   ─────────────────────  灰色分割线
 *   y=22   "Left Rear Only"         青色单编码器提示
 *   y=40   "ONLINE: YES/NO"         绿/红
 *   y=60   "Wheel : Left Rear"      灰色静态标签
 *   y=80   "Count : <n>"            白色，累计 count
 *   y=100  "Dist  : <n> mm"         白色，距离
 *   y=120  "Speed : <n> mm/s"       青色，速度
 *   y=144  "AGEms : <n>"            白色，距上次帧
 *   y=164  "Frame : OK/--"          绿/灰
 *   y=304  "LONG:Exit"              底部灰色提示
 */
static void link_draw_encoder_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms_enc = 0U;
    char     val[64];
    uint32   now_ms   = system_getval_ms();
    uint16   end_x    = (uint16)(ips200_width_max - 10U);
    uint16   val_w;

    /* 限速：Encoder 页面用 50ms 刷新（20Hz），比其它页面的 150ms 更快 */
    #define ENC_PAGE_REFRESH_MS   50U
    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        (now_ms - last_refresh_ms_enc < ENC_PAGE_REFRESH_MS))
    {
        return;
    }
    last_refresh_ms_enc = now_ms;

    /* 全屏重绘 */
    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);

        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 2, "Encoder LR");
        ips200_draw_line(0, 16, ips200_width_max - 1, 16, RGB565_GRAY);

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 22, "Left Rear Only");

        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10,  40, "ONLINE: ");
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        ips200_show_string(10,  60, "Wheel : Left Rear");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10,  80, "Count : ");
        ips200_show_string(10, 100, "Dist  : ");
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 120, "Speed : ");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 144, "AGEms : ");
        ips200_show_string(10, 164, "Frame : ");
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        ips200_show_string(10, 184, "RxOK  : ");
        ips200_show_string(10, 200, "RxFail: ");

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(5U, (uint16)(ips200_height_max - 16U),
                      (uint16)(ips200_width_max - 10U), "LONG:Exit");

        *menu_full_redraw = 0U;
    }

    /* ================================================================
     * 每帧刷新：只清除并重绘 VALUE 部分
     * 所有标签 8 chars ("ONLINE: " 等) => val_x = 74
     * ================================================================ */

    /* --- ONLINE --- */
    {
        uint8 online = board_comm_encl_is_online();
        snprintf(val, sizeof(val), "%s", (0U != online) ? "YES" : "NO ");
        val_w = (uint16)(end_x - 74U);
        if (0U != online)
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        else
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        menu_ui_show_pad(74, 40, val_w, val);
    }

    /* --- Count --- */
    snprintf(val, sizeof(val), "%ld",
             (long)board_comm_encl_get_count());
    val_w = (uint16)(end_x - 74U);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(74, 80, val_w, val);

    /* --- Dist --- */
    snprintf(val, sizeof(val), "%ld mm",
             (long)board_comm_encl_get_dist_mm());
    val_w = (uint16)(end_x - 74U);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(74, 100, val_w, val);

    /* --- Speed --- */
    snprintf(val, sizeof(val), "%ld mm/s",
             (long)board_comm_encl_get_spd_mm_s());
    val_w = (uint16)(end_x - 74U);
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    menu_ui_show_pad(74, 120, val_w, val);

    /* --- AGEms --- 封顶 9999，超时用红色警示 */
    {
        uint32 last = board_comm_encl_get_last_rx_ms();
        uint32 age;
        if (0U == last)
        {
            snprintf(val, sizeof(val), "--");
            ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        }
        else
        {
            age = now_ms - last;
            if (age > 9999U)
                snprintf(val, sizeof(val), ">9999");
            else
                snprintf(val, sizeof(val), "%lu", (unsigned long)age);

            if (age <= 100U)
                ips200_set_color(RGB565_GREEN, RGB565_BLACK);
            else if (age <= BC_ENCL_DISPLAY_WARN_MS)
                ips200_set_color(RGB565_WHITE, RGB565_BLACK);
            else
                ips200_set_color(RGB565_RED, RGB565_BLACK);
        }
        val_w = (uint16)(end_x - 74U);
        menu_ui_show_pad(74, 144, val_w, val);
    }

    /* --- Frame --- */
    {
        uint8 has = board_comm_encl_has_frame();
        snprintf(val, sizeof(val), "%s", (0U != has) ? "OK" : "--");
        val_w = (uint16)(end_x - 74U);
        if (0U != has)
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        else
            ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(74, 164, val_w, val);
    }

    /* --- RxOK (ENCL 解析成功总数) --- */
    snprintf(val, sizeof(val), "%lu",
             (unsigned long)board_comm_encl_get_ok_count());
    val_w = (uint16)(end_x - 74U);
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    menu_ui_show_pad(74, 184, val_w, val);

    /* --- RxFail (ENCL 前缀匹配但解析失败) --- */
    {
        uint32 fail = board_comm_encl_get_fail_count();
        snprintf(val, sizeof(val), "%lu", (unsigned long)fail);
        val_w = (uint16)(end_x - 74U);
        if (0U != fail)
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        else
            ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(74, 200, val_w, val);
    }
}

/* ---- 公共接口 ------------------------------------------------------- */

void menu_link_action_enter(menu_view_ctx_t *ctx, uint8 target_mode)
{
    menu_view_enter(ctx, target_mode);
}

uint8 menu_link_handle_view(menu_view_ctx_t *ctx)
{
    if ((NULL == ctx) || (NULL == ctx->mode) || (LINK_VIEW_NONE == *(ctx->mode))) { return 0U; }

    if ((MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1)) || menu_ui_check_exit_hold(&link_exit_hold_timer, 250U))
    {
        menu_ui_consume_key1();
        *(ctx->mode) = LINK_VIEW_NONE;
        if (NULL != ctx->drain_encoder_events) { ctx->drain_encoder_events(); }
        if (NULL != ctx->reset_dynamic_region) { ctx->reset_dynamic_region(); }
        if (NULL != ctx->request_redraw)       { ctx->request_redraw(1U); }
        return 1U;
    }

    if (LINK_VIEW_DEBUG == *(ctx->mode))
    {
        link_draw_debug_page(ctx->menu_full_redraw);
    }
    else if (LINK_VIEW_HQ_STATUS == *(ctx->mode))
    {
        link_draw_hq_page(ctx->menu_full_redraw);
    }
    else if (LINK_VIEW_ENCODER == *(ctx->mode))
    {
        link_draw_encoder_page(ctx->menu_full_redraw);
    }

    return 1U;
}
