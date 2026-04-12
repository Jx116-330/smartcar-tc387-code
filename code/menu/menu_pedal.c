/*********************************************************************************************************************
* File: menu_pedal.c
* Brief: 踏板调试菜单实现
*        - 油门源已切到 A47：显示 A47 raw / filt / %  / pressed
*        - 控制层输出：throttle valid / enable / cmd
*        - A45 现在仅作为空闲通道观察（不再是油门源）
*        - 刹车：当前硬件未提供可用通道，标记为 "unavailable"
* Author: JX116
*********************************************************************************************************************/

#include "menu_pedal.h"

#include <stdio.h>

#include "zf_common_headfile.h"
#include "zf_device_ips200.h"
#include "pedal_input.h"

#define PEDAL_VIEW_EXIT_HOLD_MS  250U
#define PEDAL_VIEW_EXIT_KEY_PIN  P20_2
#define PEDAL_REFRESH_MS         100U

static void pedal_fill_rect(uint16 x0, uint16 y0, uint16 x1, uint16 y1, uint16 color)
{
    uint16 y;
    for (y = y0; y <= y1; y++)
    {
        ips200_draw_line(x0, y, x1, y, color);
    }
}

static void pedal_show_pad(uint16 x, uint16 y, uint16 max_width, const char *s)
{
    char buf[64];
    int max_chars = (int)(max_width / 8U);
    int i = 0;
    if (max_chars > (int)(sizeof(buf) - 1)) { max_chars = (int)(sizeof(buf) - 1); }
    while ((i < max_chars) && (NULL != s) && (s[i] != '\0')) { buf[i] = s[i]; i++; }
    while (i < max_chars) { buf[i++] = ' '; }
    buf[i] = '\0';
    ips200_show_string(x, y, buf);
}

static void pedal_consume_key1_press(void)
{
    my_key_clear_state(MY_KEY_1);
    key_long_press_flag[MY_KEY_1] = close_status;
}

static uint8 pedal_is_exit_hold_triggered(void)
{
    static uint32 key_press_start_ms = 0U;
    uint32 now_ms = system_getval_ms();

    if (MY_KEY_RELEASE_LEVEL != gpio_get_level(PEDAL_VIEW_EXIT_KEY_PIN))
    {
        if (0U == key_press_start_ms)
        {
            key_press_start_ms = now_ms;
        }
        else if ((now_ms - key_press_start_ms) >= PEDAL_VIEW_EXIT_HOLD_MS)
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

static void pedal_draw_debug_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char line[64];
    uint32 now_ms = system_getval_ms();
    uint16 col_w  = (uint16)(ips200_width_max - 20U);
    uint16 y;

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        (now_ms - last_refresh_ms < PEDAL_REFRESH_MS))
    {
        return;
    }
    last_refresh_ms = now_ms;

    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 2, "Pedal Debug");
        ips200_draw_line(0, 16, ips200_width_max - 1, 16, RGB565_GRAY);
        *menu_full_redraw = 0U;
    }

    /* ---- Throttle (A47) primitive block —— 当前油门源 ---- */
    y = 20U;
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(10, y, "Throttle (A47)");

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    y = 34U;
    snprintf(line, sizeof(line), "A47 raw : %4u",
             (unsigned int)pedal_input_get_a47());
    pedal_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    pedal_show_pad(10, y, col_w, line);

    y = 48U;
    snprintf(line, sizeof(line), "A47 filt: %4u",
             (unsigned int)pedal_input_get_throttle_filtered());
    pedal_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    pedal_show_pad(10, y, col_w, line);

    y = 62U;
    snprintf(line, sizeof(line), "A47 %%   : %4u/1000",
             (unsigned int)pedal_input_get_throttle_percent());
    pedal_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    pedal_show_pad(10, y, col_w, line);

    y = 76U;
    {
        uint8 pressed = pedal_input_get_throttle_pressed();
        snprintf(line, sizeof(line), "A47 pressed: %s",
                 (0U != pressed) ? "YES" : "no ");
        pedal_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        if (0U != pressed)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        }
        pedal_show_pad(10, y, col_w, line);
    }

    /* ---- Control-layer command block (Step 3) ---- */
    y = 96U;
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(10, y, "Throttle Cmd");

    y = 110U;
    {
        uint8 valid = pedal_input_get_throttle_valid();
        snprintf(line, sizeof(line), "valid : %s",
                 (0U != valid) ? "YES" : "NO ");
        pedal_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        if (0U != valid)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        }
        pedal_show_pad(10, y, col_w, line);
    }

    y = 124U;
    {
        uint8 enable = pedal_input_get_throttle_enable_request();
        snprintf(line, sizeof(line), "enable: %s",
                 (0U != enable) ? "YES" : "no ");
        pedal_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
        if (0U != enable)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        else
        {
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        }
        pedal_show_pad(10, y, col_w, line);
    }

    y = 138U;
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    snprintf(line, sizeof(line), "cmd   : %4u/1000",
             (unsigned int)pedal_input_get_throttle_cmd());
    pedal_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    pedal_show_pad(10, y, col_w, line);

    /* ---- A45 idle 观察 + Brake unavailable ----
     * A45 现在不再接油门，这里只把 raw 打出来便于排线/确认；
     * 本板仍未提供可用刹车通道，Brake 保持红色 unavailable。 */
    y = 158U;
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(10, y, "A45 (idle)");

    y = 172U;
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    snprintf(line, sizeof(line), "A45 raw : %4u",
             (unsigned int)pedal_input_get_a45());
    pedal_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    pedal_show_pad(10, y, col_w, line);

    y = 186U;
    ips200_set_color(RGB565_RED, RGB565_BLACK);
    snprintf(line, sizeof(line), "Brake: unavailable");
    pedal_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    pedal_show_pad(10, y, col_w, line);

    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    pedal_show_pad(5U, (uint16)(ips200_height_max - 16U),
                   (uint16)(ips200_width_max - 10U), "LONG:Exit");
}

/* ---- 公共接口 ---------------------------------------------------------- */

void menu_pedal_action_debug(pedal_view_mode_t *pedal_mode,
                             uint8 *menu_full_redraw,
                             void (*drain_encoder_events)(void),
                             void (*request_redraw)(uint8 full_redraw),
                             void (*reset_dynamic_region)(void))
{
    *pedal_mode = PEDAL_VIEW_DEBUG;
    if (NULL != drain_encoder_events) { drain_encoder_events(); }
    pedal_consume_key1_press();
    if (NULL != reset_dynamic_region) { reset_dynamic_region(); }
    if (NULL != menu_full_redraw)     { *menu_full_redraw = 1U; }
    if (NULL != request_redraw)       { request_redraw(1U); }
}

uint8 menu_pedal_handle_view(pedal_view_mode_t *pedal_mode,
                             uint8 *menu_full_redraw,
                             void (*drain_encoder_events)(void),
                             void (*request_redraw)(uint8 full_redraw),
                             void (*reset_dynamic_region)(void))
{
    if ((NULL == pedal_mode) || (PEDAL_VIEW_NONE == *pedal_mode)) { return 0U; }

    if ((MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1)) || pedal_is_exit_hold_triggered())
    {
        pedal_consume_key1_press();
        *pedal_mode = PEDAL_VIEW_NONE;
        if (NULL != drain_encoder_events) { drain_encoder_events(); }
        if (NULL != reset_dynamic_region) { reset_dynamic_region(); }
        if (NULL != request_redraw)       { request_redraw(1U); }
        return 1U;
    }

    if (PEDAL_VIEW_DEBUG == *pedal_mode)
    {
        pedal_draw_debug_page(menu_full_redraw);
    }

    return 1U;
}
