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
#include "menu_ui_utils.h"

#define PEDAL_REFRESH_MS         100U

static uint32 pedal_exit_hold_timer = 0U;

static void pedal_draw_debug_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char val[32];
    uint32 now_ms = system_getval_ms();
    uint16 end_x  = (uint16)(ips200_width_max - 10U);
    uint16 val_w;

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

        /* ---- Static labels (drawn once) ---- */

        /* Section: Throttle (A47) */
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 20, "Throttle (A47)");

        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 34,  "A47 raw : ");
        ips200_show_string(10, 48,  "A47 filt: ");
        ips200_show_string(10, 62,  "A47 %   : ");
        ips200_show_string(10, 76,  "A47 pressed: ");

        /* Section: Throttle Cmd */
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 96, "Throttle Cmd");

        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 110, "valid : ");
        ips200_show_string(10, 124, "enable: ");
        ips200_show_string(10, 138, "cmd   : ");

        /* Section: A45 (idle) */
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 158, "A45 (idle)");

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        ips200_show_string(10, 172, "A45 raw : ");

        /* Fully static line: brake unavailable */
        ips200_set_color(RGB565_RED, RGB565_BLACK);
        ips200_show_string(10, 186, "Brake: unavailable");

        /* Footer */
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(5U, (uint16)(ips200_height_max - 16U),
                       (uint16)(ips200_width_max - 10U), "LONG:Exit");

        *menu_full_redraw = 0U;
    }

    /* ---- Per-frame: clear and redraw value portions only ---- */

    /* A47 raw  (label 11 chars, value x=98) */
    snprintf(val, sizeof(val), "%4u", (unsigned int)pedal_input_get_a47());
    val_w = (uint16)(end_x - 98U);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(98, 34, val_w, val);

    /* A47 filt (label 11 chars, value x=98) */
    snprintf(val, sizeof(val), "%4u", (unsigned int)pedal_input_get_throttle_filtered());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(98, 48, val_w, val);

    /* A47 %    (label 11 chars, value x=98) */
    snprintf(val, sizeof(val), "%4u/1000", (unsigned int)pedal_input_get_throttle_percent());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(98, 62, val_w, val);

    /* A47 pressed (label 15 chars, value x=130) — color-changing value */
    {
        uint8 pressed = pedal_input_get_throttle_pressed();
        uint16 val_w15 = (uint16)(end_x - 130U);
        snprintf(val, sizeof(val), "%s", (0U != pressed) ? "YES" : "no ");
        ips200_set_color((0U != pressed) ? RGB565_GREEN : RGB565_WHITE, RGB565_BLACK);
        menu_ui_show_pad(130, 76, val_w15, val);
    }

    /* valid   (label 9 chars, value x=82) — color-changing value */
    {
        uint8 valid = pedal_input_get_throttle_valid();
        uint16 val_w9 = (uint16)(end_x - 82U);
        snprintf(val, sizeof(val), "%s", (0U != valid) ? "YES" : "NO ");
        ips200_set_color((0U != valid) ? RGB565_GREEN : RGB565_RED, RGB565_BLACK);
        menu_ui_show_pad(82, 110, val_w9, val);
    }

    /* enable  (label 9 chars, value x=82) — color-changing value */
    {
        uint8 enable = pedal_input_get_throttle_enable_request();
        uint16 val_w9 = (uint16)(end_x - 82U);
        snprintf(val, sizeof(val), "%s", (0U != enable) ? "YES" : "no ");
        ips200_set_color((0U != enable) ? RGB565_GREEN : RGB565_WHITE, RGB565_BLACK);
        menu_ui_show_pad(82, 124, val_w9, val);
    }

    /* cmd     (label 9 chars, value x=82) */
    {
        uint16 val_w9 = (uint16)(end_x - 82U);
        snprintf(val, sizeof(val), "%4u/1000", (unsigned int)pedal_input_get_throttle_cmd());
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        menu_ui_show_pad(82, 138, val_w9, val);
    }

    /* A45 raw  (label 11 chars, value x=98, gray) */
    snprintf(val, sizeof(val), "%4u", (unsigned int)pedal_input_get_a45());
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    menu_ui_show_pad(98, 172, val_w, val);
}

/* ---- 公共接口 ---------------------------------------------------------- */

void menu_pedal_action_enter(menu_view_ctx_t *ctx, uint8 target_mode)
{
    menu_view_enter(ctx, target_mode);
}

uint8 menu_pedal_handle_view(menu_view_ctx_t *ctx)
{
    if ((NULL == ctx) || (NULL == ctx->mode) || (PEDAL_VIEW_NONE == *(ctx->mode))) { return 0U; }

    if ((MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1)) || menu_ui_check_exit_hold(&pedal_exit_hold_timer, 250U))
    {
        menu_ui_consume_key1();
        *(ctx->mode) = PEDAL_VIEW_NONE;
        if (NULL != ctx->drain_encoder_events) { ctx->drain_encoder_events(); }
        if (NULL != ctx->reset_dynamic_region) { ctx->reset_dynamic_region(); }
        if (NULL != ctx->request_redraw)       { ctx->request_redraw(1U); }
        return 1U;
    }

    if (PEDAL_VIEW_DEBUG == *(ctx->mode))
    {
        pedal_draw_debug_page(ctx->menu_full_redraw);
    }

    return 1U;
}
