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
#include "MyEncoder.h"    /* switch_encoder_change_num */
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

        /* Section: Brake (A24) */
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 190, "Brake (A24)");

        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 204, "A24 raw : ");
        ips200_show_string(10, 218, "BrkFilt : ");
        ips200_show_string(10, 232, "BrkPct  : ");
        ips200_show_string(10, 246, "BrkPress: ");
        ips200_show_string(10, 260, "BrkValid: ");
        ips200_show_string(10, 274, "BrkCmd  : ");

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

    snprintf(val, sizeof(val), "%4u", (unsigned int)pedal_input_get_a24());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(98, 204, val_w, val);

    snprintf(val, sizeof(val), "%4u", (unsigned int)pedal_input_get_brake_filtered());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(98, 218, val_w, val);

    snprintf(val, sizeof(val), "%4u/1000", (unsigned int)pedal_input_get_brake_percent());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(98, 232, val_w, val);

    {
        uint8 pressed = pedal_input_get_brake_pressed();
        uint16 val_w15 = (uint16)(end_x - 98U);
        snprintf(val, sizeof(val), "%s", (0U != pressed) ? "YES" : "no ");
        ips200_set_color((0U != pressed) ? RGB565_GREEN : RGB565_WHITE, RGB565_BLACK);
        menu_ui_show_pad(98, 246, val_w15, val);
    }

    {
        uint8 valid = pedal_input_get_brake_valid();
        uint16 val_w15 = (uint16)(end_x - 98U);
        snprintf(val, sizeof(val), "%s", (0U != valid) ? "YES" : "NO ");
        ips200_set_color((0U != valid) ? RGB565_GREEN : RGB565_RED, RGB565_BLACK);
        menu_ui_show_pad(98, 260, val_w15, val);
    }

    snprintf(val, sizeof(val), "%4u/1000", (unsigned int)pedal_input_get_brake_cmd());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(98, 274, val_w, val);
}

/* ---- 驱动控制页 --------------------------------------------------------
 * 提供菜单级的驱动安全控制：
 *   - Drive Enable: 总开关，K1 短按切换 ON/OFF
 *   - PWM Limit:    旋转编码器调节 0~100%（步进 5%，对应内部 0~1000）
 *   - 实时显示当前油门命令、最终出力占空比
 *
 * 页面布局 (8x16 字体, 240x320):
 *   y=  2   "Drive Control"        Yellow
 *   y= 16   ────────────           Gray
 *   y= 22   "Safety"               Cyan
 *   y= 40   "Drive : ON/OFF"       Green/Red    K1 切换
 *   y= 60   "Limit : XX%"          White        旋钮调节
 *   y= 84   "Realtime"             Cyan
 *   y=100   "Cmd   : XXXX/1000"    White        油门踏板命令
 *   y=120   "Enable: YES/no"       Green/White  enable_request AND drive_enable
 *   y=140   "Valid : YES/NO"       Green/Red
 *   y=160   "Output: XX.X%"        Yellow       最终 = cmd * limit / 1000 / 10
 *   y=304   "K1:ON/OFF  ENC:Limit  LONG:Exit"
 */

static void pedal_draw_drive_ctrl_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms_ctrl = 0U;
    char     val[32];
    uint32   now_ms = system_getval_ms();
    uint16   end_x  = (uint16)(ips200_width_max - 10U);
    uint16   val_w  = (uint16)(end_x - 74U);

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        (now_ms - last_refresh_ms_ctrl < PEDAL_REFRESH_MS))
    {
        return;
    }
    last_refresh_ms_ctrl = now_ms;

    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);

        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 2, "Drive Control");
        ips200_draw_line(0, 16, ips200_width_max - 1, 16, RGB565_GRAY);

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 22, "Safety");

        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10,  40, "Drive : ");
        ips200_show_string(10,  60, "Limit : ");

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 84, "Realtime");

        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 100, "Cmd   : ");
        ips200_show_string(10, 120, "Enable: ");
        ips200_show_string(10, 140, "Valid : ");
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 160, "Output: ");

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(5U, (uint16)(ips200_height_max - 16U),
                      (uint16)(ips200_width_max - 10U), "K1:ON/OFF ENC:Limit LONG:Exit");

        *menu_full_redraw = 0U;
    }

    /* Drive ON/OFF */
    {
        uint8 drv = pedal_input_get_drive_enable();
        snprintf(val, sizeof(val), "%s", (0U != drv) ? "ON " : "OFF");
        if (0U != drv)
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        else
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        menu_ui_show_pad(74, 40, val_w, val);
    }

    /* Limit XX% */
    {
        uint16 limit = pedal_input_get_pwm_limit();
        snprintf(val, sizeof(val), "%u%%", (unsigned)(limit / 10U));
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        menu_ui_show_pad(74, 60, val_w, val);
    }

    /* Cmd */
    snprintf(val, sizeof(val), "%u/1000", (unsigned)pedal_input_get_throttle_cmd());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(74, 100, val_w, val);

    /* Enable (enable_request AND drive_enable) */
    {
        uint8 en = pedal_input_get_throttle_enable_request()
                   && pedal_input_get_drive_enable();
        snprintf(val, sizeof(val), "%s", (0U != en) ? "YES" : "no ");
        if (0U != en)
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        else
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        menu_ui_show_pad(74, 120, val_w, val);
    }

    /* Valid */
    {
        uint8 vld = pedal_input_get_throttle_valid();
        snprintf(val, sizeof(val), "%s", (0U != vld) ? "YES" : "NO ");
        if (0U != vld)
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        else
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        menu_ui_show_pad(74, 140, val_w, val);
    }

    /* Output = cmd * limit / 1000 → 显示为百分比 */
    {
        uint16 cmd   = pedal_input_get_throttle_cmd();
        uint16 limit = pedal_input_get_pwm_limit();
        uint32 out   = (uint32)cmd * (uint32)limit / 1000U;  /* 0~1000 */
        snprintf(val, sizeof(val), "%u.%u%%", (unsigned)(out / 10U), (unsigned)(out % 10U));
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        menu_ui_show_pad(74, 160, val_w, val);
    }
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

    /* Drive Ctrl 页: K1 切换驱动开关，旋转编码器调 limit */
    if (PEDAL_VIEW_DRIVE_CTRL == *(ctx->mode))
    {
        if (MY_KEY_SHORT_PRESS == my_key_get_state(MY_KEY_1))
        {
            menu_ui_consume_key1();
            pedal_input_set_drive_enable((uint8)(0U == pedal_input_get_drive_enable()));
            if (NULL != ctx->menu_full_redraw) *(ctx->menu_full_redraw) = 1U;
            return 1U;
        }

        /* 旋钮调节 PWM 上限：每格 ±5%（50/1000），钳位 0~1000
         * 必须主动调 If_Switch_Encoder_Change() 把 pending_steps 转成 change_num，
         * 因为 view 激活时 menu_update_selection_from_encoder 不运行 */
        if (If_Switch_Encoder_Change())
        {
            int16 cur = (int16)pedal_input_get_pwm_limit();
            cur -= (int16)switch_encoder_change_num * 10;   /* 反向，每格 1%（10/1000） */
            if (cur < 0)    cur = 0;
            if (cur > 1000) cur = 1000;
            pedal_input_set_pwm_limit((uint16)cur);
        }
    }

    if (PEDAL_VIEW_DEBUG == *(ctx->mode))
    {
        pedal_draw_debug_page(ctx->menu_full_redraw);
    }
    else if (PEDAL_VIEW_DRIVE_CTRL == *(ctx->mode))
    {
        pedal_draw_drive_ctrl_page(ctx->menu_full_redraw);
    }

    return 1U;
}
