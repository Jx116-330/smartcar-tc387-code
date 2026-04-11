/*********************************************************************************************************************
* File: menu_fusion.c
* Brief: GPS+INS 融合菜单模块实现 —— 融合结果/调试展示页
* Author: JX116
*
* 这个页面显示融合后的位置、速度、创新量和有效性标志，
* 是后续"使用导航位置/速度"功能的入口。
*********************************************************************************************************************/

#include "menu_fusion.h"

#include <stdio.h>
#include <string.h>

#include "zf_common_headfile.h"
#include "zf_device_ips200.h"
#include "icm_gps_fusion.h"
#include "icm_ins.h"

#define FUSION_VIEW_EXIT_HOLD_MS  250U
#define FUSION_VIEW_EXIT_KEY_PIN  P20_2
#define FUSION_REFRESH_MS         100U

/* ---- 内部工具 --------------------------------------------------------- */

static void fusion_fill_rect(uint16 x0, uint16 y0, uint16 x1, uint16 y1, uint16 color)
{
    uint16 y;
    for (y = y0; y <= y1; y++)
    {
        ips200_draw_line(x0, y, x1, y, color);
    }
}

static void fusion_show_pad(uint16 x, uint16 y, uint16 max_width, const char *s)
{
    char buf[64];
    int max_chars = (int)(max_width / 8U);
    int i = 0;

    if (max_chars > (int)(sizeof(buf) - 1)) max_chars = (int)(sizeof(buf) - 1);
    while ((i < max_chars) && (NULL != s) && (s[i] != '\0')) { buf[i] = s[i]; i++; }
    while (i < max_chars) { buf[i++] = ' '; }
    buf[i] = '\0';
    ips200_show_string(x, y, buf);
}

static void fusion_consume_key1_press(void)
{
    my_key_clear_state(MY_KEY_1);
    key_long_press_flag[MY_KEY_1] = close_status;
}

static uint8 fusion_is_exit_hold_triggered(void)
{
    static uint32 key_press_start_ms = 0U;
    uint32 now_ms = system_getval_ms();

    if (MY_KEY_RELEASE_LEVEL != gpio_get_level(FUSION_VIEW_EXIT_KEY_PIN))
    {
        if (0U == key_press_start_ms)
        {
            key_press_start_ms = now_ms;
        }
        else if ((now_ms - key_press_start_ms) >= FUSION_VIEW_EXIT_HOLD_MS)
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

static void fusion_enter_view(fusion_view_mode_t *fusion_mode,
                              fusion_view_mode_t mode,
                              uint8 *menu_full_redraw,
                              void (*drain_encoder_events)(void),
                              void (*request_redraw)(uint8 full_redraw),
                              void (*reset_dynamic_region)(void))
{
    *fusion_mode = mode;
    if (NULL != drain_encoder_events) drain_encoder_events();
    fusion_consume_key1_press();
    if (NULL != reset_dynamic_region) reset_dynamic_region();
    if (NULL != menu_full_redraw) *menu_full_redraw = 1U;
    if (NULL != request_redraw) request_redraw(1U);
}

/* ---- 融合调试页绘制 ---------------------------------------------------- */

static void fusion_draw_debug_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char line[64];
    const icm_gps_fusion_debug_t *d = icm_gps_fusion_get_debug();
    uint32 now_ms = system_getval_ms();
    uint16 col_w  = (uint16)(ips200_width_max - 20U);
    uint16 y = 0U;

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        (now_ms - last_refresh_ms < FUSION_REFRESH_MS))
    {
        return;
    }
    last_refresh_ms = now_ms;

    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);

        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 2, "GPS+INS Fusion");
        ips200_draw_line(0, 16, ips200_width_max - 1, 16, RGB565_GRAY);

        *menu_full_redraw = 0U;
    }

    /* K1 short press: reset origin */
    if (my_key_get_state(MY_KEY_1) == MY_KEY_SHORT_PRESS)
    {
        icm_gps_fusion_reset_origin();
        icm_ins_reset_position();
        icm_ins_reset_velocity();
        fusion_consume_key1_press();
    }

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    /* ---- 状态标志行 ---- */
    y = 20U;
    snprintf(line, sizeof(line), "ORI:%s EN:%s ACT:%s",
             d->origin_ready    ? "Y" : "-",
             d->fusion_enable   ? "Y" : "-",
             d->correction_active ? "Y" : "-");
    fusion_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    fusion_show_pad(10, y, col_w, line);

    y = 34U;
    snprintf(line, sizeof(line), "GPS:%s SPD:%s HDG:%s N:%lu",
             d->gps_valid         ? "Y" : "-",
             d->gps_speed_valid   ? "Y" : "-",
             d->gps_heading_valid ? "Y" : "-",
             (unsigned long)d->gps_update_count);
    fusion_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    fusion_show_pad(10, y, col_w, line);

    /* ---- 位置：融合后 / GPS / INS ---- */
    y = 50U;
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(10, y, "Position (m)");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    y = 64U;
    {
        float fx = 0.0f, fy = 0.0f;
        icm_gps_fusion_get_position(&fx, &fy);
        snprintf(line, sizeof(line), "F:%+7.2f %+7.2f", (double)fx, (double)fy);
    }
    fusion_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    fusion_show_pad(10, y, col_w, line);

    y = 78U;
    snprintf(line, sizeof(line), "G:%+7.2f %+7.2f", (double)d->gps_x_m, (double)d->gps_y_m);
    fusion_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    fusion_show_pad(10, y, col_w, line);

    y = 92U;
    snprintf(line, sizeof(line), "I:%+7.2f %+7.2f", (double)d->ins_x_m, (double)d->ins_y_m);
    fusion_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    fusion_show_pad(10, y, col_w, line);

    /* ---- 速度：融合后 / GPS / INS ---- */
    y = 108U;
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(10, y, "Velocity (m/s)");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    y = 122U;
    {
        float fvx = 0.0f, fvy = 0.0f;
        icm_gps_fusion_get_velocity(&fvx, &fvy);
        snprintf(line, sizeof(line), "F:%+6.2f %+6.2f", (double)fvx, (double)fvy);
    }
    fusion_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    fusion_show_pad(10, y, col_w, line);

    y = 136U;
    snprintf(line, sizeof(line), "G:%+6.2f %+6.2f", (double)d->gps_vx_ms, (double)d->gps_vy_ms);
    fusion_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    fusion_show_pad(10, y, col_w, line);

    y = 150U;
    snprintf(line, sizeof(line), "I:%+6.2f %+6.2f", (double)d->ins_vx_ms, (double)d->ins_vy_ms);
    fusion_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    fusion_show_pad(10, y, col_w, line);

    /* ---- 创新量 ---- */
    y = 166U;
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(10, y, "Innovation");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    y = 180U;
    snprintf(line, sizeof(line), "dP:%+6.2f %+6.2f",
             (double)d->innov_px_m, (double)d->innov_py_m);
    fusion_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    fusion_show_pad(10, y, col_w, line);

    y = 194U;
    snprintf(line, sizeof(line), "dV:%+6.2f %+6.2f",
             (double)d->innov_vx_ms, (double)d->innov_vy_ms);
    fusion_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    fusion_show_pad(10, y, col_w, line);

    /* ---- 统计 ---- */
    y = 210U;
    snprintf(line, sizeof(line), "ZUPT:%s Corr:%lu",
             icm_ins_is_stationary() ? "ON" : "--",
             (unsigned long)d->correction_count);
    fusion_fill_rect(10U, y, (uint16)(ips200_width_max - 10U), (uint16)(y + 12U), RGB565_BLACK);
    fusion_show_pad(10, y, col_w, line);

    /* ---- 底部提示 ---- */
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    fusion_show_pad(5, (uint16)(ips200_height_max - 14U),
                    (uint16)(ips200_width_max - 10U), "K1:Rst Origin LONG:Exit");
}

/* ---- 公共接口 ---------------------------------------------------------- */

void menu_fusion_action_debug(fusion_view_mode_t *fusion_mode,
                              uint8 *menu_full_redraw,
                              void (*drain_encoder_events)(void),
                              void (*request_redraw)(uint8 full_redraw),
                              void (*reset_dynamic_region)(void))
{
    fusion_enter_view(fusion_mode, FUSION_VIEW_DEBUG, menu_full_redraw,
                      drain_encoder_events, request_redraw, reset_dynamic_region);
}

uint8 menu_fusion_handle_view(fusion_view_mode_t *fusion_mode,
                              uint8 *menu_full_redraw,
                              void (*drain_encoder_events)(void),
                              void (*request_redraw)(uint8 full_redraw),
                              void (*reset_dynamic_region)(void))
{
    if ((NULL == fusion_mode) || (FUSION_VIEW_NONE == *fusion_mode)) return 0U;

    /* 长按退出 */
    if ((MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1)) || fusion_is_exit_hold_triggered())
    {
        fusion_consume_key1_press();
        *fusion_mode = FUSION_VIEW_NONE;
        if (NULL != drain_encoder_events) drain_encoder_events();
        if (NULL != reset_dynamic_region) reset_dynamic_region();
        if (NULL != request_redraw) request_redraw(1U);
        return 1U;
    }

    if (FUSION_VIEW_DEBUG == *fusion_mode)
    {
        fusion_draw_debug_page(menu_full_redraw);
    }

    return 1U;
}
