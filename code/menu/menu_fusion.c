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
#include "menu_ui_utils.h"

#define FUSION_REFRESH_MS         100U

static uint32 fusion_exit_hold_timer = 0U;

/* fusion_enter_view 已被 menu_view_enter(ctx, mode) 替代 */

/* ---- 融合调试页绘制 ---------------------------------------------------- */

static void fusion_draw_debug_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char val[48];
    const icm_gps_fusion_debug_t *d = icm_gps_fusion_get_debug();
    uint32 now_ms = system_getval_ms();
    uint16 val_w  = (uint16)(ips200_width_max - 36U);  /* 26 起始的值宽 */
    uint16 val_w3 = (uint16)(ips200_width_max - 44U);  /* 34 起始的值宽 */

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        (now_ms - last_refresh_ms < FUSION_REFRESH_MS))
    {
        return;
    }
    last_refresh_ms = now_ms;

    /* ---- 静态标签（仅 full_redraw 绘制一次）---- */
    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);

        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 2, "GPS+INS Fusion");
        ips200_draw_line(0, 16, ips200_width_max - 1, 16, RGB565_GRAY);

        /* section headers */
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 50, "Position (m)");
        ips200_show_string(10, 108, "Velocity (m/s)");
        ips200_show_string(10, 166, "Innovation");

        /* 数据行标签 */
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        ips200_show_string(10, 64, "F:");
        ips200_show_string(10, 122, "F:");
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 78, "G:");
        ips200_show_string(10, 136, "G:");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 92, "I:");
        ips200_show_string(10, 150, "I:");
        ips200_show_string(10, 180, "dP:");
        ips200_show_string(10, 194, "dV:");

        /* footer */
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        ips200_show_string(5, (uint16)(ips200_height_max - 14U), "K1:Rst Origin LONG:Exit");

        *menu_full_redraw = 0U;
    }

    /* K1 short press: reset origin */
    if (my_key_get_state(MY_KEY_1) == MY_KEY_SHORT_PRESS)
    {
        icm_gps_fusion_reset_origin();
        icm_ins_reset_position();
        icm_ins_reset_velocity();
        menu_ui_consume_key1();
    }

    /* ---- 状态标志行（混合内容，仅覆写不清黑）---- */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    snprintf(val, sizeof(val), "ORI:%s EN:%s ACT:%s",
             d->origin_ready    ? "Y" : "-",
             d->fusion_enable   ? "Y" : "-",
             d->correction_active ? "Y" : "-");
    menu_ui_show_pad(10, 20, (uint16)(ips200_width_max - 20U), val);

    snprintf(val, sizeof(val), "GPS:%s SPD:%s HDG:%s N:%lu",
             d->gps_valid         ? "Y" : "-",
             d->gps_speed_valid   ? "Y" : "-",
             d->gps_heading_valid ? "Y" : "-",
             (unsigned long)d->gps_update_count);
    menu_ui_show_pad(10, 34, (uint16)(ips200_width_max - 20U), val);

    /* ---- 位置数值（仅值区 x=26 起）---- */
    {
        float fx = 0.0f, fy = 0.0f;
        icm_gps_fusion_get_position(&fx, &fy);
        snprintf(val, sizeof(val), "%+7.2f %+7.2f", (double)fx, (double)fy);
    }
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    menu_ui_show_pad(26, 64, val_w, val);

    snprintf(val, sizeof(val), "%+7.2f %+7.2f", (double)d->gps_x_m, (double)d->gps_y_m);
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    menu_ui_show_pad(26, 78, val_w, val);

    snprintf(val, sizeof(val), "%+7.2f %+7.2f", (double)d->ins_x_m, (double)d->ins_y_m);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(26, 92, val_w, val);

    /* ---- 速度数值（仅值区 x=26 起）---- */
    {
        float fvx = 0.0f, fvy = 0.0f;
        icm_gps_fusion_get_velocity(&fvx, &fvy);
        snprintf(val, sizeof(val), "%+6.2f %+6.2f", (double)fvx, (double)fvy);
    }
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    menu_ui_show_pad(26, 122, val_w, val);

    snprintf(val, sizeof(val), "%+6.2f %+6.2f", (double)d->gps_vx_ms, (double)d->gps_vy_ms);
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    menu_ui_show_pad(26, 136, val_w, val);

    snprintf(val, sizeof(val), "%+6.2f %+6.2f", (double)d->ins_vx_ms, (double)d->ins_vy_ms);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(26, 150, val_w, val);

    /* ---- 创新量数值（仅值区 x=34 起）---- */
    snprintf(val, sizeof(val), "%+6.2f %+6.2f",
             (double)d->innov_px_m, (double)d->innov_py_m);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(34, 180, val_w3, val);

    snprintf(val, sizeof(val), "%+6.2f %+6.2f",
             (double)d->innov_vx_ms, (double)d->innov_vy_ms);
    menu_ui_show_pad(34, 194, val_w3, val);

    /* ---- 统计行（混合内容，仅覆写不清黑）---- */
    snprintf(val, sizeof(val), "ZUPT:%s Corr:%lu",
             icm_ins_is_stationary() ? "ON" : "--",
             (unsigned long)d->correction_count);
    menu_ui_show_pad(10, 210, (uint16)(ips200_width_max - 20U), val);
}

/* ---- 公共接口 ---------------------------------------------------------- */

void menu_fusion_action_enter(menu_view_ctx_t *ctx, uint8 target_mode)
{
    menu_view_enter(ctx, target_mode);
}

uint8 menu_fusion_handle_view(menu_view_ctx_t *ctx)
{
    if ((NULL == ctx) || (NULL == ctx->mode) || (FUSION_VIEW_NONE == *(ctx->mode))) return 0U;

    /* 长按退出 */
    if ((MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1)) || menu_ui_check_exit_hold(&fusion_exit_hold_timer, 250U))
    {
        menu_ui_consume_key1();
        *(ctx->mode) = FUSION_VIEW_NONE;
        if (NULL != ctx->drain_encoder_events) ctx->drain_encoder_events();
        if (NULL != ctx->reset_dynamic_region) ctx->reset_dynamic_region();
        if (NULL != ctx->request_redraw) ctx->request_redraw(1U);
        return 1U;
    }

    if (FUSION_VIEW_DEBUG == *(ctx->mode))
    {
        fusion_draw_debug_page(ctx->menu_full_redraw);
    }

    return 1U;
}
