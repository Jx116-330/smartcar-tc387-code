/*********************************************************************************************************************
* File: menu_icm.c
* Brief: ICM42688 菜单模块实现 —— 原始数据页
* Author: JX116
*********************************************************************************************************************/

#include "menu_icm.h"

#include <stdio.h>

#include "zf_common_headfile.h"
#include "zf_device_ips200.h"
#include "ICM42688.h"
#include "icm_attitude.h"

#define ICM_GYRO_BIAS_CALIB_SAMPLES 2000U
#define ICM_VIEW_EXIT_HOLD_MS       250U
#define ICM_VIEW_EXIT_KEY_PIN       P20_2

/* ---- 内部工具 --------------------------------------------------------- */

static void icm_fill_rect(uint16 x0, uint16 y0, uint16 x1, uint16 y1, uint16 color)
{
    uint16 y;
    for (y = y0; y <= y1; y++)
    {
        ips200_draw_line(x0, y, x1, y, color);
    }
}

static void icm_show_pad(uint16 x, uint16 y, uint16 max_width, const char *s)
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

static void icm_consume_key1_press(void)
{
    my_key_clear_state(MY_KEY_1);
    key_long_press_flag[MY_KEY_1] = close_status;
}

static uint8 icm_is_exit_hold_triggered(void)
{
    static uint32 key_press_start_ms = 0U;
    uint32 now_ms = system_getval_ms();

    if (MY_KEY_RELEASE_LEVEL != gpio_get_level(ICM_VIEW_EXIT_KEY_PIN))
    {
        if (0U == key_press_start_ms)
        {
            key_press_start_ms = now_ms;
        }
        else if ((now_ms - key_press_start_ms) >= ICM_VIEW_EXIT_HOLD_MS)
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

static void icm_enter_view(icm_view_mode_t *icm_mode,
                           icm_view_mode_t mode,
                           uint8 *menu_full_redraw,
                           void (*drain_encoder_events)(void),
                           void (*request_redraw)(uint8 full_redraw),
                           void (*reset_dynamic_region)(void))
{
    *icm_mode = mode;
    if (NULL != drain_encoder_events) drain_encoder_events();
    icm_consume_key1_press();
    if (NULL != reset_dynamic_region) reset_dynamic_region();
    if (NULL != menu_full_redraw) *menu_full_redraw = 1U;
    if (NULL != request_redraw) request_redraw(1U);
}

/* ---- 原始数据页绘制 ---------------------------------------------------- */

static void icm_draw_raw_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char line[48];
    uint32 now_ms = system_getval_ms();
    uint16 col_w  = (uint16)(ips200_width_max - 20U);

    /* 限速：全刷时立即绘制，增量刷新 200 ms 一次 */
    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        (now_ms - last_refresh_ms < 200U))
    {
        return;
    }
    last_refresh_ms = now_ms;

    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        /* 清屏并绘制静态框架 */
        ips200_full(RGB565_BLACK);

        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 10, "ICM Raw Data");
        ips200_draw_line(0, 30, ips200_width_max - 1, 30, RGB565_GRAY);

        /* 分类标题 */
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 38, "Accel (g)");
        ips200_draw_line(10, 52, (uint16)(ips200_width_max - 10U), 52, RGB565_GRAY);

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 108, "Gyro (dps)");
        ips200_draw_line(10, 122, (uint16)(ips200_width_max - 10U), 122, RGB565_GRAY);

        /* 页脚 */
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        icm_show_pad(5, (uint16)(ips200_height_max - 20U), (uint16)(ips200_width_max - 10U), "LONG: Exit");

        *menu_full_redraw = 0U;
    }

    /* 动态数据区域（全量刷新后也会走这里更新数值） */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    /* 加速度 X / Y */
    snprintf(line, sizeof(line), "Ax:%+8.4f  Ay:%+8.4f", (double)icm42688_acc_x, (double)icm42688_acc_y);
    icm_fill_rect(10U, 56U, (uint16)(ips200_width_max - 10U), 70U, RGB565_BLACK);
    icm_show_pad(10, 56, col_w, line);

    /* 加速度 Z */
    snprintf(line, sizeof(line), "Az:%+8.4f", (double)icm42688_acc_z);
    icm_fill_rect(10U, 78U, (uint16)(ips200_width_max - 10U), 92U, RGB565_BLACK);
    icm_show_pad(10, 78, col_w, line);

    /* 温度占位行（如后续需要可换成温度读取） */
    icm_fill_rect(10U, 100U, (uint16)(ips200_width_max - 10U), 106U, RGB565_BLACK);

    /* 陀螺仪 X / Y */
    snprintf(line, sizeof(line), "Gx:%+8.2f  Gy:%+8.2f", (double)icm42688_gyro_x, (double)icm42688_gyro_y);
    icm_fill_rect(10U, 126U, (uint16)(ips200_width_max - 10U), 140U, RGB565_BLACK);
    icm_show_pad(10, 126, col_w, line);

    /* 陀螺仪 Z */
    snprintf(line, sizeof(line), "Gz:%+8.2f", (double)icm42688_gyro_z);
    icm_fill_rect(10U, 148U, (uint16)(ips200_width_max - 10U), 162U, RGB565_BLACK);
    icm_show_pad(10, 148, col_w, line);
}

static void icm_draw_attitude_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    static uint32 last_update_count = 0U;
    static uint32 last_rate_time_ms = 0U;
    static float update_hz = 0.0f;
    char line[48];
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    float acc_norm_g = 0.0f;
    uint32 now_ms = system_getval_ms();
    uint32 update_count = icm_attitude_get_update_count();
    uint16 col_w  = (uint16)(ips200_width_max - 20U);

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        (now_ms - last_refresh_ms < 100U))
    {
        return;
    }
    last_refresh_ms = now_ms;

    if ((0U == last_rate_time_ms) || ((now_ms - last_rate_time_ms) >= 500U))
    {
        uint32 delta_ms = (0U == last_rate_time_ms) ? 0U : (now_ms - last_rate_time_ms);
        uint32 delta_count = update_count - last_update_count;
        if (delta_ms > 0U)
        {
            update_hz = ((float)delta_count * 1000.0f) / (float)delta_ms;
        }
        last_rate_time_ms = now_ms;
        last_update_count = update_count;
    }

    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);

        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 10, "ICM Attitude");
        ips200_draw_line(0, 30, ips200_width_max - 1, 30, RGB565_GRAY);

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 38, "Euler Angle (deg)");
        ips200_draw_line(10, 52, (uint16)(ips200_width_max - 10U), 52, RGB565_GRAY);
        ips200_show_string(10, 132, "Solver Status");
        ips200_draw_line(10, 146, (uint16)(ips200_width_max - 10U), 146, RGB565_GRAY);

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        icm_show_pad(5, (uint16)(ips200_height_max - 20U), (uint16)(ips200_width_max - 10U), "Yaw drifts, LONG: Exit");

        *menu_full_redraw = 0U;
    }

    icm_attitude_get_euler(&roll_deg, &pitch_deg, &yaw_deg);
    acc_norm_g = icm_attitude_get_acc_norm_g();

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    snprintf(line, sizeof(line), "Roll :%+8.2f", (double)roll_deg);
    icm_fill_rect(10U, 60U, (uint16)(ips200_width_max - 10U), 74U, RGB565_BLACK);
    icm_show_pad(10, 60, col_w, line);

    snprintf(line, sizeof(line), "Pitch:%+8.2f", (double)pitch_deg);
    icm_fill_rect(10U, 82U, (uint16)(ips200_width_max - 10U), 96U, RGB565_BLACK);
    icm_show_pad(10, 82, col_w, line);

    snprintf(line, sizeof(line), "Yaw  :%+8.2f", (double)yaw_deg);
    icm_fill_rect(10U, 104U, (uint16)(ips200_width_max - 10U), 118U, RGB565_BLACK);
    icm_show_pad(10, 104, col_w, line);

    snprintf(line, sizeof(line), "AccNorm:%6.3fg", (double)acc_norm_g);
    icm_fill_rect(10U, 154U, (uint16)(ips200_width_max - 10U), 168U, RGB565_BLACK);
    icm_show_pad(10, 154, col_w, line);

    snprintf(line, sizeof(line), "Upd:%6.1fHz Cnt:%lu", (double)update_hz, (unsigned long)update_count);
    icm_fill_rect(10U, 176U, (uint16)(ips200_width_max - 10U), 190U, RGB565_BLACK);
    icm_show_pad(10, 176, col_w, line);
}

static void icm_draw_gyro_bias_calib_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char line[48];
    float bias_x = 0.0f;
    float bias_y = 0.0f;
    float bias_z = 0.0f;
    uint32 now_ms = system_getval_ms();
    uint32 sample_count = 0U;
    uint32 target_count = 0U;
    uint16 col_w = (uint16)(ips200_width_max - 20U);

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        (now_ms - last_refresh_ms < 100U))
    {
        return;
    }
    last_refresh_ms = now_ms;

    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);

        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 10, "Gyro Bias Calib");
        ips200_draw_line(0, 30, ips200_width_max - 1, 30, RGB565_GRAY);

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        icm_show_pad(10, 40, (uint16)(ips200_width_max - 20U), "Keep the car still");
        icm_show_pad(10, 56, (uint16)(ips200_width_max - 20U), "K1: Restart  LONG: Back");

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 82, "Calibration Status");
        ips200_draw_line(10, 96, (uint16)(ips200_width_max - 10U), 96, RGB565_GRAY);
        ips200_show_string(10, 146, "Gyro Bias (dps)");
        ips200_draw_line(10, 160, (uint16)(ips200_width_max - 10U), 160, RGB565_GRAY);

        *menu_full_redraw = 0U;
    }

    icm_attitude_get_gyro_bias(&bias_x, &bias_y, &bias_z);
    icm_attitude_get_gyro_bias_calibration_progress(&sample_count, &target_count);

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    if (icm_attitude_is_gyro_bias_calibrating())
    {
        snprintf(line, sizeof(line), "Status: CALIBRATING");
    }
    else if (icm_attitude_is_gyro_bias_valid())
    {
        snprintf(line, sizeof(line), "Status: DONE");
    }
    else
    {
        snprintf(line, sizeof(line), "Status: NOT READY");
    }
    icm_fill_rect(10U, 104U, (uint16)(ips200_width_max - 10U), 118U, RGB565_BLACK);
    icm_show_pad(10, 104, col_w, line);

    snprintf(line, sizeof(line), "Samples:%4lu/%4lu", (unsigned long)sample_count, (unsigned long)target_count);
    icm_fill_rect(10U, 124U, (uint16)(ips200_width_max - 10U), 138U, RGB565_BLACK);
    icm_show_pad(10, 124, col_w, line);

    snprintf(line, sizeof(line), "Bx:%+8.3f", (double)bias_x);
    icm_fill_rect(10U, 168U, (uint16)(ips200_width_max - 10U), 182U, RGB565_BLACK);
    icm_show_pad(10, 168, col_w, line);

    snprintf(line, sizeof(line), "By:%+8.3f Bz:%+8.3f", (double)bias_y, (double)bias_z);
    icm_fill_rect(10U, 188U, (uint16)(ips200_width_max - 10U), 202U, RGB565_BLACK);
    icm_show_pad(10, 188, col_w, line);
}

/* ---- 公共接口 ---------------------------------------------------------- */

void menu_icm_action_raw_data(icm_view_mode_t *icm_mode,
                              uint8 *menu_full_redraw,
                              void (*drain_encoder_events)(void),
                              void (*request_redraw)(uint8 full_redraw),
                              void (*reset_dynamic_region)(void))
{
    icm_enter_view(icm_mode, ICM_VIEW_RAW, menu_full_redraw,
                   drain_encoder_events, request_redraw, reset_dynamic_region);
}

void menu_icm_action_attitude(icm_view_mode_t *icm_mode,
                              uint8 *menu_full_redraw,
                              void (*drain_encoder_events)(void),
                              void (*request_redraw)(uint8 full_redraw),
                              void (*reset_dynamic_region)(void))
{
    icm_enter_view(icm_mode, ICM_VIEW_ATTITUDE, menu_full_redraw,
                   drain_encoder_events, request_redraw, reset_dynamic_region);
}

void menu_icm_action_gyro_bias_calib(icm_view_mode_t *icm_mode,
                                     uint8 *menu_full_redraw,
                                     void (*drain_encoder_events)(void),
                                     void (*request_redraw)(uint8 full_redraw),
                                     void (*reset_dynamic_region)(void))
{
    icm_attitude_start_gyro_bias_calibration(ICM_GYRO_BIAS_CALIB_SAMPLES);
    icm_enter_view(icm_mode, ICM_VIEW_GYRO_BIAS_CALIB, menu_full_redraw,
                   drain_encoder_events, request_redraw, reset_dynamic_region);
}

uint8 menu_icm_handle_view(icm_view_mode_t *icm_mode,
                           uint8 *menu_full_redraw,
                           void (*drain_encoder_events)(void),
                           void (*request_redraw)(uint8 full_redraw),
                           void (*reset_dynamic_region)(void))
{
    if ((NULL == icm_mode) || (ICM_VIEW_NONE == *icm_mode)) return 0U;

    /* 长按退出 */
    if ((MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1)) || icm_is_exit_hold_triggered())
    {
        icm_consume_key1_press();
        if (ICM_VIEW_GYRO_BIAS_CALIB == *icm_mode)
        {
            icm_attitude_cancel_gyro_bias_calibration();
        }
        *icm_mode = ICM_VIEW_NONE;
        if (NULL != drain_encoder_events) drain_encoder_events();
        if (NULL != reset_dynamic_region) reset_dynamic_region();
        if (NULL != request_redraw) request_redraw(1U);
        return 1U;
    }

    if ((ICM_VIEW_GYRO_BIAS_CALIB == *icm_mode) &&
        (MY_KEY_SHORT_PRESS == my_key_get_state(MY_KEY_1)))
    {
        icm_consume_key1_press();
        icm_attitude_start_gyro_bias_calibration(ICM_GYRO_BIAS_CALIB_SAMPLES);
        if (NULL != menu_full_redraw) *menu_full_redraw = 1U;
        return 1U;
    }

    if (ICM_VIEW_RAW == *icm_mode)
    {
        icm_draw_raw_page(menu_full_redraw);
    }
    else if (ICM_VIEW_ATTITUDE == *icm_mode)
    {
        icm_draw_attitude_page(menu_full_redraw);
    }
    else if (ICM_VIEW_GYRO_BIAS_CALIB == *icm_mode)
    {
        icm_draw_gyro_bias_calib_page(menu_full_redraw);
    }

    return 1U;
}
