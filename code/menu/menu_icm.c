/*********************************************************************************************************************
* File: menu_icm.c
* Brief: ICM42688 IMU 菜单模块实现
*        1. IMU Status  —— 显示传感器连接状态、当前配置（量程/采样率/滤波器）
*        2. Raw Data    —— 实时刷新 6 轴原始 LSB 值与物理量
*********************************************************************************************************************/

#include "menu_icm.h"
#include "zf_common_headfile.h"
#include "icm42688.h"
#include "zf_device_ips200.h"
#include "MyKey.h"
#include "MyEncoder.h"
#include <stdio.h>
#include <string.h>

/* ---- 视图模式 ---- */
typedef enum
{
    ICM_VIEW_NONE = 0U,
    ICM_VIEW_STATUS,
    ICM_VIEW_RAW,
} icm_view_mode_t;

static icm_view_mode_t icm_view_mode = ICM_VIEW_NONE;
static uint8 icm_raw_header_drawn = 0U;

/* ---- 辅助：字符串查表 ---- */
static const char *gyro_fsr_str(GYRO_FSR fsr)
{
    switch (fsr)
    {
        case GYRO_15_625DPS: return "15.6dps";
        case GYRO_31_25DPS:  return "31.3dps";
        case GYRO_62_5DPS:   return "62.5dps";
        case GYRO_125DPS:    return "125dps";
        case GYRO_250DPS:    return "250dps";
        case GYRO_500DPS:    return "500dps";
        case GYRO_1000DPS:   return "1000dps";
        case GYRO_2000DPS:   return "2000dps";
        default:             return "???";
    }
}

static const char *gyro_odr_str(GYRO_ODR odr)
{
    switch (odr)
    {
        case GYRO_ODR_12_5HZ:  return "12.5Hz";
        case GYRO_ODR_25HZ:    return "25Hz";
        case GYRO_ODR_50HZ:    return "50Hz";
        case GYRO_ODR_100HZ:   return "100Hz";
        case GYRO_ODR_200HZ:   return "200Hz";
        case GYRO_ODR_500HZ:   return "500Hz";
        case GYRO_ODR_1000HZ:  return "1000Hz";
        case GYRO_ODR_2000HZ:  return "2000Hz";
        case GYRO_ODR_4000HZ:  return "4000Hz";
        case GYRO_ODR_8000HZ:  return "8000Hz";
        case GYRO_ODR_16000HZ: return "16000Hz";
        case GYRO_ODR_32000HZ: return "32000Hz";
        default:               return "???";
    }
}

static const char *acc_fsr_str(ACC_FSR fsr)
{
    switch (fsr)
    {
        case ACC_2G:  return "2G";
        case ACC_4G:  return "4G";
        case ACC_8G:  return "8G";
        case ACC_16G: return "16G";
        default:      return "???";
    }
}

static const char *acc_odr_str(ACC_ODR odr)
{
    switch (odr)
    {
        case ACC_ODR_12_5HZ:  return "12.5Hz";
        case ACC_ODR_25HZ:    return "25Hz";
        case ACC_ODR_50HZ:    return "50Hz";
        case ACC_ODR_100HZ:   return "100Hz";
        case ACC_ODR_200HZ:   return "200Hz";
        case ACC_ODR_500HZ:   return "500Hz";
        case ACC_ODR_1000HZ:  return "1000Hz";
        case ACC_ODR_2000HZ:  return "2000Hz";
        case ACC_ODR_4000HZ:  return "4000Hz";
        case ACC_ODR_8000HZ:  return "8000Hz";
        case ACC_ODR_16000HZ: return "16000Hz";
        case ACC_ODR_32000HZ: return "32000Hz";
        default:              return "???";
    }
}

static const char *filter_order_str(Filter_Order order)
{
    switch (order)
    {
        case _1st: return "1st";
        case _2st: return "2nd";
        case _3st: return "3rd";
        default:   return "???";
    }
}

/* ---- 屏幕绘制辅助 ---- */
static void fill_rect(uint16 x0, uint16 y0, uint16 x1, uint16 y1, uint16 color)
{
    uint16 y;
    if (x1 >= ips200_width_max) x1 = ips200_width_max - 1;
    if (y1 >= ips200_height_max) y1 = ips200_height_max - 1;
    for (y = y0; y <= y1; y++)
    {
        ips200_draw_line(x0, y, x1, y, color);
    }
}

static void draw_label_value(uint16 y, const char *label, const char *value)
{
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    ips200_show_string(5, y, label);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(110, y, value);
}

static void draw_title_bar(const char *title)
{
    ips200_full(RGB565_BLACK);
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(10, 6, title);
    ips200_draw_line(0, 24, ips200_width_max - 1, 24, RGB565_GRAY);
}

static void draw_footer_hint(void)
{
    uint16 fy = ips200_height_max - 18;
    fill_rect(0, fy, ips200_width_max - 1, ips200_height_max - 1, RGB565_BLACK);
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    ips200_show_string(5, fy, "LONG K1: Back");
}

/* ---- Status 页面 ---- */
static void icm_draw_status(void)
{
    char buf[40];
    uint16 y = 32;

    draw_title_bar("ICM42688 Status");

    /* Gyro 配置 */
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(5, y, "-- Gyroscope --");
    y += 18;
    draw_label_value(y, "Range:", gyro_fsr_str(ICM42688_CONFIG.GYRO_FSR));
    y += 16;
    draw_label_value(y, "ODR:", gyro_odr_str(ICM42688_CONFIG.GYRO_ODR));
    y += 16;
    sprintf(buf, "%s order", filter_order_str(ICM42688_CONFIG.Gyro_Filter_Order));
    draw_label_value(y, "Filter:", buf);

    /* Acc 配置 */
    y += 22;
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(5, y, "-- Accelerometer --");
    y += 18;
    draw_label_value(y, "Range:", acc_fsr_str(ICM42688_CONFIG.ACC_FSR));
    y += 16;
    draw_label_value(y, "ODR:", acc_odr_str(ICM42688_CONFIG.ACC_ODR));
    y += 16;
    sprintf(buf, "%s order", filter_order_str(ICM42688_CONFIG.Acc_Filter_Order));
    draw_label_value(y, "Filter:", buf);

    /* Bias 状态 */
    y += 22;
    draw_label_value(y, "Bias:", (ICM42688_CONFIG.Bias_Option == Bias_On_Chip_On) ? "ON" : "OFF");

    draw_footer_hint();
}

/* ---- Raw Data 页面：每帧完整绘制标题 + 实时数据 ---- */

static void icm_draw_raw_page(void)
{
    char buf[40];
    uint16 y = 0U;

    /* 数据采样已移到后台周期更新，这里只显示当前缓存值。 */

    /* 首次进入时画静态框架 */
    if (0U == icm_raw_header_drawn)
    {
        draw_title_bar("ICM42688 Raw Data");

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(5, 30, "-- Gyro (dps) --");

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(5, 96, "-- Accel (g) --");

        draw_footer_hint();
        icm_raw_header_drawn = 1U;
    }

    /* Gyro 三轴：物理量 + 原始 LSB */
    y = 48;
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);

    sprintf(buf, "GX:%+8.2f  R:%+6d", ICM42688.gyro_x, ICM42688_RAW.gyro_x_lsb);
    fill_rect(5, y, ips200_width_max - 1, y + 15, RGB565_BLACK);
    ips200_show_string(5, y, buf);

    y += 16;
    sprintf(buf, "GY:%+8.2f  R:%+6d", ICM42688.gyro_y, ICM42688_RAW.gyro_y_lsb);
    fill_rect(5, y, ips200_width_max - 1, y + 15, RGB565_BLACK);
    ips200_show_string(5, y, buf);

    y += 16;
    sprintf(buf, "GZ:%+8.2f  R:%+6d", ICM42688.gyro_z, ICM42688_RAW.gyro_z_lsb);
    fill_rect(5, y, ips200_width_max - 1, y + 15, RGB565_BLACK);
    ips200_show_string(5, y, buf);

    /* Accel 三轴：物理量 + 原始 LSB */
    y = 114;
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);

    sprintf(buf, "AX:%+8.4f  R:%+6d", ICM42688.acc_x, ICM42688_RAW.acc_x_lsb);
    fill_rect(5, y, ips200_width_max - 1, y + 15, RGB565_BLACK);
    ips200_show_string(5, y, buf);

    y += 16;
    sprintf(buf, "AY:%+8.4f  R:%+6d", ICM42688.acc_y, ICM42688_RAW.acc_y_lsb);
    fill_rect(5, y, ips200_width_max - 1, y + 15, RGB565_BLACK);
    ips200_show_string(5, y, buf);

    y += 16;
    sprintf(buf, "AZ:%+8.4f  R:%+6d", ICM42688.acc_z, ICM42688_RAW.acc_z_lsb);
    fill_rect(5, y, ips200_width_max - 1, y + 15, RGB565_BLACK);
    ips200_show_string(5, y, buf);
}

/* ---- 菜单 Action 回调 ---- */
static void icm_action_status(void)
{
    icm_view_mode = ICM_VIEW_STATUS;
    icm_draw_status();
}

static void icm_action_raw(void)
{
    icm_view_mode = ICM_VIEW_RAW;
    icm_raw_header_drawn = 0U;
    icm_draw_raw_page();
}

/* ---- 菜单页面定义 ---- */
static MenuItem icm_items[] = {
    {"1. IMU Status",   icm_action_status, NULL},
    {"2. Raw Data",     icm_action_raw,    NULL},
};

MenuPage icm_menu = {
    "ICM42688",
    icm_items,
    sizeof(icm_items) / sizeof(MenuItem),
    NULL
};

/* ---- 对外接口 ---- */
uint8 menu_icm_is_active(void)
{
    return (ICM_VIEW_NONE != icm_view_mode) ? 1U : 0U;
}

uint8 menu_icm_handle_view(void)
{
    if (ICM_VIEW_NONE == icm_view_mode)
    {
        return 0U;
    }

    /* 长按返回 */
    if (my_key_get_state(MY_KEY_1) == MY_KEY_LONG_PRESS)
    {
        my_key_clear_state(MY_KEY_1);
        icm_view_mode = ICM_VIEW_NONE;
        icm_raw_header_drawn = 0U;
        menu_request_full_redraw();
        return 1U;
    }

    /* 短按在 status 页无额外动作；在 raw 页也无额外动作 */
    if (my_key_get_state(MY_KEY_1) == MY_KEY_SHORT_PRESS)
    {
        my_key_clear_state(MY_KEY_1);
    }

    /* Raw Data 页面实时刷新 */
    if (ICM_VIEW_RAW == icm_view_mode)
    {
        icm_draw_raw_page();
    }

    return 1U;
}
