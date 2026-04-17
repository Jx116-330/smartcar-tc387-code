/*********************************************************************************************************************
* File: menu_icm.c
* Brief: ICM42688 菜单模块实现 —— 原始数据页
* Author: JX116
*********************************************************************************************************************/

#include "menu_icm.h"

#include <stdio.h>
#include <string.h>

#include "zf_common_headfile.h"
#include "zf_device_ips200.h"
#include "ICM42688.h"
#include "display_gps.h"
#include "icm_attitude.h"
#include "icm_ins.h"
#include "ins_record.h"
#include "board_comm.h"
#include "encoder_odom.h"
#include "rear_right_encoder.h"
#include "encoder_odom_right.h"
#include "menu_ui_utils.h"

#define ICM_GYRO_BIAS_CALIB_SAMPLES 20000U
#define ICM_INS_MAP_REFRESH_MS      250U
#define ICM_TRACK_LINE_COLOR        RGB565_WHITE
#define ICM_TRACK_START_COLOR       RGB565_GREEN
#define ICM_TRACK_END_COLOR         RGB565_CYAN
#define ICM_TRACK_CURRENT_COLOR     RGB565_RED

static double icm_track_x_points[DISPLAY_POINT_MAX];
static double icm_track_y_points[DISPLAY_POINT_MAX];
static uint8 icm_ins_map_border_dirty = 1U;
static uint8 icm_ins_map_bounds_valid = 0U;
static double icm_ins_map_min_x = 0.0;
static double icm_ins_map_max_x = 0.0;
static double icm_ins_map_min_y = 0.0;
static double icm_ins_map_max_y = 0.0;
static uint16 icm_ins_map_rendered_point_count = 0U;
static uint16 icm_ins_map_area_x = 0U;
static uint16 icm_ins_map_area_y = 0U;
static uint16 icm_ins_map_area_w = 0U;
static uint16 icm_ins_map_area_h = 0U;
static screen_point icm_ins_map_last_current_point = {0, 0};
static uint8 icm_ins_map_last_current_valid = 0U;
static uint16 icm_ins_map_last_current_restore_color = RGB565_BLACK;
static screen_point icm_ins_map_last_end_point = {0, 0};
static uint8 icm_ins_map_last_end_valid = 0U;
static uint16 icm_ins_map_last_end_restore_color = ICM_TRACK_LINE_COLOR;
static char icm_ins_map_line0_cache[48] = "";
static char icm_ins_map_line1_cache[48] = "";
static char icm_ins_map_line2_cache[48] = "";
static char icm_ins_map_hint_cache[48] = "";

/* ---- 内部工具 --------------------------------------------------------- */

static uint32 icm_exit_hold_timer = 0U;

/* icm_enter_view 已被 menu_view_enter(ctx, mode) 替代 */

static void icm_draw_box_border(uint16 x0, uint16 y0, uint16 width, uint16 height, uint16 color)
{
    uint16 x1;
    uint16 y1;

    if (width < 2U || height < 2U)
    {
        return;
    }

    x1 = (uint16)(x0 + width - 1U);
    y1 = (uint16)(y0 + height - 1U);

    ips200_draw_line(x0, y0, x1, y0, color);
    ips200_draw_line(x0, y1, x1, y1, color);
    ips200_draw_line(x0, y0, x0, y1, color);
    ips200_draw_line(x1, y0, x1, y1, color);
}

static uint8 icm_map_point_in_area(const screen_point *point, uint16 width, uint16 height)
{
    if (NULL == point)
    {
        return 0U;
    }

    return (point->x >= 0 &&
            point->y >= 0 &&
            point->x < (int16)width &&
            point->y < (int16)height) ? 1U : 0U;
}

static void icm_draw_map_marker(uint16 map_x,
                                uint16 map_y,
                                uint16 map_w,
                                uint16 map_h,
                                const screen_point *point,
                                uint16 color)
{
    static const int8 x_offsets[9] = {0, -1, 1, 0, 0, -2, 2, 0, 0};
    static const int8 y_offsets[9] = {0, 0, 0, -1, 1, 0, 0, -2, 2};
    uint16 i;

    if (!icm_map_point_in_area(point, map_w, map_h))
    {
        return;
    }

    for (i = 0U; i < 9U; i++)
    {
        screen_point draw_point = {
            (int16)(point->x + x_offsets[i]),
            (int16)(point->y + y_offsets[i])
        };

        if (icm_map_point_in_area(&draw_point, map_w, map_h))
        {
            ips200_draw_point((uint16)(map_x + draw_point.x),
                              (uint16)(map_y + draw_point.y),
                              color);
        }
    }
}

static void icm_draw_map_square_marker(uint16 map_x,
                                       uint16 map_y,
                                       uint16 map_w,
                                       uint16 map_h,
                                       const screen_point *point,
                                       uint16 color)
{
    static const int8 x_offsets[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int8 y_offsets[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    uint16 i;

    if (!icm_map_point_in_area(point, map_w, map_h))
    {
        return;
    }

    for (i = 0U; i < 8U; i++)
    {
        screen_point draw_point = {
            (int16)(point->x + x_offsets[i]),
            (int16)(point->y + y_offsets[i])
        };

        if (icm_map_point_in_area(&draw_point, map_w, map_h))
        {
            ips200_draw_point((uint16)(map_x + draw_point.x),
                              (uint16)(map_y + draw_point.y),
                              color);
        }
    }
}

static void icm_copy_text(char *dest, uint32 dest_size, const char *source)
{
    uint32 index = 0U;
    const char *text = (NULL != source) ? source : "";

    if ((NULL == dest) || (0U == dest_size))
    {
        return;
    }

    while ((index + 1U) < dest_size && text[index] != '\0')
    {
        dest[index] = text[index];
        index++;
    }
    dest[index] = '\0';
}

static double icm_abs_double(double value)
{
    return (value < 0.0) ? -value : value;
}

static uint8 icm_screen_point_equal(const screen_point *lhs, const screen_point *rhs)
{
    if ((NULL == lhs) || (NULL == rhs))
    {
        return 0U;
    }

    return ((lhs->x == rhs->x) && (lhs->y == rhs->y)) ? 1U : 0U;
}

static void icm_ins_map_reset_cache(void)
{
    icm_ins_map_border_dirty = 1U;
    icm_ins_map_bounds_valid = 0U;
    icm_ins_map_min_x = 0.0;
    icm_ins_map_max_x = 0.0;
    icm_ins_map_min_y = 0.0;
    icm_ins_map_max_y = 0.0;
    icm_ins_map_rendered_point_count = 0U;
    icm_ins_map_area_x = 0U;
    icm_ins_map_area_y = 0U;
    icm_ins_map_area_w = 0U;
    icm_ins_map_area_h = 0U;
    icm_ins_map_last_current_valid = 0U;
    icm_ins_map_last_current_restore_color = RGB565_BLACK;
    icm_ins_map_last_end_valid = 0U;
    icm_ins_map_last_end_restore_color = ICM_TRACK_LINE_COLOR;
    icm_ins_map_line0_cache[0] = '\0';
    icm_ins_map_line1_cache[0] = '\0';
    icm_ins_map_line2_cache[0] = '\0';
    icm_ins_map_hint_cache[0] = '\0';
}

static void icm_ins_map_set_area(uint16 map_x, uint16 map_y, uint16 map_w, uint16 map_h)
{
    if ((icm_ins_map_area_x != map_x) ||
        (icm_ins_map_area_y != map_y) ||
        (icm_ins_map_area_w != map_w) ||
        (icm_ins_map_area_h != map_h))
    {
        icm_ins_map_border_dirty = 1U;
    }

    icm_ins_map_area_x = map_x;
    icm_ins_map_area_y = map_y;
    icm_ins_map_area_w = map_w;
    icm_ins_map_area_h = map_h;
}

static uint8 icm_ins_map_bounds_changed(double min_x, double max_x, double min_y, double max_y)
{
    if (!icm_ins_map_bounds_valid)
    {
        return 1U;
    }

    if (icm_abs_double(min_x - icm_ins_map_min_x) > EPSILON) return 1U;
    if (icm_abs_double(max_x - icm_ins_map_max_x) > EPSILON) return 1U;
    if (icm_abs_double(min_y - icm_ins_map_min_y) > EPSILON) return 1U;
    if (icm_abs_double(max_y - icm_ins_map_max_y) > EPSILON) return 1U;

    return 0U;
}

static void icm_ins_map_update_bounds_cache(double min_x, double max_x, double min_y, double max_y)
{
    icm_ins_map_min_x = min_x;
    icm_ins_map_max_x = max_x;
    icm_ins_map_min_y = min_y;
    icm_ins_map_max_y = max_y;
    icm_ins_map_bounds_valid = 1U;
}

static void icm_ins_map_clear_inner_area(uint16 map_x, uint16 map_y, uint16 map_w, uint16 map_h)
{
    if ((map_w <= 2U) || (map_h <= 2U))
    {
        return;
    }

    menu_ui_fill_rect((uint16)(map_x + 1U),
                  (uint16)(map_y + 1U),
                  (uint16)(map_x + map_w - 2U),
                  (uint16)(map_y + map_h - 2U),
                  RGB565_BLACK);
}

static void icm_ins_map_clear_last_current_marker(uint16 map_x, uint16 map_y, uint16 map_w, uint16 map_h)
{
    if (icm_ins_map_last_current_valid)
    {
        icm_draw_map_marker(map_x,
                            map_y,
                            map_w,
                            map_h,
                            &icm_ins_map_last_current_point,
                            icm_ins_map_last_current_restore_color);
        icm_ins_map_last_current_valid = 0U;
    }
}

static void icm_ins_map_clear_last_end_marker(uint16 map_x, uint16 map_y, uint16 map_w, uint16 map_h)
{
    if (icm_ins_map_last_end_valid)
    {
        icm_draw_map_square_marker(map_x,
                                   map_y,
                                   map_w,
                                   map_h,
                                   &icm_ins_map_last_end_point,
                                   icm_ins_map_last_end_restore_color);
        icm_ins_map_last_end_valid = 0U;
    }
}

static uint8 icm_ins_map_xy_to_screen_point(uint16 map_w,
                                            uint16 map_h,
                                            double min_x,
                                            double max_x,
                                            double min_y,
                                            double max_y,
                                            double x,
                                            double y,
                                            screen_point *point)
{
    double temp_x[1];
    double temp_y[1];

    if (NULL == point)
    {
        return 0U;
    }

    temp_x[0] = x;
    temp_y[0] = y;
    gps_set_display_area(0, 0, (int16)map_w, (int16)map_h);
    gps_set_xy_bounds(min_x, max_x, min_y, max_y);
    xy_to_screen(temp_x, temp_y, 1);
    gps_clear_xy_bounds();
    *point = screen_point_data[0];
    return 1U;
}

static void icm_ins_map_update_text_line(uint16 x,
                                         uint16 y,
                                         uint16 max_width,
                                         const char *text,
                                         char *cache,
                                         uint16 color,
                                         uint8 force_redraw)
{
    if ((NULL == cache) || (NULL == text))
    {
        return;
    }

    if (!force_redraw && (0 == strcmp(cache, text)))
    {
        return;
    }

    ips200_set_color(color, RGB565_BLACK);
    menu_ui_show_pad(x, y, max_width, text);
    icm_copy_text(cache, 48U, text);
}

static uint8 icm_ins_map_should_full_redraw(uint8 has_bounds,
                                            double min_x,
                                            double max_x,
                                            double min_y,
                                            double max_y)
{
    if (icm_ins_map_border_dirty)
    {
        return 1U;
    }
    if (ins_record_get_point_count() < icm_ins_map_rendered_point_count)
    {
        return 1U;
    }
    if (has_bounds)
    {
        return icm_ins_map_bounds_changed(min_x, max_x, min_y, max_y);
    }

    return (icm_ins_map_bounds_valid || (icm_ins_map_rendered_point_count > 0U)) ? 1U : 0U;
}

static uint16 icm_collect_ins_track_samples(double *x_points, double *y_points, uint16 max_points)
{
    uint16 total_count;
    uint16 sample_count;
    uint16 i;

    if ((NULL == x_points) || (NULL == y_points) || (0U == max_points))
    {
        return 0U;
    }

    total_count = ins_record_get_point_count();
    if (0U == total_count)
    {
        return 0U;
    }

    sample_count = total_count;
    if (sample_count > max_points)
    {
        sample_count = max_points;
    }

    if (sample_count == total_count)
    {
        for (i = 0U; i < sample_count; i++)
        {
            const ins_record_point_t *point = ins_record_get_point_ptr(i);

            if (NULL == point)
            {
                sample_count = i;
                break;
            }

            x_points[i] = (double)point->px_m;
            y_points[i] = (double)point->py_m;
        }
    }
    else if (sample_count <= 1U)
    {
        const ins_record_point_t *point = ins_record_get_point_ptr(0U);

        if (NULL == point)
        {
            return 0U;
        }

        x_points[0] = (double)point->px_m;
        y_points[0] = (double)point->py_m;
        sample_count = 1U;
    }
    else
    {
        uint32 total_span = (uint32)(total_count - 1U);
        uint32 sample_span = (uint32)(sample_count - 1U);

        for (i = 0U; i < sample_count; i++)
        {
            uint32 index = (i * total_span + (sample_span / 2U)) / sample_span;
            const ins_record_point_t *point = ins_record_get_point_ptr((uint16)index);

            if (NULL == point)
            {
                sample_count = i;
                break;
            }

            x_points[i] = (double)point->px_m;
            y_points[i] = (double)point->py_m;
        }
    }

    return sample_count;
}

static uint8 icm_collect_ins_track_bounds(double *min_x,
                                          double *max_x,
                                          double *min_y,
                                          double *max_y,
                                          uint8 include_current,
                                          float current_px,
                                          float current_py)
{
    uint16 point_count = ins_record_get_point_count();
    uint16 i;
    uint8 has_value = 0U;

    if ((NULL == min_x) || (NULL == max_x) || (NULL == min_y) || (NULL == max_y))
    {
        return 0U;
    }

    for (i = 0U; i < point_count; i++)
    {
        const ins_record_point_t *point = ins_record_get_point_ptr(i);
        double px;
        double py;

        if (NULL == point)
        {
            continue;
        }

        px = (double)point->px_m;
        py = (double)point->py_m;

        if (!has_value)
        {
            *min_x = px;
            *max_x = px;
            *min_y = py;
            *max_y = py;
            has_value = 1U;
        }
        else
        {
            if (px < *min_x) *min_x = px;
            if (px > *max_x) *max_x = px;
            if (py < *min_y) *min_y = py;
            if (py > *max_y) *max_y = py;
        }
    }

    if (include_current)
    {
        double px = (double)current_px;
        double py = (double)current_py;

        if (!has_value)
        {
            *min_x = px;
            *max_x = px;
            *min_y = py;
            *max_y = py;
            has_value = 1U;
        }
        else
        {
            if (px < *min_x) *min_x = px;
            if (px > *max_x) *max_x = px;
            if (py < *min_y) *min_y = py;
            if (py > *max_y) *max_y = py;
        }
    }

    return has_value;
}

/* ---- Encoder / INS 融合状态页 ----
 * 页面布局（8×16 字体，ips200 240x320，footer = height_max-16 = 304）：
 *   y= 2   "Encoder / INS"          黄色标题
 *   y=16   ─────────────────────   灰色分割线
 *   y=22   "Odometry Source"        青色分组头
 *   y=40   "ONLINE: YES/NO"          绿/红
 *   y=60   "Speed : xxx mm/s"        青色（编码器原始速度）
 *   y=80   "Dist  : xxx mm"          白色（编码器累计距离）
 *   y=104  "INS Fusion"              青色分组头
 *   y=120  "Active: YES/NO"          绿/灰
 *   y=140  "EncSpd: x.xx m/s"       白色
 *   y=160  "INSSpd: x.xx m/s"       白色
 *   y=180  "Odom X: x.xx m"         灰色
 *   y=200  "Odom Y: x.xx m"         灰色
 *   y=304  "LONG:Exit"               底部灰色提示
 */
#define ICM_ENC_REFRESH_MS    50U

#if 0
static void icm_draw_encoder_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms_enc_icm = 0U;
    char     val[64];
    uint32   now_ms = system_getval_ms();
    uint16   end_x  = (uint16)(ips200_width_max - 10U);
    uint16   val_w  = (uint16)(end_x - 74U);   /* 8-char labels */

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        (now_ms - last_refresh_ms_enc_icm < ICM_ENC_REFRESH_MS))
    {
        return;
    }
    last_refresh_ms_enc_icm = now_ms;

    /* ---- 全屏重绘 ---- */
    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);

        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 2, "Encoder / INS");
        ips200_draw_line(0, 16, ips200_width_max - 1, 16, RGB565_GRAY);

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 22, "Odometry Source");

        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10,  40, "ONLINE: ");
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10,  60, "Speed : ");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10,  80, "Dist  : ");

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 104, "INS Fusion");

        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 120, "Active: ");
        ips200_show_string(10, 140, "EncSpd: ");
        ips200_show_string(10, 160, "INSSpd: ");
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        ips200_show_string(10, 180, "Odom X: ");
        ips200_show_string(10, 200, "Odom Y: ");

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(5U, (uint16)(ips200_height_max - 16U),
                      (uint16)(ips200_width_max - 10U), "K1:Fusion  LONG:Exit");

        *menu_full_redraw = 0U;
    }

    /* ---- 每帧值刷新 ---- */

    /* ONLINE */
    {
        uint8 online = board_comm_encl_is_online();
        snprintf(val, sizeof(val), "%s", (0U != online) ? "YES" : "NO ");
        if (0U != online)
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        else
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        menu_ui_show_pad(74, 40, val_w, val);
    }

    /* Speed (mm/s) */
    snprintf(val, sizeof(val), "%ld mm/s",
             (long)board_comm_encl_get_spd_mm_s());
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    menu_ui_show_pad(74, 60, val_w, val);

    /* Dist (mm) */
    snprintf(val, sizeof(val), "%ld mm",
             (long)board_comm_encl_get_dist_mm());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(74, 80, val_w, val);

    /* Active: OFF(融合关) / IDLE(开但未激活) / YES(正在融合) */
    {
        uint8 en  = encoder_odom_is_enabled();
        uint8 act = encoder_odom_is_active();
        if (0U == en)
        {
            snprintf(val, sizeof(val), "OFF");
            ips200_set_color(RGB565_RED, RGB565_BLACK);
        }
        else if (0U == act)
        {
            snprintf(val, sizeof(val), "IDLE");
            ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        }
        else
        {
            snprintf(val, sizeof(val), "YES");
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        }
        menu_ui_show_pad(74, 120, val_w, val);
    }

    /* EncSpd (m/s) */
    snprintf(val, sizeof(val), "%.2f m/s",
             (double)encoder_odom_get_speed_ms());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(74, 140, val_w, val);

    /* INSSpd (m/s) */
    snprintf(val, sizeof(val), "%.2f m/s",
             (double)icm_ins_get_speed_ms());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(74, 160, val_w, val);

    /* Odom X */
    snprintf(val, sizeof(val), "%.2f m",
             (double)encoder_odom_get_px_m());
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    menu_ui_show_pad(74, 180, val_w, val);

    /* Odom Y */
    snprintf(val, sizeof(val), "%.2f m",
             (double)encoder_odom_get_py_m());
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    menu_ui_show_pad(74, 200, val_w, val);
}
#endif

static void icm_draw_encoder_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms_enc_icm = 0U;
    char val[64];
    uint32 now_ms = system_getval_ms();
    uint16 end_x = (uint16)(ips200_width_max - 10U);
    uint16 left_w = (uint16)(150U - 70U);
    uint16 right_w = (uint16)(end_x - 150U);
    uint32 left_last_ms;
    uint32 right_last_ms;
    uint32 left_age_ms;
    uint32 right_age_ms;

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        (now_ms - last_refresh_ms_enc_icm < ICM_ENC_REFRESH_MS))
    {
        return;
    }
    last_refresh_ms_enc_icm = now_ms;

    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);

        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 2, "Encoder LR");
        ips200_draw_line(0, 16, ips200_width_max - 1, 16, RGB565_GRAY);

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(70, 22, "LEFT");
        ips200_show_string(150, 22, "RIGHT");

        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10,  38, "Online:");
        ips200_show_string(10,  54, "Count :");
        ips200_show_string(10,  70, "Dist  :");
        ips200_show_string(10,  86, "Speed :");
        ips200_show_string(10, 102, "OdomPx:");
        ips200_show_string(10, 118, "OdomPy:");
        ips200_show_string(10, 134, "OdomSp:");
        ips200_show_string(10, 150, "Active:");
        ips200_show_string(10, 166, "AGEms :");

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(5U, (uint16)(ips200_height_max - 16U),
                      (uint16)(ips200_width_max - 10U), "K1:Fusion  LONG:Exit");

        *menu_full_redraw = 0U;
    }

    left_last_ms = board_comm_encl_get_last_rx_ms();
    right_last_ms = rear_right_get_last_rx_ms();
    left_age_ms = (0U != left_last_ms) ? (now_ms - left_last_ms) : 0U;
    right_age_ms = (0U != right_last_ms) ? (now_ms - right_last_ms) : 0U;

    {
        uint8 online = board_comm_encl_is_online();
        snprintf(val, sizeof(val), "%s", (0U != online) ? "YES" : "NO ");
        ips200_set_color((0U != online) ? RGB565_GREEN : RGB565_RED, RGB565_BLACK);
        menu_ui_show_pad(70, 38, left_w, val);

        online = rear_right_is_online();
        snprintf(val, sizeof(val), "%s", (0U != online) ? "YES" : "NO ");
        ips200_set_color((0U != online) ? RGB565_GREEN : RGB565_RED, RGB565_BLACK);
        menu_ui_show_pad(150, 38, right_w, val);
    }

    snprintf(val, sizeof(val), "%ld", (long)board_comm_encl_get_count());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(70, 54, left_w, val);
    snprintf(val, sizeof(val), "%ld", (long)rear_right_get_count());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(150, 54, right_w, val);

    snprintf(val, sizeof(val), "%ld mm", (long)board_comm_encl_get_dist_mm());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(70, 70, left_w, val);
    snprintf(val, sizeof(val), "%ld mm", (long)rear_right_get_dist_mm());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(150, 70, right_w, val);

    snprintf(val, sizeof(val), "%ld mm/s", (long)board_comm_encl_get_spd_mm_s());
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    menu_ui_show_pad(70, 86, left_w, val);
    snprintf(val, sizeof(val), "%ld mm/s", (long)rear_right_get_spd_mm_s());
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    menu_ui_show_pad(150, 86, right_w, val);

    snprintf(val, sizeof(val), "%.3f", (double)encoder_odom_get_px_m());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(70, 102, left_w, val);
    snprintf(val, sizeof(val), "%.3f", (double)encoder_odom_right_get_px_m());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(150, 102, right_w, val);

    snprintf(val, sizeof(val), "%.3f", (double)encoder_odom_get_py_m());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(70, 118, left_w, val);
    snprintf(val, sizeof(val), "%.3f", (double)encoder_odom_right_get_py_m());
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(150, 118, right_w, val);

    snprintf(val, sizeof(val), "%.3f", (double)encoder_odom_get_speed_ms());
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    menu_ui_show_pad(70, 134, left_w, val);
    snprintf(val, sizeof(val), "%.3f", (double)encoder_odom_right_get_speed_ms());
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    menu_ui_show_pad(150, 134, right_w, val);

    {
        uint8 active = ((0U != encoder_odom_is_enabled()) &&
                        (0U != encoder_odom_is_active())) ? 1U : 0U;
        snprintf(val, sizeof(val), "%s", (0U != active) ? "ON " : "OFF");
        ips200_set_color((0U != active) ? RGB565_GREEN : RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(70, 150, left_w, val);

        active = ((0U != encoder_odom_right_is_enabled()) &&
                  (0U != encoder_odom_right_is_active())) ? 1U : 0U;
        snprintf(val, sizeof(val), "%s", (0U != active) ? "ON " : "OFF");
        ips200_set_color((0U != active) ? RGB565_GREEN : RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(150, 150, right_w, val);
    }

    if (0U != left_last_ms)
        snprintf(val, sizeof(val), "%lu", (unsigned long)left_age_ms);
    else
        snprintf(val, sizeof(val), "--");
    ips200_set_color((left_age_ms > 500U || 0U == left_last_ms) ? RGB565_RED : RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(70, 166, left_w, val);

    if (0U != right_last_ms)
        snprintf(val, sizeof(val), "%lu", (unsigned long)right_age_ms);
    else
        snprintf(val, sizeof(val), "--");
    ips200_set_color((right_age_ms > 500U || 0U == right_last_ms) ? RGB565_RED : RGB565_WHITE, RGB565_BLACK);
    menu_ui_show_pad(150, 166, right_w, val);
}

static void icm_draw_ins_track_map_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char line0[48];
    char line1[48];
    char line2[48];
    char hint[48];
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    screen_point start_point = {0, 0};
    screen_point end_point = {0, 0};
    screen_point current_point = {0, 0};
    uint16 point_count = ins_record_get_point_count();
    uint16 map_x = 5U;
    uint16 map_y = 105U;
    uint16 map_w = (uint16)(ips200_width_max - 10U);
    uint16 map_h;
    uint16 i;
    uint16 text_width = (uint16)(ips200_width_max - 20U);
    uint16 sample_count = 0U;
    uint16 prev_rendered_point_count = icm_ins_map_rendered_point_count;
    uint32 now_ms = system_getval_ms();
    uint8 recording = ins_record_is_recording();
    uint8 current_valid = 0U;
    uint8 has_bounds = 0U;
    uint8 need_full_redraw = 0U;
    uint8 force_text_redraw = 0U;
    uint8 end_changed = 0U;
    uint8 current_changed = 0U;
    uint8 current_marker_valid = 0U;
    float current_px = 0.0f;
    float current_py = 0.0f;
    uint16 end_restore_color = ICM_TRACK_LINE_COLOR;
    uint16 current_restore_color = RGB565_BLACK;
    const ins_record_point_t *last_point = ins_record_get_last_point();

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        (now_ms - last_refresh_ms < ICM_INS_MAP_REFRESH_MS))
    {
        return;
    }
    last_refresh_ms = now_ms;

    if (recording)
    {
        icm_ins_get_position(&current_px, &current_py);
        current_valid = 1U;
    }

    map_h = (ips200_height_max > 190U) ? (uint16)(ips200_height_max - 190U) : (uint16)(ips200_height_max / 2U);
    if (map_h < 60U) map_h = 60U;
    if ((uint32)map_y + (uint32)map_h > (uint32)(ips200_height_max - 25U))
    {
        map_h = (uint16)(ips200_height_max - map_y - 25U);
    }
    if (map_h < 60U) map_h = 60U;
    icm_ins_map_set_area(map_x, map_y, map_w, map_h);

    has_bounds = icm_collect_ins_track_bounds(&min_x, &max_x, &min_y, &max_y,
                                              0U, 0.0f, 0.0f);
    need_full_redraw = ((NULL != menu_full_redraw) && *menu_full_redraw) ? 1U :
                       icm_ins_map_should_full_redraw(has_bounds, min_x, max_x, min_y, max_y);
    force_text_redraw = need_full_redraw;

    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);

        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 10, "INS Track Map");
        ips200_draw_line(0, 30, ips200_width_max - 1, 30, RGB565_GRAY);

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(5, (uint16)(ips200_height_max - 20U),
                     (uint16)(ips200_width_max - 10U),
                     "K1:Reset INS  LONG:Exit");
    }

    snprintf(line0, sizeof(line0), "PTS:%u REC:%s",
             (unsigned int)point_count,
             recording ? "ON" : "OFF");

    if (NULL != last_point)
    {
        snprintf(line1, sizeof(line1), "END:%+.2f,%+.2f m",
                 (double)last_point->px_m,
                 (double)last_point->py_m);
    }
    else
    {
        snprintf(line1, sizeof(line1), "END: --");
    }

    if (0U == point_count)
    {
        if (recording)
        {
            snprintf(line2, sizeof(line2), "Recording, waiting first point");
        }
        else
        {
            snprintf(line2, sizeof(line2), "No INS track recorded");
        }
    }
    else if (1U == point_count)
    {
        snprintf(line2, sizeof(line2), "Single point, S=E=Last");
    }
    else if (has_bounds)
    {
        snprintf(line2, sizeof(line2), "SPAN:%.2f x %.2f m",
                 (double)(max_x - min_x),
                 (double)(max_y - min_y));
    }
    else
    {
        snprintf(line2, sizeof(line2), "Track ready");
    }

    icm_ins_map_update_text_line(10U, 36U, text_width, line0,
                                 icm_ins_map_line0_cache, RGB565_WHITE, force_text_redraw);
    icm_ins_map_update_text_line(10U, 56U, text_width, line1,
                                 icm_ins_map_line1_cache, RGB565_WHITE, force_text_redraw);
    icm_ins_map_update_text_line(10U, 76U, text_width, line2,
                                 icm_ins_map_line2_cache, RGB565_WHITE, force_text_redraw);

    if (need_full_redraw)
    {
        icm_ins_map_clear_inner_area(map_x, map_y, map_w, map_h);
        icm_draw_box_border(map_x, map_y, map_w, map_h, RGB565_WHITE);
        icm_ins_map_last_current_valid = 0U;
        icm_ins_map_last_end_valid = 0U;
        icm_ins_map_border_dirty = 0U;
    }

    if (0U == point_count)
    {
        snprintf(hint, sizeof(hint), "%s",
                 recording ? "Waiting for INS track points" : "INS track is empty");
        icm_ins_map_update_text_line((uint16)(map_x + 12U),
                                     (uint16)(map_y + (map_h / 2U) - 8U),
                                     (uint16)(map_w - 24U),
                                     hint,
                                     icm_ins_map_hint_cache,
                                     RGB565_GRAY,
                                     force_text_redraw);
        icm_ins_map_bounds_valid = 0U;
        icm_ins_map_rendered_point_count = 0U;
        if (NULL != menu_full_redraw) *menu_full_redraw = 0U;
        return;
    }

    if ('\0' != icm_ins_map_hint_cache[0] && need_full_redraw)
    {
        icm_ins_map_hint_cache[0] = '\0';
    }

    sample_count = icm_collect_ins_track_samples(icm_track_x_points, icm_track_y_points, DISPLAY_POINT_MAX);
    if (0U == sample_count)
    {
        if (NULL != menu_full_redraw) *menu_full_redraw = 0U;
        return;
    }

    gps_set_display_area(0, 0, (int16)map_w, (int16)map_h);
    gps_set_xy_bounds(min_x, max_x, min_y, max_y);
    xy_to_screen(icm_track_x_points, icm_track_y_points, (int)sample_count);
    gps_clear_xy_bounds();

    start_point = screen_point_data[0];
    end_point = screen_point_data[sample_count - 1U];
    end_restore_color = (sample_count <= 1U) ? ICM_TRACK_START_COLOR : ICM_TRACK_LINE_COLOR;

    if (need_full_redraw)
    {
        for (i = 1U; i < sample_count; i++)
        {
            if (icm_map_point_in_area(&screen_point_data[i - 1U], map_w, map_h) &&
                icm_map_point_in_area(&screen_point_data[i], map_w, map_h))
            {
                ips200_draw_line((uint16)(map_x + screen_point_data[i - 1U].x),
                                 (uint16)(map_y + screen_point_data[i - 1U].y),
                                 (uint16)(map_x + screen_point_data[i].x),
                                 (uint16)(map_y + screen_point_data[i].y),
                                 ICM_TRACK_LINE_COLOR);
            }
        }
        icm_draw_map_square_marker(map_x, map_y, map_w, map_h, &start_point, ICM_TRACK_START_COLOR);
        icm_ins_map_update_bounds_cache(min_x, max_x, min_y, max_y);
        icm_ins_map_rendered_point_count = point_count;
    }
    else if (point_count > prev_rendered_point_count)
    {
        uint16 start_index = (prev_rendered_point_count > 0U) ? prev_rendered_point_count : 1U;

        for (i = start_index; i < sample_count; i++)
        {
            if (icm_map_point_in_area(&screen_point_data[i - 1U], map_w, map_h) &&
                icm_map_point_in_area(&screen_point_data[i], map_w, map_h))
            {
                ips200_draw_line((uint16)(map_x + screen_point_data[i - 1U].x),
                                 (uint16)(map_y + screen_point_data[i - 1U].y),
                                 (uint16)(map_x + screen_point_data[i].x),
                                 (uint16)(map_y + screen_point_data[i].y),
                                 ICM_TRACK_LINE_COLOR);
            }
        }
        icm_ins_map_rendered_point_count = point_count;
    }

    if (current_valid &&
        icm_ins_map_xy_to_screen_point(map_w, map_h,
                                       min_x, max_x, min_y, max_y,
                                       (double)current_px, (double)current_py,
                                       &current_point) &&
        icm_map_point_in_area(&current_point, map_w, map_h))
    {
        current_marker_valid = 1U;
        if (icm_screen_point_equal(&current_point, &end_point))
        {
            current_restore_color = ICM_TRACK_END_COLOR;
        }
        else if (icm_screen_point_equal(&current_point, &start_point))
        {
            current_restore_color = ICM_TRACK_START_COLOR;
        }
        else
        {
            current_restore_color = (sample_count > 1U) ? ICM_TRACK_LINE_COLOR : RGB565_BLACK;
        }
    }
    else
    {
        current_point = end_point;
        current_marker_valid = 1U;
        current_restore_color = (sample_count <= 1U) ? ICM_TRACK_START_COLOR : ICM_TRACK_END_COLOR;
    }

    end_changed = need_full_redraw ||
                  (point_count != prev_rendered_point_count) ||
                  !icm_ins_map_last_end_valid ||
                  !icm_screen_point_equal(&icm_ins_map_last_end_point, &end_point);
    current_changed = need_full_redraw ||
                      (icm_ins_map_last_current_valid != current_marker_valid) ||
                      (current_marker_valid &&
                       (!icm_screen_point_equal(&icm_ins_map_last_current_point, &current_point) ||
                        (icm_ins_map_last_current_restore_color != current_restore_color)));

    if (!need_full_redraw && current_changed)
    {
        icm_ins_map_clear_last_current_marker(map_x, map_y, map_w, map_h);
    }
    if (!need_full_redraw && end_changed)
    {
        icm_ins_map_clear_last_end_marker(map_x, map_y, map_w, map_h);
    }

    if (end_changed || need_full_redraw)
    {
        icm_draw_map_square_marker(map_x, map_y, map_w, map_h, &end_point, ICM_TRACK_END_COLOR);
        icm_ins_map_last_end_point = end_point;
        icm_ins_map_last_end_valid = 1U;
        icm_ins_map_last_end_restore_color = end_restore_color;
    }

    if (current_changed)
    {
        icm_draw_map_marker(map_x, map_y, map_w, map_h, &current_point, ICM_TRACK_CURRENT_COLOR);
        icm_ins_map_last_current_point = current_point;
        icm_ins_map_last_current_valid = current_marker_valid;
        icm_ins_map_last_current_restore_color = current_restore_color;
    }
    else if (!current_marker_valid)
    {
        icm_ins_map_last_current_valid = 0U;
    }

    if (NULL != menu_full_redraw) *menu_full_redraw = 0U;
}

/* ---- 原始数据页绘制 ---------------------------------------------------- */

static void icm_draw_raw_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char line[48];
    uint32 now_ms = system_getval_ms();

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

        /* 静态标签 (8px/char) */
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 56, "Ax:");        /* 3ch => val x=34 */
        ips200_show_string(114, 56, "Ay:");       /* 3ch => val x=138 */
        ips200_show_string(10, 78, "Az:");
        ips200_show_string(10, 126, "Gx:");
        ips200_show_string(114, 126, "Gy:");
        ips200_show_string(10, 148, "Gz:");

        /* 页脚 */
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(5, (uint16)(ips200_height_max - 20U), (uint16)(ips200_width_max - 10U), "LONG: Exit");

        *menu_full_redraw = 0U;
    }

    /* 动态数据区域 — 仅刷新数值，不重绘标签 */
    {
        uint16 vw = (uint16)(ips200_width_max - 10U - 34U);   /* 全宽减标签 */
        uint16 hw = (uint16)(ips200_width_max - 10U - 138U);  /* 半宽减标签 */

        ips200_set_color(RGB565_WHITE, RGB565_BLACK);

        /* 加速度 X */
        snprintf(line, sizeof(line), "%+.4f", (double)icm42688_acc_x);
        menu_ui_show_pad(34, 56, (uint16)(138U - 34U - 10U), line);

        /* 加速度 Y */
        snprintf(line, sizeof(line), "%+.4f", (double)icm42688_acc_y);
        menu_ui_show_pad(138, 56, hw, line);

        /* 加速度 Z */
        snprintf(line, sizeof(line), "%+.4f", (double)icm42688_acc_z);
        menu_ui_show_pad(34, 78, vw, line);

        /* 陀螺仪 X */
        snprintf(line, sizeof(line), "%+.2f", (double)icm42688_gyro_x);
        menu_ui_show_pad(34, 126, (uint16)(138U - 34U - 10U), line);

        /* 陀螺仪 Y */
        snprintf(line, sizeof(line), "%+.2f", (double)icm42688_gyro_y);
        menu_ui_show_pad(138, 126, hw, line);

        /* 陀螺仪 Z */
        snprintf(line, sizeof(line), "%+.2f", (double)icm42688_gyro_z);
        menu_ui_show_pad(34, 148, vw, line);
    }
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

        /* 静态标签 (8px/char): 6ch label => val x=58 */
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 60, "Roll :");
        ips200_show_string(10, 82, "Pitch:");
        ips200_show_string(10, 104, "Yaw  :");
        ips200_show_string(10, 154, "AccNorm:");  /* 8ch => val x=74 */
        ips200_show_string(122, 154, "g");         /* unit after 6-char value */
        ips200_show_string(10, 176, "Upd:");       /* 4ch => val x=42 */
        ips200_show_string(90, 176, "Hz");         /* unit after 6-char value */
        ips200_show_string(114, 176, "Cnt:");      /* 4ch => val x=146 */

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(5, (uint16)(ips200_height_max - 20U), (uint16)(ips200_width_max - 10U), "Yaw drifts, LONG: Exit");

        *menu_full_redraw = 0U;
    }

    icm_attitude_get_euler(&roll_deg, &pitch_deg, &yaw_deg);
    acc_norm_g = icm_attitude_get_acc_norm_g();

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    /* Roll value (6ch label, val x=58, 8ch value = 64px) */
    snprintf(line, sizeof(line), "%+8.2f", (double)roll_deg);
    menu_ui_show_pad(58, 60, 64, line);

    /* Pitch value */
    snprintf(line, sizeof(line), "%+8.2f", (double)pitch_deg);
    menu_ui_show_pad(58, 82, 64, line);

    /* Yaw value */
    snprintf(line, sizeof(line), "%+8.2f", (double)yaw_deg);
    menu_ui_show_pad(58, 104, 64, line);

    /* AccNorm value (8ch label, val x=74, 6ch value = 48px) */
    snprintf(line, sizeof(line), "%6.3f", (double)acc_norm_g);
    menu_ui_show_pad(74, 154, 48, line);

    /* Upd value (4ch label, val x=42, 6ch value = 48px) */
    snprintf(line, sizeof(line), "%6.1f", (double)update_hz);
    menu_ui_show_pad(42, 176, 48, line);

    /* Cnt value (4ch label at x=114, val x=146) */
    snprintf(line, sizeof(line), "%lu", (unsigned long)update_count);
    menu_ui_show_pad(146, 176, (uint16)(ips200_width_max - 156U), line);
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
        menu_ui_show_pad(10, 40, (uint16)(ips200_width_max - 20U), "Keep the car still");
        menu_ui_show_pad(10, 56, (uint16)(ips200_width_max - 20U), "K1:Restart  LONG:Back");
        menu_ui_show_pad(10, 72, (uint16)(ips200_width_max - 20U), "Auto-save to flash on done");

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 94, "Calibration Status");
        ips200_draw_line(10, 108, (uint16)(ips200_width_max - 10U), 108, RGB565_GRAY);
        ips200_show_string(10, 158, "Gyro Bias (dps)");
        ips200_draw_line(10, 172, (uint16)(ips200_width_max - 10U), 172, RGB565_GRAY);

        /* 静态标签 (8px/char) */
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 116, "Status:");   /* 7ch => val x=66 */
        ips200_show_string(10, 136, "Samples:");  /* 8ch => val x=74 */
        ips200_show_string(10, 180, "Bx:");       /* 3ch => val x=34 */
        ips200_show_string(10, 200, "By:");       /* 3ch => val x=34 */
        ips200_show_string(106, 200, "Bz:");      /* 3ch => val x=130 */

        *menu_full_redraw = 0U;
    }

    icm_attitude_get_gyro_bias(&bias_x, &bias_y, &bias_z);
    icm_attitude_get_gyro_bias_calibration_progress(&sample_count, &target_count);

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    if (icm_attitude_is_gyro_bias_calibrating())
    {
        snprintf(line, sizeof(line), " CALIBRATING");
    }
    else if (icm_attitude_is_gyro_bias_valid())
    {
        if (icm_attitude_is_gyro_bias_from_flash())
        {
            snprintf(line, sizeof(line), " DONE [FLASH]");
        }
        else
        {
            snprintf(line, sizeof(line), " DONE [SAVED]");
        }
    }
    else
    {
        snprintf(line, sizeof(line), " NOT READY");
    }
    /* Status value (7ch label, val x=66) */
    menu_ui_show_pad(66, 116, (uint16)(ips200_width_max - 76U), line);

    /* Samples value (8ch label, val x=74) */
    snprintf(line, sizeof(line), "%4lu/%4lu", (unsigned long)sample_count, (unsigned long)target_count);
    menu_ui_show_pad(74, 136, 72, line);

    /* Bx value (3ch label, val x=34) */
    snprintf(line, sizeof(line), "%+8.3f", (double)bias_x);
    menu_ui_show_pad(34, 180, 64, line);

    /* By value (3ch label, val x=34) */
    snprintf(line, sizeof(line), "%+8.3f", (double)bias_y);
    menu_ui_show_pad(34, 200, 64, line);

    /* Bz value (3ch label at x=106, val x=130) */
    snprintf(line, sizeof(line), "%+8.3f", (double)bias_z);
    menu_ui_show_pad(130, 200, 64, line);
}

/* ---- INS Dead Reckoning: ZUPT + velocity + position ------------------- */

static void icm_draw_ins_debug_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char line[48];
    float ax = 0.0f;
    float ay = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float px = 0.0f;
    float py = 0.0f;
    float yaw_deg = 0.0f;
    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    uint32 now_ms = system_getval_ms();

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
        ips200_show_string(10, 10, "INS Dead Reckoning");
        ips200_draw_line(0, 30, ips200_width_max - 1, 30, RGB565_GRAY);

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 38, "Linear Acc (m/s^2)");
        ips200_draw_line(10, 52, (uint16)(ips200_width_max - 10U), 52, RGB565_GRAY);

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 96, "Velocity (m/s)");
        ips200_draw_line(10, 110, (uint16)(ips200_width_max - 10U), 110, RGB565_GRAY);

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 152, "Position (m)");
        ips200_draw_line(10, 166, (uint16)(ips200_width_max - 10U), 166, RGB565_GRAY);

        /* 静态标签 (8px/char) */
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 56, "Ax:");        /* 3ch => val x=34 */
        ips200_show_string(98, 56, "Ay:");        /* 3ch => val x=122 */
        ips200_show_string(10, 74, "ZUPT:[");     /* 6ch => val x=58 */
        ips200_show_string(74, 74, "]");
        ips200_show_string(90, 74, "Bias:[");     /* 6ch => val x=138 */
        ips200_show_string(154, 74, "]");
        ips200_show_string(10, 114, "Vx:");       /* 3ch => val x=34 */
        ips200_show_string(98, 114, "Vy:");       /* 3ch => val x=122 */
        ips200_show_string(10, 132, "Spd:");      /* 4ch => val x=42 */
        ips200_show_string(106, 132, "m/s");      /* unit suffix (1ch gap) */
        ips200_show_string(10, 170, "Px:");       /* 3ch => val x=34 */
        ips200_show_string(106, 170, "Py:");      /* 3ch => val x=130 */
        ips200_show_string(10, 192, "Yaw:");      /* 4ch => val x=42 */
        ips200_show_string(10, 210, "R:");        /* 2ch => val x=26 */
        ips200_show_string(82, 210, "P:");        /* 2ch => val x=98 */

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(5, (uint16)(ips200_height_max - 20U),
                     (uint16)(ips200_width_max - 10U), "K1:Reset Pos  LONG:Exit");

        *menu_full_redraw = 0U;
    }

    /* K1 short press: reset position and velocity */
    if (my_key_get_state(MY_KEY_1) == MY_KEY_SHORT_PRESS)
    {
        icm_ins_reset_velocity();
        icm_ins_reset_position();
        menu_ui_consume_key1();
    }

    icm_ins_get_linear_acc(&ax, &ay, NULL);
    icm_ins_get_velocity(&vx, &vy, NULL);
    icm_ins_get_position(&px, &py);
    icm_attitude_get_euler(&roll_deg, &pitch_deg, &yaw_deg);

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    /* Lin Acc X (3ch label, val x=34, 7ch value = 56px) */
    snprintf(line, sizeof(line), "%+7.3f", (double)ax);
    menu_ui_show_pad(34, 56, 56, line);

    /* Lin Acc Y (3ch label at x=98, val x=122, 7ch = 56px) */
    snprintf(line, sizeof(line), "%+7.3f", (double)ay);
    menu_ui_show_pad(122, 56, 56, line);

    /* ZUPT value (val x=58, 2ch = 16px) */
    snprintf(line, sizeof(line), "%s", icm_ins_is_stationary() ? "ON" : "--");
    if (icm_ins_is_stationary())
    {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    }
    menu_ui_show_pad(58, 74, 16, line);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    /* Bias value (val x=138, 2ch = 16px) */
    snprintf(line, sizeof(line), "%s", icm_attitude_is_gyro_bias_valid() ? "ON" : "--");
    menu_ui_show_pad(138, 74, 16, line);

    /* Vel X (3ch label, val x=34, 7ch = 56px) */
    snprintf(line, sizeof(line), "%+7.3f", (double)vx);
    menu_ui_show_pad(34, 114, 56, line);

    /* Vel Y (3ch label at x=98, val x=122, 7ch = 56px) */
    snprintf(line, sizeof(line), "%+7.3f", (double)vy);
    menu_ui_show_pad(122, 114, 56, line);

    /* Speed value (4ch label, val x=42, 7ch = 56px) */
    snprintf(line, sizeof(line), "%7.3f", (double)icm_ins_get_speed_ms());
    menu_ui_show_pad(42, 132, 56, line);

    /* Position X (3ch label, val x=34, 8ch = 64px) */
    snprintf(line, sizeof(line), "%+8.2f", (double)px);
    menu_ui_show_pad(34, 170, 64, line);

    /* Position Y (3ch label at x=106, val x=130, 8ch = 64px) */
    snprintf(line, sizeof(line), "%+8.2f", (double)py);
    menu_ui_show_pad(130, 170, 64, line);

    /* Yaw value (4ch label, val x=42, 7ch = 56px) */
    snprintf(line, sizeof(line), "%+7.1f", (double)yaw_deg);
    menu_ui_show_pad(42, 192, 56, line);

    /* Roll value (2ch label, val x=26, 6ch = 48px) */
    snprintf(line, sizeof(line), "%+6.1f", (double)roll_deg);
    menu_ui_show_pad(26, 210, 48, line);

    /* Pitch value (2ch label at x=82, val x=98, 6ch = 48px) */
    snprintf(line, sizeof(line), "%+6.1f", (double)pitch_deg);
    menu_ui_show_pad(98, 210, 48, line);
}

/* ---- 公共接口 ---------------------------------------------------------- */

void menu_icm_action_enter(menu_view_ctx_t *ctx, uint8 target_mode)
{
    /* 特殊进入逻辑：部分视图需要额外初始化 */
    if (ICM_VIEW_INS_MAP == target_mode)
    {
        icm_ins_map_reset_cache();
    }
    else if (ICM_VIEW_GYRO_BIAS_CALIB == target_mode)
    {
        icm_attitude_start_gyro_bias_calibration(ICM_GYRO_BIAS_CALIB_SAMPLES);
    }

    menu_view_enter(ctx, target_mode);
}

uint8 menu_icm_handle_view(menu_view_ctx_t *ctx)
{
    if ((NULL == ctx) || (NULL == ctx->mode) || (ICM_VIEW_NONE == *(ctx->mode))) return 0U;

    /* 长按退出 */
    if ((MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1)) || menu_ui_check_exit_hold(&icm_exit_hold_timer, 250U))
    {
        menu_ui_consume_key1();
        if (ICM_VIEW_GYRO_BIAS_CALIB == *(ctx->mode))
        {
            icm_attitude_cancel_gyro_bias_calibration();
        }
        *(ctx->mode) = ICM_VIEW_NONE;
        if (NULL != ctx->drain_encoder_events) ctx->drain_encoder_events();
        if (NULL != ctx->reset_dynamic_region) ctx->reset_dynamic_region();
        if (NULL != ctx->request_redraw) ctx->request_redraw(1U);
        return 1U;
    }

    if ((ICM_VIEW_GYRO_BIAS_CALIB == *(ctx->mode)) &&
        (MY_KEY_SHORT_PRESS == my_key_get_state(MY_KEY_1)))
    {
        menu_ui_consume_key1();
        icm_attitude_start_gyro_bias_calibration(ICM_GYRO_BIAS_CALIB_SAMPLES);
        if (NULL != ctx->menu_full_redraw) *(ctx->menu_full_redraw) = 1U;
        return 1U;
    }

    /* Encoder 页: K1 短按 → 切换编码器融合开/关 */
    if ((ICM_VIEW_ENCODER == *(ctx->mode)) &&
        (MY_KEY_SHORT_PRESS == my_key_get_state(MY_KEY_1)))
    {
        uint8 next_enable;
        menu_ui_consume_key1();
        next_enable = (encoder_odom_is_enabled() || encoder_odom_right_is_enabled()) ? 0U : 1U;
        encoder_odom_set_enable(next_enable);
        encoder_odom_right_set_enable(next_enable);
        if (NULL != ctx->menu_full_redraw) *(ctx->menu_full_redraw) = 1U;
        return 1U;
    }

    /* Track Map 页: K1 短按 → 重置 INS 位置/速度 + 编码器里程计 */
    if ((ICM_VIEW_INS_MAP == *(ctx->mode)) &&
        (MY_KEY_SHORT_PRESS == my_key_get_state(MY_KEY_1)))
    {
        menu_ui_consume_key1();
        icm_ins_reset_position();
        icm_ins_reset_velocity();
        encoder_odom_reset();
        if (NULL != ctx->menu_full_redraw) *(ctx->menu_full_redraw) = 1U;
        return 1U;
    }

    if (ICM_VIEW_RAW == *(ctx->mode))
    {
        icm_draw_raw_page(ctx->menu_full_redraw);
    }
    else if (ICM_VIEW_ATTITUDE == *(ctx->mode))
    {
        icm_draw_attitude_page(ctx->menu_full_redraw);
    }
    else if (ICM_VIEW_GYRO_BIAS_CALIB == *(ctx->mode))
    {
        icm_draw_gyro_bias_calib_page(ctx->menu_full_redraw);
    }
    else if (ICM_VIEW_INS_DEBUG == *(ctx->mode))
    {
        icm_draw_ins_debug_page(ctx->menu_full_redraw);
    }
    else if (ICM_VIEW_INS_MAP == *(ctx->mode))
    {
        icm_draw_ins_track_map_page(ctx->menu_full_redraw);
    }
    else if (ICM_VIEW_ENCODER == *(ctx->mode))
    {
        icm_draw_encoder_page(ctx->menu_full_redraw);
    }

    return 1U;
}
