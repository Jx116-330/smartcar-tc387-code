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

#define ICM_GYRO_BIAS_CALIB_SAMPLES 20000U
#define ICM_VIEW_EXIT_HOLD_MS       250U
#define ICM_VIEW_EXIT_KEY_PIN       P20_2
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

    icm_fill_rect((uint16)(map_x + 1U),
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
    icm_show_pad(x, y, max_width, text);
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
        icm_show_pad(5, (uint16)(ips200_height_max - 20U),
                     (uint16)(ips200_width_max - 10U),
                     "S:GRN E:CYN C/L:RED LONG:Exit");
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
        icm_show_pad(10, 56, (uint16)(ips200_width_max - 20U), "K1:Restart  LONG:Back");
        icm_show_pad(10, 72, (uint16)(ips200_width_max - 20U), "Auto-save to flash on done");

        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 94, "Calibration Status");
        ips200_draw_line(10, 108, (uint16)(ips200_width_max - 10U), 108, RGB565_GRAY);
        ips200_show_string(10, 158, "Gyro Bias (dps)");
        ips200_draw_line(10, 172, (uint16)(ips200_width_max - 10U), 172, RGB565_GRAY);

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
        if (icm_attitude_is_gyro_bias_from_flash())
        {
            snprintf(line, sizeof(line), "Status: DONE [FLASH]");
        }
        else
        {
            snprintf(line, sizeof(line), "Status: DONE [SAVED]");
        }
    }
    else
    {
        snprintf(line, sizeof(line), "Status: NOT READY");
    }
    icm_fill_rect(10U, 116U, (uint16)(ips200_width_max - 10U), 130U, RGB565_BLACK);
    icm_show_pad(10, 116, col_w, line);

    snprintf(line, sizeof(line), "Samples:%4lu/%4lu", (unsigned long)sample_count, (unsigned long)target_count);
    icm_fill_rect(10U, 136U, (uint16)(ips200_width_max - 10U), 150U, RGB565_BLACK);
    icm_show_pad(10, 136, col_w, line);

    snprintf(line, sizeof(line), "Bx:%+8.3f", (double)bias_x);
    icm_fill_rect(10U, 180U, (uint16)(ips200_width_max - 10U), 194U, RGB565_BLACK);
    icm_show_pad(10, 180, col_w, line);

    snprintf(line, sizeof(line), "By:%+8.3f Bz:%+8.3f", (double)bias_y, (double)bias_z);
    icm_fill_rect(10U, 200U, (uint16)(ips200_width_max - 10U), 214U, RGB565_BLACK);
    icm_show_pad(10, 200, col_w, line);
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
    uint16 col_w  = (uint16)(ips200_width_max - 20U);

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

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        icm_show_pad(5, (uint16)(ips200_height_max - 20U),
                     (uint16)(ips200_width_max - 10U), "K1:Reset Pos  LONG:Exit");

        *menu_full_redraw = 0U;
    }

    /* K1 short press: reset position and velocity */
    if (my_key_get_state(MY_KEY_1) == MY_KEY_SHORT_PRESS)
    {
        icm_ins_reset_velocity();
        icm_ins_reset_position();
        icm_consume_key1_press();
    }

    icm_ins_get_linear_acc(&ax, &ay, NULL);
    icm_ins_get_velocity(&vx, &vy, NULL);
    icm_ins_get_position(&px, &py);
    icm_attitude_get_euler(&roll_deg, &pitch_deg, &yaw_deg);

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    /* Lin Acc XY */
    snprintf(line, sizeof(line), "Ax:%+7.3f Ay:%+7.3f", (double)ax, (double)ay);
    icm_fill_rect(10U, 56U, (uint16)(ips200_width_max - 10U), 70U, RGB565_BLACK);
    icm_show_pad(10, 56, col_w, line);

    /* ZUPT + Bias status */
    snprintf(line, sizeof(line), "ZUPT:[%s] Bias:[%s]",
             icm_ins_is_stationary() ? "ON" : "--",
             icm_attitude_is_gyro_bias_valid() ? "ON" : "--");
    icm_fill_rect(10U, 74U, (uint16)(ips200_width_max - 10U), 88U, RGB565_BLACK);
    if (icm_ins_is_stationary())
    {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    }
    icm_show_pad(10, 74, col_w, line);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    /* Vel XY + speed */
    snprintf(line, sizeof(line), "Vx:%+7.3f Vy:%+7.3f", (double)vx, (double)vy);
    icm_fill_rect(10U, 114U, (uint16)(ips200_width_max - 10U), 128U, RGB565_BLACK);
    icm_show_pad(10, 114, col_w, line);

    snprintf(line, sizeof(line), "Spd:%7.3f m/s", (double)icm_ins_get_speed_ms());
    icm_fill_rect(10U, 132U, (uint16)(ips200_width_max - 10U), 146U, RGB565_BLACK);
    icm_show_pad(10, 132, col_w, line);

    /* Position XY */
    snprintf(line, sizeof(line), "Px:%+8.2f Py:%+8.2f", (double)px, (double)py);
    icm_fill_rect(10U, 170U, (uint16)(ips200_width_max - 10U), 184U, RGB565_BLACK);
    icm_show_pad(10, 170, col_w, line);

    /* Yaw + Roll/Pitch */
    snprintf(line, sizeof(line), "Yaw:%+7.1f", (double)yaw_deg);
    icm_fill_rect(10U, 192U, (uint16)(ips200_width_max - 10U), 206U, RGB565_BLACK);
    icm_show_pad(10, 192, col_w, line);

    snprintf(line, sizeof(line), "R:%+6.1f P:%+6.1f", (double)roll_deg, (double)pitch_deg);
    icm_fill_rect(10U, 210U, (uint16)(ips200_width_max - 10U), 224U, RGB565_BLACK);
    icm_show_pad(10, 210, col_w, line);
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

void menu_icm_action_ins_debug(icm_view_mode_t *icm_mode,
                              uint8 *menu_full_redraw,
                              void (*drain_encoder_events)(void),
                              void (*request_redraw)(uint8 full_redraw),
                              void (*reset_dynamic_region)(void))
{
    icm_enter_view(icm_mode, ICM_VIEW_INS_DEBUG, menu_full_redraw,
                   drain_encoder_events, request_redraw, reset_dynamic_region);
}

void menu_icm_action_ins_track_map(icm_view_mode_t *icm_mode,
                                   uint8 *menu_full_redraw,
                                   void (*drain_encoder_events)(void),
                                   void (*request_redraw)(uint8 full_redraw),
                                   void (*reset_dynamic_region)(void))
{
    icm_ins_map_reset_cache();
    icm_enter_view(icm_mode, ICM_VIEW_INS_MAP, menu_full_redraw,
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
    else if (ICM_VIEW_INS_DEBUG == *icm_mode)
    {
        icm_draw_ins_debug_page(menu_full_redraw);
    }
    else if (ICM_VIEW_INS_MAP == *icm_mode)
    {
        icm_draw_ins_track_map_page(menu_full_redraw);
    }

    return 1U;
}
