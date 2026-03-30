/**
 * @file path_display.c
 * @brief GPS 轨迹地图显示实现
 * @author JX116
 */

#include "path_recorder.h"
#include "display_gps.h"
#include "zf_device_ips200.h"

#define DISPLAY_AREA_X      10U
#define DISPLAY_AREA_Y      10U
#define DISPLAY_AREA_WIDTH  300U
#define DISPLAY_AREA_HEIGHT 220U

#define CURRENT_POS_COLOR   RGB565_RED
#define MAP_BORDER_COLOR    RGB565_WHITE

static uint16 display_x = DISPLAY_AREA_X;
static uint16 display_y = DISPLAY_AREA_Y;
static uint16 display_width = DISPLAY_AREA_WIDTH;
static uint16 display_height = DISPLAY_AREA_HEIGHT;
static uint8 display_initialized = 0U;
static uint8 display_border_dirty = 1U;

static void path_display_clamp_area(void);
static void path_display_fill_rect(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, uint16 color);
static void path_display_draw_border(void);
static uint16 path_display_collect_points(gps_point *gps_points);
static uint8 path_display_collect_bounds(double *min_x, double *max_x, double *min_y, double *max_y);
static uint8 path_display_point_in_area(const screen_point *point);
static void path_display_draw_marker(const screen_point *point, uint16 color);
static void path_display_draw_trajectory(uint16 color);
static void path_display_draw_current_position(void);

static void path_display_clamp_area(void)
{
    if (0U == ips200_width_max || 0U == ips200_height_max)
    {
        return;
    }

    if (display_x >= ips200_width_max)
    {
        display_x = ips200_width_max - 1U;
    }
    if (display_y >= ips200_height_max)
    {
        display_y = ips200_height_max - 1U;
    }

    if (0U == display_width)
    {
        display_width = 1U;
    }
    if (0U == display_height)
    {
        display_height = 1U;
    }

    if ((uint32)display_x + (uint32)display_width > (uint32)ips200_width_max)
    {
        display_width = ips200_width_max - display_x;
    }
    if ((uint32)display_y + (uint32)display_height > (uint32)ips200_height_max)
    {
        display_height = ips200_height_max - display_y;
    }

    if (0U == display_width)
    {
        display_width = 1U;
    }
    if (0U == display_height)
    {
        display_height = 1U;
    }
}

static void path_display_fill_rect(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, uint16 color)
{
    uint16 y;

    if (0U == ips200_width_max || 0U == ips200_height_max)
    {
        return;
    }

    if (x_start > x_end)
    {
        uint16 temp = x_start;
        x_start = x_end;
        x_end = temp;
    }
    if (y_start > y_end)
    {
        uint16 temp = y_start;
        y_start = y_end;
        y_end = temp;
    }

    if (x_start >= ips200_width_max) x_start = ips200_width_max - 1U;
    if (x_end >= ips200_width_max) x_end = ips200_width_max - 1U;
    if (y_start >= ips200_height_max) y_start = ips200_height_max - 1U;
    if (y_end >= ips200_height_max) y_end = ips200_height_max - 1U;

    for (y = y_start; y <= y_end; y++)
    {
        ips200_draw_line(x_start, y, x_end, y, color);
    }
}

static void path_display_draw_border(void)
{
    uint16 x_end;
    uint16 y_end;

    if (!display_initialized || display_width < 2U || display_height < 2U)
    {
        return;
    }

    x_end = display_x + display_width - 1U;
    y_end = display_y + display_height - 1U;

    ips200_draw_line(display_x, display_y, x_end, display_y, MAP_BORDER_COLOR);
    ips200_draw_line(display_x, y_end, x_end, y_end, MAP_BORDER_COLOR);
    ips200_draw_line(display_x, display_y, display_x, y_end, MAP_BORDER_COLOR);
    ips200_draw_line(x_end, display_y, x_end, y_end, MAP_BORDER_COLOR);
}

static uint16 path_display_collect_points(gps_point *gps_points)
{
    uint16 total_count;
    uint16 sample_count;
    uint16 i;

    total_count = path_data.point_count;
    if (0U == total_count)
    {
        return 0U;
    }

    sample_count = total_count;
    if (sample_count > DISPLAY_POINT_MAX)
    {
        sample_count = DISPLAY_POINT_MAX;
    }

    if (sample_count == total_count)
    {
        for (i = 0U; i < sample_count; i++)
        {
            gps_points[i].lat = (double)path_data.points[i].latitude;
            gps_points[i].lon = (double)path_data.points[i].longitude;
        }
    }
    else if (sample_count <= 1U)
    {
        gps_points[0].lat = (double)path_data.points[0].latitude;
        gps_points[0].lon = (double)path_data.points[0].longitude;
        sample_count = 1U;
    }
    else
    {
        uint32 total_span = (uint32)(total_count - 1U);
        uint32 sample_span = (uint32)(sample_count - 1U);

        for (i = 0U; i < sample_count; i++)
        {
            uint32 index = (i * total_span + (sample_span / 2U)) / sample_span;
            gps_points[i].lat = (double)path_data.points[index].latitude;
            gps_points[i].lon = (double)path_data.points[index].longitude;
        }
    }

    return sample_count;
}

static uint8 path_display_collect_bounds(double *min_x, double *max_x, double *min_y, double *max_y)
{
    double ref_lat;
    double ref_lon;
    uint16 i;

    if (NULL == min_x || NULL == max_x || NULL == min_y || NULL == max_y || path_data.point_count == 0U)
    {
        return 0U;
    }

    ref_lat = (double)path_data.points[0].latitude * USER_PI / 180.0;
    ref_lon = (double)path_data.points[0].longitude * USER_PI / 180.0;

    for (i = 0U; i < path_data.point_count; i++)
    {
        double lat = (double)path_data.points[i].latitude * USER_PI / 180.0;
        double lon = (double)path_data.points[i].longitude * USER_PI / 180.0;
        double x = EARTH_RADIUS * (lon - ref_lon) * cos(ref_lat);
        double y = EARTH_RADIUS * (lat - ref_lat);

        if (0U == i)
        {
            *min_x = x;
            *max_x = x;
            *min_y = y;
            *max_y = y;
        }
        else
        {
            if (x < *min_x) *min_x = x;
            if (x > *max_x) *max_x = x;
            if (y < *min_y) *min_y = y;
            if (y > *max_y) *max_y = y;
        }
    }

    return 1U;
}

static uint8 path_display_point_in_area(const screen_point *point)
{
    if (NULL == point)
    {
        return 0U;
    }

    return (point->x >= 0 &&
            point->y >= 0 &&
            point->x < (int16)display_width &&
            point->y < (int16)display_height);
}

static void path_display_draw_marker(const screen_point *point, uint16 color)
{
    static const int8 x_offsets[5] = {0, -1, 1, 0, 0};
    static const int8 y_offsets[5] = {0, 0, 0, -1, 1};
    uint16 i;

    for (i = 0U; i < 5U; i++)
    {
        screen_point draw_point = {
            (int16)(point->x + x_offsets[i]),
            (int16)(point->y + y_offsets[i])
        };

        if (path_display_point_in_area(&draw_point))
        {
            ips200_draw_point((uint16)(display_x + draw_point.x),
                              (uint16)(display_y + draw_point.y),
                              color);
        }
    }
}

static void path_display_draw_trajectory(uint16 color)
{
    gps_point gps_points[DISPLAY_POINT_MAX];
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    uint16 count;
    uint16 i;

    if (!display_initialized || path_data.point_count == 0U)
    {
        return;
    }

    count = path_display_collect_points(gps_points);
    if (0U == count)
    {
        return;
    }

    gps_set_display_area(0, 0, (int16)display_width, (int16)display_height);
    if (path_display_collect_bounds(&min_x, &max_x, &min_y, &max_y))
    {
        gps_set_xy_bounds(min_x, max_x, min_y, max_y);
    }
    user_gps_transition(gps_points, (int)count);
    gps_clear_xy_bounds();

    if (count < 2U)
    {
        return;
    }

    for (i = 1U; i < count; i++)
    {
        if (path_display_point_in_area(&screen_point_data[i - 1U]) &&
            path_display_point_in_area(&screen_point_data[i]))
        {
            ips200_draw_line((uint16)(screen_point_data[i - 1U].x + display_x),
                             (uint16)(screen_point_data[i - 1U].y + display_y),
                             (uint16)(screen_point_data[i].x + display_x),
                             (uint16)(screen_point_data[i].y + display_y),
                             color);
        }
    }
}

static void path_display_draw_current_position(void)
{
    gps_point current_gps;
    screen_point current_point;
    uint8 point_ready = 0U;

    if (!display_initialized || !gnss.state)
    {
        return;
    }

    current_gps.lat = (double)gnss.latitude;
    current_gps.lon = (double)gnss.longitude;

    if (path_data.point_count > 0U)
    {
        point_ready = gps_point_to_screen(&current_gps, &current_point);
    }
    else
    {
        gps_set_display_area(0, 0, (int16)display_width, (int16)display_height);
        gps_clear_xy_bounds();
        user_gps_transition(&current_gps, 1);
        current_point = screen_point_data[0];
        point_ready = 1U;
    }

    if (point_ready && path_display_point_in_area(&current_point))
    {
        path_display_draw_marker(&current_point, CURRENT_POS_COLOR);
    }
}

void path_display_init(void)
{
    display_x = DISPLAY_AREA_X;
    display_y = DISPLAY_AREA_Y;
    display_width = DISPLAY_AREA_WIDTH;
    display_height = DISPLAY_AREA_HEIGHT;
    path_display_clamp_area();
    display_border_dirty = 1U;
    display_initialized = 1U;
}

void path_display_set_area(uint16 x, uint16 y, uint16 width, uint16 height)
{
    if (display_x != x || display_y != y || display_width != width || display_height != height)
    {
        display_border_dirty = 1U;
    }

    display_x = x;
    display_y = y;
    display_width = width;
    display_height = height;
    path_display_clamp_area();
    display_initialized = 1U;
}

void path_display_draw_map(uint16 color)
{
    if (!display_initialized)
    {
        return;
    }

    if (display_width > 2U && display_height > 2U)
    {
        path_display_fill_rect((uint16)(display_x + 1U),
                               (uint16)(display_y + 1U),
                               (uint16)(display_x + display_width - 2U),
                               (uint16)(display_y + display_height - 2U),
                               RGB565_BLACK);
    }

    path_display_draw_border();
    display_border_dirty = 0U;
    path_display_draw_trajectory(color);
    path_display_draw_current_position();
}
