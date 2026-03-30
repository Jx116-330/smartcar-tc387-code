/**
 * @file display_gps.c
 * @brief GPS 坐标转换与屏幕映射实现
 * @author JX116
 *
 * 这里维护了一套“最近一次轨迹转换”的缓存参数：
 * - 参考点经纬度
 * - 轨迹包围盒最小值
 * - 缩放比例
 *
 * 这样轨迹线画完后，当前位置和回放点就能复用同一套映射，
 * 避免出现“线是一套缩放，点又是另一套缩放”的问题。
 */

#include <GPS/display_gps.h>

/** 轨迹缩放后的四周预留边距，避免线条贴边显示 */
#define GPS_DISPLAY_PADDING      (14.0)

/** 当前映射区域左上角 X */
static int16 display_start_x = 0;
/** 当前映射区域左上角 Y */
static int16 display_start_y = 0;
/** 当前映射区域宽度 */
static int16 display_width = 0;
/** 当前映射区域高度 */
static int16 display_high = 0;
/** 最近一次参与批量变换的点数 */
static int16 transition_point_num = 0;

/** 最近一次轨迹变换使用的参考纬度（弧度） */
static double transform_ref_lat = 0.0;
/** 最近一次轨迹变换使用的参考经度（弧度） */
static double transform_ref_lon = 0.0;
/** 最近一次轨迹包围盒的最小 X */
static double transform_min_x = 0.0;
/** 最近一次轨迹包围盒的最小 Y */
static double transform_min_y = 0.0;
/** 最近一次轨迹映射使用的缩放系数 */
static double transform_scale = 1.0;
/** 最近一次轨迹缩放后在显示区域内的起始 X 偏移 */
static double transform_offset_x = 0.0;
/** 最近一次轨迹缩放后在显示区域内的起始 Y 偏移 */
static double transform_offset_y = 0.0;
/** 最近一次轨迹缩放后的有效内容高度 */
static double transform_content_height = 0.0;
/** 当前是否存在可复用的变换缓存 */
static uint8 transform_valid = 0;
/** 是否使用外部指定的坐标包围盒 */
static uint8 transform_bounds_override_valid = 0;
/** 外部指定包围盒最小 X */
static double transform_bounds_min_x = 0.0;
/** 外部指定包围盒最大 X */
static double transform_bounds_max_x = 0.0;
/** 外部指定包围盒最小 Y */
static double transform_bounds_min_y = 0.0;
/** 外部指定包围盒最大 Y */
static double transform_bounds_max_y = 0.0;

screen_point screen_point_data[DISPLAY_POINT_MAX];

/**
 * @brief 基于最近一次缓存的参考点，将单个经纬度转换成局部平面坐标
 */
static void gps_convert_single_to_xy(const gps_point *gps, double *x, double *y)
{
    double lat = gps->lat * USER_PI / 180.0;
    double lon = gps->lon * USER_PI / 180.0;

    *x = EARTH_RADIUS * (lon - transform_ref_lon) * cos(transform_ref_lat);
    *y = EARTH_RADIUS * (lat - transform_ref_lat);
}

void gps_set_display_area(int16 start_x, int16 start_y, int16 width, int16 height)
{
    display_start_x = start_x;
    display_start_y = start_y;
    display_width = width;
    display_high = height;

    /* 显示区域变化后，之前的缩放结果不再可靠，需要重新计算。 */
    transform_valid = 0;
}

void gps_set_xy_bounds(double min_x, double max_x, double min_y, double max_y)
{
    transform_bounds_min_x = min_x;
    transform_bounds_max_x = max_x;
    transform_bounds_min_y = min_y;
    transform_bounds_max_y = max_y;
    transform_bounds_override_valid = 1;
}

void gps_clear_xy_bounds(void)
{
    transform_bounds_override_valid = 0;
}

void gps_to_xy(gps_point *gps, double *x, double *y)
{
    int i;

    /* 以第一个点作为局部参考点，把经纬度投影到局部平面坐标。 */
    transform_ref_lat = gps[0].lat * USER_PI / 180.0;
    transform_ref_lon = gps[0].lon * USER_PI / 180.0;

    for (i = 0; i < transition_point_num; i++)
    {
        gps_convert_single_to_xy(&gps[i], &x[i], &y[i]);
    }
}

void xy_to_screen(double *x, double *y, int point_num)
{
    int i;
    double min_x = x[0], max_x = x[0];
    double min_y = y[0], max_y = y[0];
    double scale_x, scale_y, scale;
    double available_width;
    double available_height;
    double content_width;
    double content_height;
    double center_value;

    if (transform_bounds_override_valid)
    {
        min_x = transform_bounds_min_x;
        max_x = transform_bounds_max_x;
        min_y = transform_bounds_min_y;
        max_y = transform_bounds_max_y;
    }
    else
    {
        /* 先找出轨迹包围盒，用于后续统一缩放。 */
        for (i = 1; i < point_num; i++)
        {
            if (x[i] < min_x) min_x = x[i];
            if (x[i] > max_x) max_x = x[i];
            if (y[i] < min_y) min_y = y[i];
            if (y[i] > max_y) max_y = y[i];
        }
    }

    /* 单点或近似单点轨迹时，围绕中心对称扩展，避免结果贴在角落。 */
    if (fabs(max_x - min_x) < EPSILON)
    {
        center_value = (max_x + min_x) / 2.0;
        min_x = center_value - 0.5;
        max_x = center_value + 0.5;
    }
    if (fabs(max_y - min_y) < EPSILON)
    {
        center_value = (max_y + min_y) / 2.0;
        min_y = center_value - 0.5;
        max_y = center_value + 0.5;
    }

    available_width = display_width - (GPS_DISPLAY_PADDING * 2.0);
    available_height = display_high - (GPS_DISPLAY_PADDING * 2.0);

    if (available_width < 1.0) available_width = 1.0;
    if (available_height < 1.0) available_height = 1.0;

    scale_x = available_width / (max_x - min_x);
    scale_y = available_height / (max_y - min_y);
    scale = (scale_x < scale_y) ? scale_x : scale_y;
    content_width = (max_x - min_x) * scale;
    content_height = (max_y - min_y) * scale;

    /* 缓存当前映射参数，供当前位置和回放点复用。 */
    transform_min_x = min_x;
    transform_min_y = min_y;
    transform_scale = scale;
    transform_offset_x = display_start_x + ((display_width - content_width) / 2.0);
    transform_offset_y = display_start_y + ((display_high - content_height) / 2.0);
    transform_content_height = content_height;
    transform_valid = 1;

    for (i = 0; i < point_num; i++)
    {
        double screen_x = (x[i] - min_x) * scale + transform_offset_x;
        double screen_y = transform_offset_y + transform_content_height - ((y[i] - min_y) * scale);

        /* 防止结果超出 int16 范围。 */
        if (screen_x > 32767.0) screen_x = 32767.0;
        if (screen_x < -32768.0) screen_x = -32768.0;
        if (screen_y > 32767.0) screen_y = 32767.0;
        if (screen_y < -32768.0) screen_y = -32768.0;

        screen_point_data[i].x = (int16)screen_x;
        screen_point_data[i].y = (int16)screen_y;
    }
}

void get_transition(gps_point *gps, int point_num)
{
    double x[DISPLAY_POINT_MAX];
    double y[DISPLAY_POINT_MAX];

    if (point_num <= 0)
    {
        transform_valid = 0;
        return;
    }
    if (point_num > DISPLAY_POINT_MAX) point_num = DISPLAY_POINT_MAX;
    if (point_num > 32767) point_num = 32767;

    transition_point_num = (int16)point_num;
    gps_to_xy(gps, x, y);
    xy_to_screen(x, y, point_num);
}

void user_gps_transition(gps_point *gps, int point_num)
{
    /* 如果上层还没指定显示区域，就使用默认区域兜底。 */
    if (display_width <= 0 || display_high <= 0)
    {
        gps_set_display_area(0, 0, 220, 120);
    }
    get_transition(gps, point_num);
}

uint8 gps_point_to_screen(const gps_point *gps, screen_point *point)
{
    double x;
    double y;
    double screen_x;
    double screen_y;

    if (NULL == gps || NULL == point || !transform_valid)
    {
        return 0;
    }

    gps_convert_single_to_xy(gps, &x, &y);

    screen_x = (x - transform_min_x) * transform_scale + transform_offset_x;
    screen_y = transform_offset_y + transform_content_height - ((y - transform_min_y) * transform_scale);

    if (screen_x > 32767.0) screen_x = 32767.0;
    if (screen_x < -32768.0) screen_x = -32768.0;
    if (screen_y > 32767.0) screen_y = 32767.0;
    if (screen_y < -32768.0) screen_y = -32768.0;

    point->x = (int16)screen_x;
    point->y = (int16)screen_y;
    return 1;
}

void screen_print_gps_point(void)
{
    int i;

    for (i = 0; i < transition_point_num; i++)
    {
        if (screen_point_data[i].x >= 0 && screen_point_data[i].y >= 0)
        {
            ips200_draw_point((uint16)screen_point_data[i].x,
                              (uint16)screen_point_data[i].y,
                              RGB565_RED);
        }
    }
}
