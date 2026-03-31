/***
 * @file path_recorder.c
 * @brief GPS轨迹记录模块实现
 * @author JX116
 * @date 2026-03-21
 * @version 1.0
 *
 * @details 实现GPS轨迹记录、轨迹点管理与统计功能。
 ***/

#include "path_recorder.h"
#include "display_gps.h"
#include <string.h>
#include <math.h>

#define get_current_time_ms()   system_getval_ms()

path_data_t path_data;
static path_record_config_t path_record_config;

static uint8 is_valid_gps_fix(void);
static void update_path_statistics(path_point_t* new_point, float segment_distance);
static uint8 path_recorder_add_point(path_point_t* point, float segment_distance);
static float path_recorder_calculate_distance(double lat1, double lon1, double lat2, double lon2);

static float path_recorder_clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static uint32 path_recorder_clamp_u32(uint32 value, uint32 min_value, uint32 max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static uint8 is_valid_gps_fix(void)
{
    return (gnss.state == 1 &&
            gnss.satellite_used >= path_record_config.min_satellites &&
            gnss.speed <= path_record_config.max_record_speed_kph &&
            gnss.latitude != 0.0 &&
            gnss.longitude != 0.0);
}

void path_recorder_init(void)
{
    memset(&path_data, 0, sizeof(path_data));
    path_data.state = PATH_STATE_IDLE;
    path_recorder_reset_config();
}

void path_recorder_reset_config(void)
{
    path_record_config.min_record_distance = MIN_RECORD_DISTANCE;
    path_record_config.min_record_interval_ms = MIN_RECORD_INTERVAL_MS;
    path_record_config.min_satellites = GPS_MIN_SATELLITES;
    path_record_config.max_record_speed_kph = MAX_RECORD_SPEED_KPH;
}

uint8 path_recorder_start(void)
{
    if (!is_valid_gps_fix())
    {
        return 0;
    }

    if (path_data.state == PATH_STATE_RECORDING)
    {
        return 1;
    }

    path_data.state = PATH_STATE_RECORDING;
    path_data.last_record_time = get_current_time_ms();
    path_data.last_latitude  = gnss.latitude;
    path_data.last_longitude = gnss.longitude;
    return 1;
}

uint8 path_recorder_start_new(void)
{
    if (!is_valid_gps_fix())
    {
        return 0;
    }

    path_recorder_clear();
    return path_recorder_start();
}

void path_recorder_stop(void)
{
    if (path_data.state == PATH_STATE_RECORDING)
    {
        path_data.state = PATH_STATE_COMPLETED;
    }
}

void path_recorder_clear(void)
{
    memset(&path_data, 0, sizeof(path_data));
    path_data.state = PATH_STATE_IDLE;
}

static uint8 path_recorder_add_point(path_point_t* point, float segment_distance)
{
    if (path_data.point_count >= MAX_PATH_POINTS || point == NULL)
    {
        return 0;
    }

    path_data.points[path_data.point_count] = *point;
    path_data.point_count++;
    update_path_statistics(point, segment_distance);
    path_data.last_latitude = point->latitude;
    path_data.last_longitude = point->longitude;
    path_data.last_record_time = point->timestamp;
    return 1;
}

void path_recorder_task(void)
{
    float distance = 0.0f;
    uint32 current_time;
    path_point_t point;

    if (path_data.state != PATH_STATE_RECORDING)
    {
        return;
    }

    if (!is_valid_gps_fix())
    {
        return;
    }

    current_time = get_current_time_ms();

    if (path_data.point_count > 0)
    {
        if (current_time - path_data.last_record_time < path_record_config.min_record_interval_ms)
        {
            return;
        }
        distance = path_recorder_calculate_distance(
            path_data.last_latitude, path_data.last_longitude,
            gnss.latitude, gnss.longitude);
        if (distance < path_record_config.min_record_distance)
        {
            return;
        }
    }

    point.latitude       = gnss.latitude;
    point.longitude      = gnss.longitude;
    point.timestamp      = current_time;
    point.speed          = gnss.speed;
    point.direction      = gnss.direction;
    point.satellite_count = gnss.satellite_used;
    point.fix_quality    = gnss.state;
    path_recorder_add_point(&point, distance);
}

const path_record_config_t *path_recorder_get_config(void)
{
    return &path_record_config;
}

void path_recorder_set_min_distance(float value)
{
    path_record_config.min_record_distance = path_recorder_clamp_float(value, 0.05f, 5.0f);
}

void path_recorder_set_min_interval_ms(uint32 value)
{
    path_record_config.min_record_interval_ms = path_recorder_clamp_u32(value, 10U, 1000U);
}

void path_recorder_set_min_satellites(uint8 value)
{
    path_record_config.min_satellites = (uint8)path_recorder_clamp_u32(value, 3U, 12U);
}

void path_recorder_set_max_speed_kph(float value)
{
    path_record_config.max_record_speed_kph = path_recorder_clamp_float(value, 5.0f, 120.0f);
}

static float path_recorder_calculate_distance(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = (lat2 - lat1) * USER_PI / 180.0;
    double dlon = (lon2 - lon1) * USER_PI / 180.0;
    double sin_dlat = sin(dlat / 2.0);
    double sin_dlon = sin(dlon / 2.0);
    double a = sin_dlat * sin_dlat +
               cos(lat1 * USER_PI / 180.0) * cos(lat2 * USER_PI / 180.0) *
               sin_dlon * sin_dlon;
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return (float)(EARTH_RADIUS * c);
}

static void update_path_statistics(path_point_t* new_point, float segment_distance)
{
    if (path_data.point_count > 1)
    {
        path_data.total_distance += segment_distance;
        path_data.total_time = new_point->timestamp - path_data.points[0].timestamp;
    }
}

void path_recorder_get_stats(float* distance, uint32* time, float* avg_speed)
{
    if (distance) *distance = path_data.total_distance;
    if (time) *time = path_data.total_time;
    if (avg_speed)
    {
        if (path_data.total_time > 0)
            *avg_speed = (path_data.total_distance / ((float)path_data.total_time / 1000.0f)) * 3.6f;
        else
            *avg_speed = 0.0f;
    }
}

path_state_enum path_recorder_get_state(void)
{
    return path_data.state;
}

uint16 path_recorder_get_point_count(void)
{
    return path_data.point_count;
}
