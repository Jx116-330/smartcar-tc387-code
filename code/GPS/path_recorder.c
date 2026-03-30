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

static uint8 is_valid_gps_fix(void);
static void update_path_statistics(path_point_t* new_point);
static uint8 path_recorder_should_record(double lat, double lon, uint32 current_time);
static uint8 path_recorder_add_point(path_point_t* point);
static float path_recorder_calculate_distance(double lat1, double lon1, double lat2, double lon2);

static uint8 is_valid_gps_fix(void)
{
    return (gnss.state == 1 &&
            gnss.satellite_used >= GPS_MIN_SATELLITES &&
            gnss.speed <= MAX_RECORD_SPEED_KPH &&
            gnss.latitude != 0.0 &&
            gnss.longitude != 0.0);
}

void path_recorder_init(void)
{
    memset(&path_data, 0, sizeof(path_data));
    path_data.state = PATH_STATE_IDLE;
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
    path_data.last_latitude = (float)gnss.latitude;
    path_data.last_longitude = (float)gnss.longitude;
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

static uint8 path_recorder_should_record(double lat, double lon, uint32 current_time)
{
    float distance;

    if (path_data.point_count == 0)
    {
        return 1;
    }

    if (current_time - path_data.last_record_time < MIN_RECORD_INTERVAL_MS)
    {
        return 0;
    }

    distance = path_recorder_calculate_distance(path_data.last_latitude, path_data.last_longitude, lat, lon);
    return (distance >= MIN_RECORD_DISTANCE);
}

static uint8 path_recorder_add_point(path_point_t* point)
{
    if (path_data.point_count >= MAX_PATH_POINTS || point == NULL)
    {
        return 0;
    }

    path_data.points[path_data.point_count] = *point;
    path_data.point_count++;
    update_path_statistics(point);
    path_data.last_latitude = point->latitude;
    path_data.last_longitude = point->longitude;
    path_data.last_record_time = point->timestamp;
    return 1;
}

void path_recorder_task(void)
{
    if (path_data.state != PATH_STATE_RECORDING)
    {
        return;
    }

    if (!is_valid_gps_fix())
    {
        return;
    }

    uint32 current_time = get_current_time_ms();
    if (path_recorder_should_record(gnss.latitude, gnss.longitude, current_time))
    {
        path_point_t point;
        point.latitude = (float)gnss.latitude;
        point.longitude = (float)gnss.longitude;
        point.timestamp = current_time;
        point.speed = gnss.speed;
        point.direction = gnss.direction;
        point.satellite_count = gnss.satellite_used;
        point.fix_quality = gnss.state;
        path_recorder_add_point(&point);
    }
}

static float path_recorder_calculate_distance(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = (lat2 - lat1) * USER_PI / 180.0;
    double dlon = (lon2 - lon1) * USER_PI / 180.0;
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1 * USER_PI / 180.0) * cos(lat2 * USER_PI / 180.0) *
               sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return (float)(EARTH_RADIUS * c);
}

static void update_path_statistics(path_point_t* new_point)
{
    if (path_data.point_count > 1)
    {
        path_point_t* prev = &path_data.points[path_data.point_count - 2];
        path_data.total_distance += path_recorder_calculate_distance(
            prev->latitude, prev->longitude, new_point->latitude, new_point->longitude);
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
