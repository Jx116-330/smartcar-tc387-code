/**
 * @file path_recorder.h
 * @brief GPS 轨迹记录模块头文件
 * @author JX116
 * @date 2026-03-21
 * @version 1.0
 *
 * 当前模块只保留自动轨迹记录所需的核心能力：
 * 1. 开始新的轨迹记录
 * 2. 停止或清空当前轨迹
 * 3. 周期性自动打点
 * 4. 提供轨迹统计信息
 */

#ifndef _PATH_RECORDER_H_
#define _PATH_RECORDER_H_

#include "path_config.h"
#include "zf_common_headfile.h"
#include "zf_device_gnss.h"
#include "zf_driver_timer.h"

typedef enum
{
    PATH_STATE_IDLE = 0,
    PATH_STATE_RECORDING,
    PATH_STATE_COMPLETED
} path_state_enum;

typedef struct
{
    float min_record_distance;
    uint32 min_record_interval_ms;
    uint8 min_satellites;
    float max_record_speed_kph;
} path_record_config_t;

typedef struct
{
    float latitude;
    float longitude;
    uint32 timestamp;
    float speed;
    float direction;
    uint8 satellite_count;
    uint8 fix_quality;
} path_point_t;

typedef struct
{
    path_point_t points[MAX_PATH_POINTS];
    uint16 point_count;
    path_state_enum state;
    uint32 last_record_time;
    float last_latitude;
    float last_longitude;
    float total_distance;
    uint32 total_time;
} path_data_t;

extern path_data_t path_data;

void path_recorder_init(void);
void path_recorder_reset_config(void);
uint8 path_recorder_start(void);
uint8 path_recorder_start_new(void);
void path_recorder_stop(void);
void path_recorder_clear(void);
void path_recorder_task(void);
const path_record_config_t *path_recorder_get_config(void);
void path_recorder_set_min_distance(float value);
void path_recorder_set_min_interval_ms(uint32 value);
void path_recorder_set_min_satellites(uint8 value);
void path_recorder_set_max_speed_kph(float value);
void path_recorder_get_stats(float* distance, uint32* time, float* avg_speed);
path_state_enum path_recorder_get_state(void);
uint16 path_recorder_get_point_count(void);

#endif
