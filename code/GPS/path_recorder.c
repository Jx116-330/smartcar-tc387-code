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
#define PATH_FLASH_MAGIC            0x5452414BU
#define PATH_FLASH_VERSION          1U
#define PATH_FLASH_LAST_PAGE        123U
#define PATH_FLASH_PAGE_COUNT       8U
#define PATH_FLASH_WORDS_PER_PAGE   ((uint32)EEPROM_PAGE_LENGTH)
#define PATH_FLASH_CAPACITY_WORDS   (PATH_FLASH_PAGE_COUNT * PATH_FLASH_WORDS_PER_PAGE)

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 point_count;
    uint32 total_time_ms;
    float total_distance_m;
    uint32 reserved;
} path_flash_header_t;

#define PATH_FLASH_HEADER_WORDS     ((uint32)((sizeof(path_flash_header_t) + 3U) / 4U))
#define PATH_FLASH_POINT_WORDS      ((uint32)((sizeof(path_point_t) + 3U) / 4U))

path_data_t path_data;
static path_record_config_t path_record_config;
static uint32 path_flash_word_buffer[EEPROM_PAGE_LENGTH];

static uint8 is_valid_gps_fix(void);
static void path_recorder_reset_runtime(void);
static void path_recorder_update_last_sample(const path_point_t *point);
static void path_recorder_fill_current_point(path_point_t *point, uint32 timestamp);
static void update_path_statistics(path_point_t* new_point, float segment_distance);
static uint8 path_recorder_add_point(path_point_t* point, float segment_distance);
static float path_recorder_calculate_distance(double lat1, double lon1, double lat2, double lon2);
static uint8 path_flash_has_capacity(uint16 point_count);
static uint32 path_flash_total_words(uint16 point_count);
static uint32 path_flash_page_number(uint32 page_index);
static uint32 path_flash_page_word_count(uint32 total_words, uint32 page_index);
static void path_flash_fill_header(path_flash_header_t *header);
static uint8 path_flash_header_is_valid(const path_flash_header_t *header);
static void path_flash_restore_runtime_state(const path_flash_header_t *header);
static void path_flash_read_header(path_flash_header_t *header);
static void path_flash_read_page_words(uint32 page_index, uint32 words_this_page);
static void path_flash_write_page_words(uint32 page_index, uint32 words_this_page);
static void path_flash_build_page_words(uint32 page_index, uint32 total_words, const path_flash_header_t *header);
static uint8 path_flash_save_current_track(void);
static uint8 path_flash_load_saved_track(void);

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

static void path_recorder_reset_runtime(void)
{
    memset(&path_data, 0, sizeof(path_data));
    path_data.state = PATH_STATE_IDLE;
}

static void path_recorder_update_last_sample(const path_point_t *point)
{
    if (NULL == point)
    {
        return;
    }

    path_data.last_latitude = point->latitude;
    path_data.last_longitude = point->longitude;
    path_data.last_record_time = point->timestamp;
}

static void path_recorder_fill_current_point(path_point_t *point, uint32 timestamp)
{
    if (NULL == point)
    {
        return;
    }

    point->latitude = gnss.latitude;
    point->longitude = gnss.longitude;
    point->timestamp = timestamp;
    point->speed = gnss.speed;
    point->direction = gnss.direction;
    point->satellite_count = gnss.satellite_used;
    point->fix_quality = gnss.state;
}

static uint8 path_flash_has_capacity(uint16 point_count)
{
    return (path_flash_total_words(point_count) <= PATH_FLASH_CAPACITY_WORDS) ? 1U : 0U;
}

/* Flash layout helpers keep the page packing rules in one place. */
static uint32 path_flash_total_words(uint16 point_count)
{
    return PATH_FLASH_HEADER_WORDS + ((uint32)point_count * PATH_FLASH_POINT_WORDS);
}

static uint32 path_flash_page_number(uint32 page_index)
{
    return PATH_FLASH_LAST_PAGE - page_index;
}

static uint32 path_flash_page_word_count(uint32 total_words, uint32 page_index)
{
    uint32 offset = page_index * PATH_FLASH_WORDS_PER_PAGE;
    uint32 remaining = (total_words > offset) ? (total_words - offset) : 0U;

    if (remaining > PATH_FLASH_WORDS_PER_PAGE)
    {
        remaining = PATH_FLASH_WORDS_PER_PAGE;
    }

    return remaining;
}

static void path_flash_fill_header(path_flash_header_t *header)
{
    if (NULL == header)
    {
        return;
    }

    header->magic = PATH_FLASH_MAGIC;
    header->version = PATH_FLASH_VERSION;
    header->point_count = path_data.point_count;
    header->total_time_ms = path_data.total_time;
    header->total_distance_m = path_data.total_distance;
    header->reserved = 0U;
}

static uint8 path_flash_header_is_valid(const path_flash_header_t *header)
{
    if (NULL == header)
    {
        return 0U;
    }

    return ((header->magic == PATH_FLASH_MAGIC) &&
            (header->version == PATH_FLASH_VERSION) &&
            path_flash_has_capacity(header->point_count)) ? 1U : 0U;
}

static void path_flash_restore_runtime_state(const path_flash_header_t *header)
{
    path_recorder_reset_runtime();

    if (NULL == header)
    {
        return;
    }

    path_data.point_count = header->point_count;
    path_data.total_time = header->total_time_ms;
    path_data.total_distance = header->total_distance_m;
}

static void path_flash_read_header(path_flash_header_t *header)
{
    if (NULL == header)
    {
        return;
    }

    memset(header, 0, sizeof(*header));
    flash_read_page(0U, PATH_FLASH_LAST_PAGE, (uint32 *)header, (uint16)PATH_FLASH_HEADER_WORDS);
}

static void path_flash_read_page_words(uint32 page_index, uint32 words_this_page)
{
    memset(path_flash_word_buffer, 0, sizeof(path_flash_word_buffer));
    flash_read_page(0U,
                    path_flash_page_number(page_index),
                    path_flash_word_buffer,
                    (uint16)words_this_page);
}

static void path_flash_write_page_words(uint32 page_index, uint32 words_this_page)
{
    flash_write_page(0U,
                     path_flash_page_number(page_index),
                     path_flash_word_buffer,
                     (uint16)words_this_page);
}

static void path_flash_build_page_words(uint32 page_index, uint32 total_words, const path_flash_header_t *header)
{
    uint32 page_offset = page_index * PATH_FLASH_WORDS_PER_PAGE;
    uint32 words_this_page = path_flash_page_word_count(total_words, page_index);
    const uint32 *header_words = (const uint32 *)header;
    const uint32 *point_words = (const uint32 *)path_data.points;
    uint32 word_index;

    memset(path_flash_word_buffer, 0, sizeof(path_flash_word_buffer));
    for (word_index = 0U; word_index < words_this_page; word_index++)
    {
        uint32 global_word_index = page_offset + word_index;

        if (global_word_index < PATH_FLASH_HEADER_WORDS)
        {
            path_flash_word_buffer[word_index] = header_words[global_word_index];
        }
        else
        {
            path_flash_word_buffer[word_index] = point_words[global_word_index - PATH_FLASH_HEADER_WORDS];
        }
    }
}

/* Flash persistence is manual: menu actions call save/load explicitly. */
static uint8 path_flash_save_current_track(void)
{
    path_flash_header_t header;
    uint32 total_words;
    uint32 page_index;

    if (0U == path_data.point_count)
    {
        return 0U;
    }

    if (!path_flash_has_capacity(path_data.point_count))
    {
        return 0U;
    }

    path_flash_fill_header(&header);
    total_words = path_flash_total_words(path_data.point_count);

    for (page_index = 0U; page_index < PATH_FLASH_PAGE_COUNT; page_index++)
    {
        uint32 words_this_page = path_flash_page_word_count(total_words, page_index);

        if (0U == words_this_page)
        {
            break;
        }

        path_flash_build_page_words(page_index, total_words, &header);
        path_flash_write_page_words(page_index, words_this_page);
    }

    return 1U;
}

static uint8 path_flash_load_saved_track(void)
{
    path_flash_header_t header;
    uint32 total_words;
    uint32 page_index;
    uint32 *point_words = (uint32 *)path_data.points;

    path_flash_read_header(&header);

    if (!path_flash_header_is_valid(&header))
    {
        return 0U;
    }

    path_flash_restore_runtime_state(&header);
    total_words = path_flash_total_words(header.point_count);

    for (page_index = 0U; page_index < PATH_FLASH_PAGE_COUNT; page_index++)
    {
        uint32 words_this_page = path_flash_page_word_count(total_words, page_index);
        uint32 word_index;

        if (0U == words_this_page)
        {
            break;
        }

        path_flash_read_page_words(page_index, words_this_page);

        for (word_index = 0U; word_index < words_this_page; word_index++)
        {
            uint32 global_word_index = (page_index * PATH_FLASH_WORDS_PER_PAGE) + word_index;

            if (global_word_index < PATH_FLASH_HEADER_WORDS)
            {
                continue;
            }

            point_words[global_word_index - PATH_FLASH_HEADER_WORDS] = path_flash_word_buffer[word_index];
        }
    }

    if (path_data.point_count > 0U)
    {
        path_point_t *last_point = &path_data.points[path_data.point_count - 1U];

        path_data.state = PATH_STATE_COMPLETED;
        path_recorder_update_last_sample(last_point);
    }

    return 1U;
}

void path_recorder_init(void)
{
    path_recorder_reset_runtime();
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

    path_recorder_reset_runtime();
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
    path_recorder_reset_runtime();
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
    path_recorder_update_last_sample(point);
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

    path_recorder_fill_current_point(&point, current_time);
    if (!path_recorder_add_point(&point, distance))
    {
        path_recorder_stop();
    }
}

uint8 path_recorder_save_to_flash(void)
{
    return path_flash_save_current_track();
}

uint8 path_recorder_load_from_flash(void)
{
    return path_flash_load_saved_track();
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
