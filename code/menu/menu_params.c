#include "menu_params.h"

#include <string.h>

#include "zf_driver_flash.h"

/* Flash 参数存储位置定义 */
#define FLASH_SECTOR                0U
#define PARAM_FLASH_LAST_PAGE       127U
#define PARAM_FLASH_PAGE_COUNT      4U
#define PARAM_FLASH_WORDS_PER_PAGE  ((uint32)EEPROM_PAGE_LENGTH)
#define PARAM_FLASH_CAPACITY_WORDS  (PARAM_FLASH_PAGE_COUNT * PARAM_FLASH_WORDS_PER_PAGE)
#define MAGIC_NUM                   0x5A5A5A5AU
#define SAVE_LEN                    ((uint32)((sizeof(MyParams_t) + 3U) / 4U))

static uint8 menu_params_flash_has_capacity(void);
static uint32 menu_params_flash_page_number(uint32 page_index);
static uint32 menu_params_flash_page_word_count(uint32 page_index);

uint8 menu_params_are_valid(const MyParams_t *params)
{
    if (NULL == params)
    {
        return 0U;
    }

    if (params->magic_code != MAGIC_NUM)
    {
        return 0U;
    }

    if (!(params->pid_p >= PARAM_PID_P_MIN && params->pid_p <= PARAM_PID_P_MAX))
    {
        return 0U;
    }

    if (!(params->pid_i >= PARAM_PID_I_MIN && params->pid_i <= PARAM_PID_I_MAX))
    {
        return 0U;
    }

    if (!(params->pid_d >= PARAM_PID_D_MIN && params->pid_d <= PARAM_PID_D_MAX))
    {
        return 0U;
    }

    if (!(params->record_min_distance >= PARAM_RECORD_DISTANCE_MIN && params->record_min_distance <= PARAM_RECORD_DISTANCE_MAX))
    {
        return 0U;
    }

    if (!(params->record_min_interval_ms >= PARAM_RECORD_INTERVAL_MIN && params->record_min_interval_ms <= PARAM_RECORD_INTERVAL_MAX))
    {
        return 0U;
    }

    if (!(params->record_max_speed_kph >= PARAM_RECORD_SPEED_MIN && params->record_max_speed_kph <= PARAM_RECORD_SPEED_MAX))
    {
        return 0U;
    }

    if (!(params->record_min_satellites >= PARAM_RECORD_SAT_MIN && params->record_min_satellites <= PARAM_RECORD_SAT_MAX))
    {
        return 0U;
    }

    if (params->threshold_mode > PARAM_THRESHOLD_MODE_MAX)
    {
        return 0U;
    }

    return 1U;
}

static uint8 menu_params_flash_has_capacity(void)
{
    return (SAVE_LEN <= PARAM_FLASH_CAPACITY_WORDS) ? 1U : 0U;
}

static uint32 menu_params_flash_page_number(uint32 page_index)
{
    return PARAM_FLASH_LAST_PAGE - page_index;
}

static uint32 menu_params_flash_page_word_count(uint32 page_index)
{
    uint32 offset = page_index * PARAM_FLASH_WORDS_PER_PAGE;
    uint32 remaining = (SAVE_LEN > offset) ? (SAVE_LEN - offset) : 0U;

    if (remaining > PARAM_FLASH_WORDS_PER_PAGE)
    {
        remaining = PARAM_FLASH_WORDS_PER_PAGE;
    }

    return remaining;
}

uint8 menu_params_load_from_flash(MyParams_t *params)
{
    uint32 page_index;
    uint32 *buffer = (uint32 *)params;

    if ((NULL == params) || !menu_params_flash_has_capacity())
    {
        return 0U;
    }

    memset(params, 0, sizeof(*params));
    for (page_index = 0U; page_index < PARAM_FLASH_PAGE_COUNT; page_index++)
    {
        uint32 words_this_page = menu_params_flash_page_word_count(page_index);

        if (0U == words_this_page)
        {
            break;
        }

        flash_read_page(FLASH_SECTOR,
                        menu_params_flash_page_number(page_index),
                        buffer + (page_index * PARAM_FLASH_WORDS_PER_PAGE),
                        (uint16)words_this_page);
    }

    return 1U;
}

uint8 menu_params_save_to_flash(const MyParams_t *params)
{
    uint32 page_index;
    MyParams_t save_params;
    const uint32 *buffer;

    if ((NULL == params) || !menu_params_flash_has_capacity())
    {
        return 0U;
    }

    save_params = *params;
    save_params.magic_code = MAGIC_NUM;
    buffer = (const uint32 *)&save_params;

    for (page_index = 0U; page_index < PARAM_FLASH_PAGE_COUNT; page_index++)
    {
        uint32 words_this_page = menu_params_flash_page_word_count(page_index);

        if (0U == words_this_page)
        {
            break;
        }

        flash_write_page(FLASH_SECTOR,
                         menu_params_flash_page_number(page_index),
                         buffer + (page_index * PARAM_FLASH_WORDS_PER_PAGE),
                         (uint16)words_this_page);
    }

    return 1U;
}

void menu_params_apply_record_config(const MyParams_t *params)
{
    if (NULL == params)
    {
        return;
    }

    path_recorder_set_min_distance(params->record_min_distance);
    path_recorder_set_min_interval_ms(params->record_min_interval_ms);
    path_recorder_set_max_speed_kph(params->record_max_speed_kph);
    path_recorder_set_min_satellites(params->record_min_satellites);
}

void menu_params_capture_record_config(MyParams_t *params)
{
    const path_record_config_t *config = path_recorder_get_config();

    if ((NULL == params) || (NULL == config))
    {
        return;
    }

    params->record_min_distance = config->min_record_distance;
    params->record_min_interval_ms = config->min_record_interval_ms;
    params->record_max_speed_kph = config->max_record_speed_kph;
    params->record_min_satellites = config->min_satellites;
}

void menu_params_set_default(MyParams_t *params)
{
    if (NULL == params)
    {
        return;
    }

    params->pid_p = 1.2f;
    params->pid_i = 0.01f;
    params->pid_d = 0.5f;
    params->record_min_distance = MIN_RECORD_DISTANCE;
    params->record_min_interval_ms = MIN_RECORD_INTERVAL_MS;
    params->record_max_speed_kph = MAX_RECORD_SPEED_KPH;
    params->record_min_satellites = GPS_MIN_SATELLITES;
    params->threshold_mode = 0U;
    params->threshold_val = 128U;
    params->magic_code = MAGIC_NUM;
}

uint8 menu_params_load_or_default(MyParams_t *params)
{
    MyParams_t temp_read;

    if (NULL == params)
    {
        return 0U;
    }

    if (menu_params_load_from_flash(&temp_read) && menu_params_are_valid(&temp_read))
    {
        *params = temp_read;
        return 1U;
    }

    menu_params_set_default(params);
    return 0U;
}
