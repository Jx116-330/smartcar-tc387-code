#include "mock_gnss.h"

#include "zf_common_headfile.h"

#define MOCK_GNSS_POINT_COUNT 12U
#define MOCK_GNSS_STEP_MS     150U

typedef struct
{
    double latitude;
    double longitude;
    float speed;
    float direction;
    uint8 satellite_used;
    uint8 state;
} mock_gnss_point_t;

static const mock_gnss_point_t mock_gnss_route[MOCK_GNSS_POINT_COUNT] = {
    {30.572800, 104.066800, 0.0f,   0.0f, 10U, 1U},
    {30.572805, 104.066808, 6.0f,  18.0f, 10U, 1U},
    {30.572812, 104.066820, 9.5f,  24.0f, 10U, 1U},
    {30.572824, 104.066836, 13.0f, 33.0f, 10U, 1U},
    {30.572840, 104.066852, 16.0f, 42.0f, 10U, 1U},
    {30.572860, 104.066872, 18.5f, 46.0f, 10U, 1U},
    {30.572880, 104.066900, 19.0f, 54.0f, 10U, 1U},
    {30.572900, 104.066930, 17.5f, 60.0f, 10U, 1U},
    {30.572918, 104.066965, 14.0f, 74.0f, 10U, 1U},
    {30.572930, 104.066995, 10.0f, 88.0f, 10U, 1U},
    {30.572936, 104.067020, 5.0f, 100.0f, 9U,  1U},
    {30.572940, 104.067030, 0.0f, 110.0f, 9U,  1U}
};

static uint8 mock_gnss_enabled = 0U;
static uint32 mock_gnss_last_ms = 0U;
static uint8 mock_gnss_index = 0U;

static void mock_gnss_apply_point(const mock_gnss_point_t *point)
{
    if (NULL == point)
    {
        return;
    }

    gnss.state = point->state;
    gnss.satellite_used = point->satellite_used;
    gnss.latitude = point->latitude;
    gnss.longitude = point->longitude;
    gnss.speed = point->speed;
    gnss.direction = point->direction;
    gnss_flag = 1U;
}

void mock_gnss_init(void)
{
    mock_gnss_enabled = 0U;
    mock_gnss_last_ms = system_getval_ms();
    mock_gnss_index = 0U;
}

void mock_gnss_restart(void)
{
    mock_gnss_index = 0U;
    mock_gnss_last_ms = system_getval_ms();
    mock_gnss_apply_point(&mock_gnss_route[0]);
}

void mock_gnss_set_enabled(uint8 enabled)
{
    mock_gnss_enabled = enabled ? 1U : 0U;
    mock_gnss_last_ms = system_getval_ms();
    if (mock_gnss_enabled)
    {
        mock_gnss_restart();
    }
}

uint8 mock_gnss_is_enabled(void)
{
    return mock_gnss_enabled;
}

void mock_gnss_task(void)
{
    uint32 now_ms;

    if (!mock_gnss_enabled)
    {
        return;
    }

    now_ms = system_getval_ms();
    if ((uint32)(now_ms - mock_gnss_last_ms) < MOCK_GNSS_STEP_MS)
    {
        return;
    }

    mock_gnss_last_ms = now_ms;
    mock_gnss_apply_point(&mock_gnss_route[mock_gnss_index]);

    mock_gnss_index++;
    if (mock_gnss_index >= MOCK_GNSS_POINT_COUNT)
    {
        mock_gnss_index = 0U;
    }
}
