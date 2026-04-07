#include "icm_attitude.h"

#include <stddef.h>
#include <math.h>
#include "zf_driver_flash.h"

#define GYRO_BIAS_FLASH_SECTOR  0U
#define GYRO_BIAS_FLASH_PAGE    115U
#define GYRO_BIAS_FLASH_WORDS   4U
#define GYRO_BIAS_MAGIC         0x42494153U

#ifndef ICM_ATTITUDE_PI
#define ICM_ATTITUDE_PI 3.14159265358979323846f
#endif

#define ICM_ATTITUDE_KP           1.6f
#define ICM_ATTITUDE_ACC_MIN_NORM 0.60f
#define ICM_ATTITUDE_ACC_MAX_NORM 1.40f
#define ICM_ATTITUDE_CALIB_SAMPLES_DEFAULT 2000U

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float acc_norm_g;
    uint32 update_count;
    float gyro_bias_x_dps;
    float gyro_bias_y_dps;
    float gyro_bias_z_dps;
    float gyro_bias_sum_x_dps;
    float gyro_bias_sum_y_dps;
    float gyro_bias_sum_z_dps;
    uint32 gyro_bias_sample_count;
    uint32 gyro_bias_target_count;
    uint8 gyro_bias_calibrating;
    uint8 gyro_bias_valid;
    uint8 gyro_bias_from_flash;
} icm_attitude_state_t;

static volatile icm_attitude_state_t g_icm_attitude = {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f,
    0U,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0U, 0U,
    0U, 0U, 0U
};

static float icm_attitude_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void icm_attitude_reset_pose(void)
{
    g_icm_attitude.q0 = 1.0f;
    g_icm_attitude.q1 = 0.0f;
    g_icm_attitude.q2 = 0.0f;
    g_icm_attitude.q3 = 0.0f;
    g_icm_attitude.roll_deg = 0.0f;
    g_icm_attitude.pitch_deg = 0.0f;
    g_icm_attitude.yaw_deg = 0.0f;
    g_icm_attitude.acc_norm_g = 0.0f;
    g_icm_attitude.update_count = 0U;
}

static void icm_attitude_reset_gyro_bias_calibration_state(void)
{
    g_icm_attitude.gyro_bias_sum_x_dps = 0.0f;
    g_icm_attitude.gyro_bias_sum_y_dps = 0.0f;
    g_icm_attitude.gyro_bias_sum_z_dps = 0.0f;
    g_icm_attitude.gyro_bias_sample_count = 0U;
    g_icm_attitude.gyro_bias_target_count = 0U;
    g_icm_attitude.gyro_bias_calibrating = 0U;
}

void icm_attitude_reset(void)
{
    icm_attitude_reset_pose();
}

uint8 icm_attitude_save_gyro_bias_to_flash(void)
{
    uint32 buf[GYRO_BIAS_FLASH_WORDS];

    if (!g_icm_attitude.gyro_bias_valid)
    {
        return 0U;
    }

    buf[0] = *((const uint32 *)(const void *)&g_icm_attitude.gyro_bias_x_dps);
    buf[1] = *((const uint32 *)(const void *)&g_icm_attitude.gyro_bias_y_dps);
    buf[2] = *((const uint32 *)(const void *)&g_icm_attitude.gyro_bias_z_dps);
    buf[3] = GYRO_BIAS_MAGIC;

    flash_write_page(GYRO_BIAS_FLASH_SECTOR, GYRO_BIAS_FLASH_PAGE,
                     buf, (uint16)GYRO_BIAS_FLASH_WORDS);
    return 1U;
}

uint8 icm_attitude_load_gyro_bias_from_flash(void)
{
    uint32 buf[GYRO_BIAS_FLASH_WORDS];
    float bx;
    float by;
    float bz;

    flash_read_page(GYRO_BIAS_FLASH_SECTOR, GYRO_BIAS_FLASH_PAGE,
                    buf, (uint16)GYRO_BIAS_FLASH_WORDS);

    if (buf[3] != GYRO_BIAS_MAGIC)
    {
        return 0U;
    }

    bx = *((const float *)(const void *)&buf[0]);
    by = *((const float *)(const void *)&buf[1]);
    bz = *((const float *)(const void *)&buf[2]);

    /* 简单有效性检查：偏置超过 20 dps 视为异常数据 */
    if ((bx < -20.0f) || (bx > 20.0f) ||
        (by < -20.0f) || (by > 20.0f) ||
        (bz < -20.0f) || (bz > 20.0f))
    {
        return 0U;
    }

    g_icm_attitude.gyro_bias_x_dps = bx;
    g_icm_attitude.gyro_bias_y_dps = by;
    g_icm_attitude.gyro_bias_z_dps = bz;
    g_icm_attitude.gyro_bias_valid = 1U;
    g_icm_attitude.gyro_bias_from_flash = 1U;
    return 1U;
}

uint8 icm_attitude_is_gyro_bias_from_flash(void)
{
    return g_icm_attitude.gyro_bias_from_flash;
}

void icm_attitude_init(void)
{
    icm_attitude_reset_pose();
    g_icm_attitude.gyro_bias_x_dps = 0.0f;
    g_icm_attitude.gyro_bias_y_dps = 0.0f;
    g_icm_attitude.gyro_bias_z_dps = 0.0f;
    g_icm_attitude.gyro_bias_valid = 0U;
    g_icm_attitude.gyro_bias_from_flash = 0U;
    icm_attitude_reset_gyro_bias_calibration_state();

    /* 尝试从 flash 恢复上次校准结果 */
    (void)icm_attitude_load_gyro_bias_from_flash();
}

void icm_attitude_start_gyro_bias_calibration(uint32 sample_count)
{
    icm_attitude_reset_pose();
    icm_attitude_reset_gyro_bias_calibration_state();

    g_icm_attitude.gyro_bias_target_count = (sample_count > 0U) ? sample_count : ICM_ATTITUDE_CALIB_SAMPLES_DEFAULT;
    g_icm_attitude.gyro_bias_calibrating = 1U;
}

void icm_attitude_cancel_gyro_bias_calibration(void)
{
    icm_attitude_reset_gyro_bias_calibration_state();
}

uint8 icm_attitude_is_gyro_bias_calibrating(void)
{
    return g_icm_attitude.gyro_bias_calibrating;
}

uint8 icm_attitude_is_gyro_bias_valid(void)
{
    return g_icm_attitude.gyro_bias_valid;
}

void icm_attitude_get_gyro_bias(float *bias_x_dps, float *bias_y_dps, float *bias_z_dps)
{
    if (NULL != bias_x_dps) *bias_x_dps = g_icm_attitude.gyro_bias_x_dps;
    if (NULL != bias_y_dps) *bias_y_dps = g_icm_attitude.gyro_bias_y_dps;
    if (NULL != bias_z_dps) *bias_z_dps = g_icm_attitude.gyro_bias_z_dps;
}

void icm_attitude_get_gyro_bias_calibration_progress(uint32 *sample_count, uint32 *target_count)
{
    if (NULL != sample_count) *sample_count = g_icm_attitude.gyro_bias_sample_count;
    if (NULL != target_count) *target_count = g_icm_attitude.gyro_bias_target_count;
}

void icm_attitude_update(float gyro_x_dps,
                         float gyro_y_dps,
                         float gyro_z_dps,
                         float acc_x_g,
                         float acc_y_g,
                         float acc_z_g,
                         float dt_s)
{
    float q0 = g_icm_attitude.q0;
    float q1 = g_icm_attitude.q1;
    float q2 = g_icm_attitude.q2;
    float q3 = g_icm_attitude.q3;
    float q_dot0;
    float q_dot1;
    float q_dot2;
    float q_dot3;
    float gyro_x_corrected_dps = gyro_x_dps;
    float gyro_y_corrected_dps = gyro_y_dps;
    float gyro_z_corrected_dps = gyro_z_dps;
    float gx;
    float gy;
    float gz;
    float acc_norm = sqrtf(acc_x_g * acc_x_g + acc_y_g * acc_y_g + acc_z_g * acc_z_g);

    if (dt_s <= 0.0f)
    {
        return;
    }

    g_icm_attitude.acc_norm_g = acc_norm;

    if (g_icm_attitude.gyro_bias_calibrating)
    {
        g_icm_attitude.gyro_bias_sum_x_dps += gyro_x_dps;
        g_icm_attitude.gyro_bias_sum_y_dps += gyro_y_dps;
        g_icm_attitude.gyro_bias_sum_z_dps += gyro_z_dps;
        g_icm_attitude.gyro_bias_sample_count++;

        if (g_icm_attitude.gyro_bias_sample_count >= g_icm_attitude.gyro_bias_target_count)
        {
            float sample_count = (float)g_icm_attitude.gyro_bias_sample_count;

            if (sample_count > 0.0f)
            {
                g_icm_attitude.gyro_bias_x_dps = g_icm_attitude.gyro_bias_sum_x_dps / sample_count;
                g_icm_attitude.gyro_bias_y_dps = g_icm_attitude.gyro_bias_sum_y_dps / sample_count;
                g_icm_attitude.gyro_bias_z_dps = g_icm_attitude.gyro_bias_sum_z_dps / sample_count;
                g_icm_attitude.gyro_bias_valid = 1U;
                g_icm_attitude.gyro_bias_from_flash = 0U;
                (void)icm_attitude_save_gyro_bias_to_flash();
            }

            g_icm_attitude.gyro_bias_calibrating = 0U;
            icm_attitude_reset_pose();
        }
        return;
    }

    if (g_icm_attitude.gyro_bias_valid)
    {
        gyro_x_corrected_dps -= g_icm_attitude.gyro_bias_x_dps;
        gyro_y_corrected_dps -= g_icm_attitude.gyro_bias_y_dps;
        gyro_z_corrected_dps -= g_icm_attitude.gyro_bias_z_dps;
    }

    gx = gyro_x_corrected_dps * (ICM_ATTITUDE_PI / 180.0f);
    gy = gyro_y_corrected_dps * (ICM_ATTITUDE_PI / 180.0f);
    gz = gyro_z_corrected_dps * (ICM_ATTITUDE_PI / 180.0f);

    if ((acc_norm > ICM_ATTITUDE_ACC_MIN_NORM) && (acc_norm < ICM_ATTITUDE_ACC_MAX_NORM))
    {
        float ax = acc_x_g / acc_norm;
        float ay = acc_y_g / acc_norm;
        float az = acc_z_g / acc_norm;
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        gx += ICM_ATTITUDE_KP * ex;
        gy += ICM_ATTITUDE_KP * ey;
        gz += ICM_ATTITUDE_KP * ez;
    }

    q_dot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    q_dot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    q_dot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    q_dot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    q0 += q_dot0 * dt_s;
    q1 += q_dot1 * dt_s;
    q2 += q_dot2 * dt_s;
    q3 += q_dot3 * dt_s;

    acc_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (acc_norm > 0.0f)
    {
        q0 /= acc_norm;
        q1 /= acc_norm;
        q2 /= acc_norm;
        q3 /= acc_norm;
    }

    g_icm_attitude.q0 = q0;
    g_icm_attitude.q1 = q1;
    g_icm_attitude.q2 = q2;
    g_icm_attitude.q3 = q3;
    g_icm_attitude.roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3),
                                     1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0f / ICM_ATTITUDE_PI);
    g_icm_attitude.pitch_deg = asinf(icm_attitude_clamp(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * (180.0f / ICM_ATTITUDE_PI);
    g_icm_attitude.yaw_deg = atan2f(2.0f * (q0 * q3 + q1 * q2),
                                    1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (180.0f / ICM_ATTITUDE_PI);
    g_icm_attitude.update_count++;
}

void icm_attitude_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (NULL != roll_deg) *roll_deg = g_icm_attitude.roll_deg;
    if (NULL != pitch_deg) *pitch_deg = g_icm_attitude.pitch_deg;
    if (NULL != yaw_deg) *yaw_deg = g_icm_attitude.yaw_deg;
}

void icm_attitude_get_quaternion(float *q0, float *q1, float *q2, float *q3)
{
    if (NULL != q0) *q0 = g_icm_attitude.q0;
    if (NULL != q1) *q1 = g_icm_attitude.q1;
    if (NULL != q2) *q2 = g_icm_attitude.q2;
    if (NULL != q3) *q3 = g_icm_attitude.q3;
}

float icm_attitude_get_acc_norm_g(void)
{
    return g_icm_attitude.acc_norm_g;
}

uint32 icm_attitude_get_update_count(void)
{
    return g_icm_attitude.update_count;
}
