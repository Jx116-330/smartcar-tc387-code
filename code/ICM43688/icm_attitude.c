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

#define ICM_ATTITUDE_KP                    1.6f
#define ICM_ATTITUDE_KI                    0.005f   /* 积分增益，保守慢修正 */
#define ICM_ATTITUDE_ACC_MIN_NORM          0.60f
#define ICM_ATTITUDE_ACC_MAX_NORM          1.40f
#define ICM_ATTITUDE_KI_ACC_MIN            0.88f    /* Ki 门控：加速度模长下限 */
#define ICM_ATTITUDE_KI_ACC_MAX            1.12f    /* Ki 门控：加速度模长上限 */
#define ICM_ATTITUDE_KI_GYRO_MAX_DPS       20.0f    /* Ki 门控：角速度幅值上限 */
#define ICM_ATTITUDE_KI_INT_LIMIT          0.40f    /* 积分项饱和限幅（无量纲误差×s；最大修正 Ki×0.40=0.002 rad/s） */
#define ICM_ATTITUDE_KI_LEAK               0.9999f  /* 极端情况泄放系数：1s后保留~90%，10s后保留~37% */
#define ICM_ATTITUDE_BIAS_TRACK_ACC_MIN    0.97f    /* ???????????????????? */
#define ICM_ATTITUDE_BIAS_TRACK_ACC_MAX    1.03f
#define ICM_ATTITUDE_BIAS_TRACK_GYRO_MAX   6.0f     /* dps????????? */
#define ICM_ATTITUDE_BIAS_TRACK_RATE       0.03f    /* 1/s?? 33s ????????? */
#define ICM_ATTITUDE_CALIB_SAMPLES_DEFAULT 20000U

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
    float ex_int;   /* Mahony Ki 积分状态 x（roll/pitch 修正） */
    float ey_int;   /* Mahony Ki 积分状态 y（roll/pitch 修正） */
    /* ez_int 不使用：6轴无航向参考，gz 不加 Ki，避免 yaw 自强化漂移 */

    /* ---- yaw 调试 / 调参状态 ---- */
    float yaw_gz_raw;       /* 最新 gz 原始值（dps） */
    float yaw_gz_bias;      /* 实际作用的 z 轴 bias（主校准 + 手动） */
    float yaw_gz_comp;      /* gz 去偏后（dps） */
    float yaw_gz_scaled;    /* gz 去偏 + 缩放后（dps，进入积分） */
    float yaw_integral;     /* gz_scaled 纯积分累计（度，不回绕） */
    float yaw_final;        /* 当前 yaw_deg（四元数输出，与 yaw_deg 相同） */
    float yaw_correction;   /* Mahony Kp*ez 对 gz 的修正量（rad/s） */
    float yaw_dt_s;         /* 上次 update 使用的 dt */
    float yaw_manual_bias;  /* 额外手动 z-bias（dps），叠加在主校准之上 */
    float yaw_scale;        /* z 轴陀螺缩放因子，默认 1.0 */
    uint8  yaw_dbg_en;      /* 调试使能标志 */
    uint32 yaw_param_ver;   /* 参数版本号，SET/ZERO 后自增 */

    /* yaw zero 状态 */
    uint8  yaw_zero_busy;
    uint8  yaw_zero_ok;
    uint32 yaw_zero_n;
    uint32 yaw_zero_target;
    float  yaw_zero_sum;
    float  yaw_zero_gbz;    /* zero 结果：gz_raw 均值 */

    /* yaw test 状态 */
    uint8  yaw_test_on;
    float  yaw_test_start_deg;
    float  yaw_test_last_delta;
} icm_attitude_state_t;

static volatile icm_attitude_state_t g_icm_attitude = {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f,
    0U,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0U, 0U,
    0U, 0U, 0U,
    0.0f, 0.0f
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
    g_icm_attitude.ex_int = 0.0f;
    g_icm_attitude.ey_int = 0.0f;
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

static void icm_attitude_track_gyro_bias(float gyro_x_dps,
                                         float gyro_y_dps,
                                         float gyro_z_dps,
                                         float acc_norm_g,
                                         float dt_s)
{
    float gyro_sq;
    float gyro_lim_sq;
    float alpha;

    if ((!g_icm_attitude.gyro_bias_valid) || (dt_s <= 0.0f))
    {
        return;
    }

    if ((acc_norm_g < ICM_ATTITUDE_BIAS_TRACK_ACC_MIN) ||
        (acc_norm_g > ICM_ATTITUDE_BIAS_TRACK_ACC_MAX))
    {
        return;
    }

    gyro_sq = gyro_x_dps * gyro_x_dps
            + gyro_y_dps * gyro_y_dps
            + gyro_z_dps * gyro_z_dps;
    gyro_lim_sq = ICM_ATTITUDE_BIAS_TRACK_GYRO_MAX * ICM_ATTITUDE_BIAS_TRACK_GYRO_MAX;
    if (gyro_sq > gyro_lim_sq)
    {
        return;
    }

    alpha = icm_attitude_clamp(ICM_ATTITUDE_BIAS_TRACK_RATE * dt_s, 0.0f, 1.0f);
    g_icm_attitude.gyro_bias_x_dps += (gyro_x_dps - g_icm_attitude.gyro_bias_x_dps) * alpha;
    g_icm_attitude.gyro_bias_y_dps += (gyro_y_dps - g_icm_attitude.gyro_bias_y_dps) * alpha;
    g_icm_attitude.gyro_bias_z_dps += (gyro_z_dps - g_icm_attitude.gyro_bias_z_dps) * alpha;
    g_icm_attitude.gyro_bias_from_flash = 0U;
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

    /* yaw 调试状态初始化 */
    g_icm_attitude.yaw_gz_raw       = 0.0f;
    g_icm_attitude.yaw_gz_bias      = 0.0f;
    g_icm_attitude.yaw_gz_comp      = 0.0f;
    g_icm_attitude.yaw_gz_scaled    = 0.0f;
    g_icm_attitude.yaw_integral     = 0.0f;
    g_icm_attitude.yaw_final        = 0.0f;
    g_icm_attitude.yaw_correction   = 0.0f;
    g_icm_attitude.yaw_dt_s         = 0.0f;
    g_icm_attitude.yaw_manual_bias  = 0.0f;
    g_icm_attitude.yaw_scale        = 1.0f;
    g_icm_attitude.yaw_dbg_en       = 0U;
    g_icm_attitude.yaw_param_ver    = 0U;
    g_icm_attitude.yaw_zero_busy    = 0U;
    g_icm_attitude.yaw_zero_ok      = 0U;
    g_icm_attitude.yaw_zero_n       = 0U;
    g_icm_attitude.yaw_zero_target  = 0U;
    g_icm_attitude.yaw_zero_sum     = 0.0f;
    g_icm_attitude.yaw_zero_gbz     = 0.0f;
    g_icm_attitude.yaw_test_on      = 0U;
    g_icm_attitude.yaw_test_start_deg    = 0.0f;
    g_icm_attitude.yaw_test_last_delta   = 0.0f;

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

    icm_attitude_track_gyro_bias(gyro_x_dps,
                                gyro_y_dps,
                                gyro_z_dps,
                                acc_norm,
                                dt_s);

    /* ---- yaw zero 静止采样（采集 gz 原始值） ---- */
    if (g_icm_attitude.yaw_zero_busy)
    {
        g_icm_attitude.yaw_zero_sum += gyro_z_dps;
        g_icm_attitude.yaw_zero_n++;
        if (g_icm_attitude.yaw_zero_n >= g_icm_attitude.yaw_zero_target)
        {
            float gz_mean = g_icm_attitude.yaw_zero_sum
                          / (float)g_icm_attitude.yaw_zero_n;
            g_icm_attitude.yaw_zero_gbz = gz_mean;
            /* 自动修正 manual_bias 使 gz_comp ≈ 0 */
            g_icm_attitude.yaw_manual_bias = gz_mean
                - (g_icm_attitude.gyro_bias_valid
                   ? g_icm_attitude.gyro_bias_z_dps : 0.0f);
            g_icm_attitude.yaw_param_ver++;
            g_icm_attitude.yaw_zero_busy = 0U;
            g_icm_attitude.yaw_zero_ok   = 1U;
        }
    }

    if (g_icm_attitude.gyro_bias_valid)
    {
        gyro_x_corrected_dps -= g_icm_attitude.gyro_bias_x_dps;
        gyro_y_corrected_dps -= g_icm_attitude.gyro_bias_y_dps;
        gyro_z_corrected_dps -= g_icm_attitude.gyro_bias_z_dps;
    }

    /* ---- 额外手动 bias + 缩放（仅 z 轴） ---- */
    {
        float gz_scaled_dps;
        gyro_z_corrected_dps -= g_icm_attitude.yaw_manual_bias;
        gz_scaled_dps = (g_icm_attitude.yaw_scale > 0.01f)
                      ? gyro_z_corrected_dps * g_icm_attitude.yaw_scale
                      : gyro_z_corrected_dps;

        /* 记录调试量 */
        g_icm_attitude.yaw_gz_raw    = gyro_z_dps;
        g_icm_attitude.yaw_gz_bias   = (g_icm_attitude.gyro_bias_valid
                                         ? g_icm_attitude.gyro_bias_z_dps
                                         : 0.0f)
                                       + g_icm_attitude.yaw_manual_bias;
        g_icm_attitude.yaw_gz_comp   = gyro_z_corrected_dps;
        g_icm_attitude.yaw_gz_scaled = gz_scaled_dps;
        g_icm_attitude.yaw_dt_s      = dt_s;
        /* 积分累计（度，用于观测纯陀螺积分路径） */
        g_icm_attitude.yaw_integral += gz_scaled_dps * dt_s;

        gx = gyro_x_corrected_dps * (ICM_ATTITUDE_PI / 180.0f);
        gy = gyro_y_corrected_dps * (ICM_ATTITUDE_PI / 180.0f);
        gz = gz_scaled_dps        * (ICM_ATTITUDE_PI / 180.0f);
    }

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

        /* 捕获 yaw 修正量（Kp*ez，rad/s） */
        g_icm_attitude.yaw_correction = ICM_ATTITUDE_KP * ez;

        /* Kp 比例反馈（保持原有逻辑不变） */
        gx += ICM_ATTITUDE_KP * ex;
        gy += ICM_ATTITUDE_KP * ey;
        gz += ICM_ATTITUDE_KP * ez;

        /* Ki 门控：加速度模长更严格 + 角速度幅值较小时，才认为当前处于准静止可信状态 */
        {
            float gyro_sq = gyro_x_corrected_dps * gyro_x_corrected_dps
                          + gyro_y_corrected_dps * gyro_y_corrected_dps
                          + gyro_z_corrected_dps * gyro_z_corrected_dps;
            float gyro_lim_sq = ICM_ATTITUDE_KI_GYRO_MAX_DPS * ICM_ATTITUDE_KI_GYRO_MAX_DPS;

            if ((acc_norm > ICM_ATTITUDE_KI_ACC_MIN) && (acc_norm < ICM_ATTITUDE_KI_ACC_MAX)
                && (gyro_sq < gyro_lim_sq))
            {
                /* 可信状态：缓慢累积积分，附带限幅防止 windup
                 * 仅累积 x/y 两轴：6轴方案重力叉积对纯 yaw 误差不可观测，
                 * z 轴积分无有效参考，不累积，避免 yaw 自强化漂移。 */
                g_icm_attitude.ex_int += ex * dt_s;
                g_icm_attitude.ey_int += ey * dt_s;
                g_icm_attitude.ex_int = icm_attitude_clamp(g_icm_attitude.ex_int,
                                                           -ICM_ATTITUDE_KI_INT_LIMIT,
                                                            ICM_ATTITUDE_KI_INT_LIMIT);
                g_icm_attitude.ey_int = icm_attitude_clamp(g_icm_attitude.ey_int,
                                                           -ICM_ATTITUDE_KI_INT_LIMIT,
                                                            ICM_ATTITUDE_KI_INT_LIMIT);
            }
            /* 内层门控未通过（动态运动）：冻结积分，保留已积累的修正量 */
        }

        /* Ki 积分反馈：仅修正 gx/gy（roll/pitch），gz 不加 Ki */
        gx += ICM_ATTITUDE_KI * g_icm_attitude.ex_int;
        gy += ICM_ATTITUDE_KI * g_icm_attitude.ey_int;
    }
    else
    {
        /* 加速度完全不可信（自由落体或剧烈冲击）：缓慢泄放积分
         * 0.9999^1000 ≈ 0.905（1s后保留90%），0.9999^10000 ≈ 0.368（10s后保留37%） */
        g_icm_attitude.ex_int *= ICM_ATTITUDE_KI_LEAK;
        g_icm_attitude.ey_int *= ICM_ATTITUDE_KI_LEAK;
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
    g_icm_attitude.yaw_final = g_icm_attitude.yaw_deg;
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

/* =====================================================================
 *  Yaw 调试 / 调参 API
 * ===================================================================== */

void icm_attitude_yaw_get_debug(float *gz_raw,       float *gz_bias,
                                  float *gz_comp,      float *gz_scaled,
                                  float *yaw_integral, float *yaw_final,
                                  float *yaw_corr,     float *dt_s)
{
    if (NULL != gz_raw)       *gz_raw       = g_icm_attitude.yaw_gz_raw;
    if (NULL != gz_bias)      *gz_bias      = g_icm_attitude.yaw_gz_bias;
    if (NULL != gz_comp)      *gz_comp      = g_icm_attitude.yaw_gz_comp;
    if (NULL != gz_scaled)    *gz_scaled    = g_icm_attitude.yaw_gz_scaled;
    if (NULL != yaw_integral) *yaw_integral = g_icm_attitude.yaw_integral;
    if (NULL != yaw_final)    *yaw_final    = g_icm_attitude.yaw_final;
    if (NULL != yaw_corr)     *yaw_corr     = g_icm_attitude.yaw_correction;
    if (NULL != dt_s)         *dt_s         = g_icm_attitude.yaw_dt_s;
}

void icm_attitude_yaw_set_manual_bias(float bias_dps)
{
    g_icm_attitude.yaw_manual_bias = bias_dps;
    g_icm_attitude.yaw_param_ver++;
}

float icm_attitude_yaw_get_manual_bias(void)
{
    return g_icm_attitude.yaw_manual_bias;
}

void icm_attitude_yaw_set_scale(float scale)
{
    if (scale > 0.01f)
    {
        g_icm_attitude.yaw_scale = scale;
        g_icm_attitude.yaw_param_ver++;
    }
}

float icm_attitude_yaw_get_scale(void)
{
    return g_icm_attitude.yaw_scale;
}

void icm_attitude_yaw_set_debug_enable(uint8 en)
{
    g_icm_attitude.yaw_dbg_en = en ? 1U : 0U;
}

uint8 icm_attitude_yaw_get_debug_enable(void)
{
    return g_icm_attitude.yaw_dbg_en;
}

uint32 icm_attitude_yaw_get_param_version(void)
{
    return g_icm_attitude.yaw_param_ver;
}

/* ---- Yaw Zero ---- */

void icm_attitude_yaw_zero_start(uint32 samples)
{
    g_icm_attitude.yaw_zero_sum    = 0.0f;
    g_icm_attitude.yaw_zero_n      = 0U;
    g_icm_attitude.yaw_zero_target = (samples > 0U) ? samples : 1000U;
    g_icm_attitude.yaw_zero_ok     = 0U;
    g_icm_attitude.yaw_zero_busy   = 1U;
}

void icm_attitude_yaw_zero_get_state(uint8 *busy, uint8 *ok,
                                      uint32 *n,   float *gbz)
{
    if (NULL != busy) *busy = g_icm_attitude.yaw_zero_busy;
    if (NULL != ok)   *ok   = g_icm_attitude.yaw_zero_ok;
    if (NULL != n)    *n    = g_icm_attitude.yaw_zero_n;
    if (NULL != gbz)  *gbz  = g_icm_attitude.yaw_zero_gbz;
}

/* ---- Yaw Test ---- */

void icm_attitude_yaw_test_start(void)
{
    g_icm_attitude.yaw_test_start_deg   = g_icm_attitude.yaw_deg;
    g_icm_attitude.yaw_test_last_delta  = 0.0f;
    g_icm_attitude.yaw_test_on          = 1U;
}

void icm_attitude_yaw_test_stop(float *out_start, float *out_end,
                                  float *out_delta, float *out_err90)
{
    float start_deg = g_icm_attitude.yaw_test_start_deg;
    float end_deg   = g_icm_attitude.yaw_deg;
    float delta     = end_deg - start_deg;

    /* 回绕到 [-180, 180] */
    while (delta >  180.0f) { delta -= 360.0f; }
    while (delta < -180.0f) { delta += 360.0f; }

    g_icm_attitude.yaw_test_last_delta = delta;
    g_icm_attitude.yaw_test_on         = 0U;

    if (NULL != out_start) *out_start = start_deg;
    if (NULL != out_end)   *out_end   = end_deg;
    if (NULL != out_delta) *out_delta = delta;
    if (NULL != out_err90) *out_err90 = delta - 90.0f;  /* 正值 = 转多了 */
}

uint8 icm_attitude_yaw_test_is_on(void)
{
    return g_icm_attitude.yaw_test_on;
}

float icm_attitude_yaw_test_get_start(void)
{
    return g_icm_attitude.yaw_test_start_deg;
}

float icm_attitude_yaw_test_get_last_delta(void)
{
    return g_icm_attitude.yaw_test_last_delta;
}
