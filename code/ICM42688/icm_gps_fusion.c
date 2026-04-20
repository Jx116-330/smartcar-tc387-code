/**
 * @file icm_gps_fusion.c
 * @brief GPS+INS 松耦合融合校正层（v0）
 * @author JX116
 * @date 2026-04-11
 *
 * 坐标约定（全链路统一 ENU）：
 *   x = East    东向
 *   y = North   北向
 *   z = Up      天向
 *
 * 策略：
 *   - 只做水平二维（位置+速度）校正，不碰高度
 *   - 不修改 roll / pitch / yaw
 *   - 不把 GPS course 硬绑到 yaw
 *   - ZUPT 成立时不做速度校正，防止 GPS 小抖动带跑
 *   - 低速时不信任 GPS direction，只用速度标量
 *   - antenna_direction_state 仅预留，不融合
 */

#include <ICM42688/icm_gps_fusion.h>
#include <ICM42688/icm_ins.h>
#include "encoder_odom.h"
#include "encoder_odom_right.h"

#include <math.h>
#include <string.h>

/* ================================================================== */
/* 可调参数                                                             */
/* ================================================================== */

/** 地球半径（米） */
#define FUSION_EARTH_RADIUS     6371000.0

/** 圆周率 */
#define FUSION_PI               3.14159265358979

/** 位置校正增益（每次 GPS 更新，把 INS 位置往 GPS 拉的比例）
 *  0.0 = 不校正，1.0 = 硬覆盖。推荐 0.02~0.10 */
#define FUSION_POS_GAIN         0.05f

/** 速度校正增益。推荐 0.02~0.08 */
#define FUSION_VEL_GAIN         0.03f

/** GPS 速度可信门限（m/s）。低于此值不信任 direction */
#define FUSION_SPEED_TRUST_MIN  1.0f

/** GPS fix 最少卫星数 */
#define FUSION_MIN_SATELLITES   4U

/** GPS 位置创新量门限（米）。超出此值视为异常跳变，不校正 */
#define FUSION_POS_INNOV_MAX    50.0f

/** GPS 速度创新量门限（m/s）。超出此值不校正 */
#define FUSION_VEL_INNOV_MAX    10.0f

/* ================================================================== */
/* 内部状态                                                             */
/* ================================================================== */

/** 原点经纬度（弧度） */
static double origin_lat_rad = 0.0;
static double origin_lon_rad = 0.0;

/** 原点是否已建立 */
static uint8 origin_ready = 0U;

/** 融合总使能 */
static uint8 fusion_enable = 1U;

/** 调试信息快照 */
static icm_gps_fusion_debug_t g_debug;

/* ================================================================== */
/* 内部工具                                                             */
/* ================================================================== */

/**
 * @brief 经纬度 → 局部 ENU 平面坐标（相对原点）
 *        x = East, y = North
 */
static void latlon_to_local_enu(double lat_deg, double lon_deg,
                                float *x_east_m, float *y_north_m)
{
    double lat_rad = lat_deg * FUSION_PI / 180.0;
    double lon_rad = lon_deg * FUSION_PI / 180.0;

    *x_east_m  = (float)(FUSION_EARTH_RADIUS * (lon_rad - origin_lon_rad) * cos(origin_lat_rad));
    *y_north_m = (float)(FUSION_EARTH_RADIUS * (lat_rad - origin_lat_rad));
}

/**
 * @brief GPS 地速+航迹角 → ENU 速度分量
 *        direction: 0=North, 90=East, 顺时针（罗盘约定）
 *        输出: vx=East, vy=North
 */
static void speed_direction_to_enu(float speed_ms, float direction_deg,
                                   float *vx_east, float *vy_north)
{
    float dir_rad = direction_deg * (float)(FUSION_PI / 180.0);
    *vx_east  = speed_ms * sinf(dir_rad);
    *vy_north = speed_ms * cosf(dir_rad);
}

/* ================================================================== */
/* 公开接口                                                             */
/* ================================================================== */

void icm_gps_fusion_init(void)
{
    origin_lat_rad = 0.0;
    origin_lon_rad = 0.0;
    origin_ready   = 0U;
    fusion_enable  = 1U;
    memset(&g_debug, 0, sizeof(g_debug));
    g_debug.fusion_enable = 1U;
}

void icm_gps_fusion_set_enable(uint8 enable)
{
    fusion_enable = enable ? 1U : 0U;
    g_debug.fusion_enable = fusion_enable;
}

void icm_gps_fusion_reset_origin(void)
{
    origin_ready = 0U;
    g_debug.origin_ready = 0U;
    g_debug.gps_update_count = 0U;
    g_debug.correction_count = 0U;
}

void icm_gps_fusion_update(double lat_deg, double lon_deg,
                           float speed_kph, float direction_deg,
                           uint8 gps_state, uint8 sat_count)
{
    float gps_x = 0.0f;
    float gps_y = 0.0f;
    float gps_speed_ms = 0.0f;
    float gps_vx = 0.0f;
    float gps_vy = 0.0f;
    float ins_px = 0.0f;
    float ins_py = 0.0f;
    float ins_vx = 0.0f;
    float ins_vy = 0.0f;
    float innov_px = 0.0f;
    float innov_py = 0.0f;
    float innov_vx = 0.0f;
    float innov_vy = 0.0f;
    float innov_pos_mag = 0.0f;
    float innov_vel_mag = 0.0f;
    uint8 fix_valid = 0U;
    uint8 speed_valid = 0U;
    uint8 heading_valid = 0U;
    uint8 do_correct = 0U;

    g_debug.correction_active = 0U;

    /* ---- 1. 检查 GPS fix 有效性 ---- */
    fix_valid = (uint8)((gps_state == 1U) &&
                        (sat_count >= FUSION_MIN_SATELLITES) &&
                        (lat_deg != 0.0) && (lon_deg != 0.0));

    g_debug.gps_valid = fix_valid;

    if (!fix_valid)
    {
        return;
    }

    g_debug.gps_update_count++;

    /* ---- 2. 首次有效 fix → 锁定原点 ---- */
    if (!origin_ready)
    {
        origin_lat_rad = lat_deg * FUSION_PI / 180.0;
        origin_lon_rad = lon_deg * FUSION_PI / 180.0;
        origin_ready   = 1U;
        g_debug.origin_ready = 1U;

        /* 同时重置 INS 位置，让 INS 和 GPS 从同一原点开始 */
        icm_ins_reset_position();
        encoder_odom_reset();
        encoder_odom_right_reset();
        return;
    }

    /* ---- 3. GPS 局部坐标 ---- */
    latlon_to_local_enu(lat_deg, lon_deg, &gps_x, &gps_y);

    g_debug.gps_x_m = gps_x;
    g_debug.gps_y_m = gps_y;

    /* ---- 4. GPS 速度 ---- */
    gps_speed_ms = speed_kph / 3.6f;

    if (gps_speed_ms >= FUSION_SPEED_TRUST_MIN)
    {
        speed_valid   = 1U;
        heading_valid = 1U;
        speed_direction_to_enu(gps_speed_ms, direction_deg, &gps_vx, &gps_vy);
    }
    else
    {
        /* 低速：速度标量仍可参考但方向不可信 */
        speed_valid   = 0U;
        heading_valid = 0U;
        gps_vx = 0.0f;
        gps_vy = 0.0f;
    }

    g_debug.gps_speed_valid   = speed_valid;
    g_debug.gps_heading_valid = heading_valid;
    g_debug.gps_vx_ms = gps_vx;
    g_debug.gps_vy_ms = gps_vy;

    /* ---- 5. 获取 INS 当前状态 ---- */
    icm_ins_get_position(&ins_px, &ins_py);
    icm_ins_get_velocity(&ins_vx, &ins_vy, NULL);

    g_debug.ins_x_m   = ins_px;
    g_debug.ins_y_m   = ins_py;
    g_debug.ins_vx_ms = ins_vx;
    g_debug.ins_vy_ms = ins_vy;

    /* ---- 6. 计算创新量 ---- */
    innov_px = gps_x - ins_px;
    innov_py = gps_y - ins_py;

    g_debug.innov_px_m  = innov_px;
    g_debug.innov_py_m  = innov_py;

    if (speed_valid)
    {
        innov_vx = gps_vx - ins_vx;
        innov_vy = gps_vy - ins_vy;
        g_debug.innov_vx_ms = innov_vx;
        g_debug.innov_vy_ms = innov_vy;
    }
    else
    {
        /* 低速时速度创新量不可信，清零避免调试页显示误导 */
        innov_vx = 0.0f;
        innov_vy = 0.0f;
        g_debug.innov_vx_ms = 0.0f;
        g_debug.innov_vy_ms = 0.0f;
    }

    /* ---- 7. 执行校正 ---- */
    if (!fusion_enable)
    {
        return;
    }

    /* ZUPT 成立时不做校正，保持速度归零 */
    if (icm_ins_is_stationary())
    {
        return;
    }

    /* 位置创新量异常检查 */
    innov_pos_mag = sqrtf(innov_px * innov_px + innov_py * innov_py);
    if (innov_pos_mag > FUSION_POS_INNOV_MAX)
    {
        return;  /* 跳变过大，可能 GPS 多径或冷启动漂移 */
    }

    do_correct = 1U;

    /* 位置校正：用小增益把 INS 往 GPS 拉 */
    icm_ins_correct_position(FUSION_POS_GAIN * innov_px,
                             FUSION_POS_GAIN * innov_py);

    /* 速度校正：仅在 GPS 速度可信时 */
    if (speed_valid)
    {
        innov_vel_mag = sqrtf(innov_vx * innov_vx + innov_vy * innov_vy);
        if (innov_vel_mag < FUSION_VEL_INNOV_MAX)
        {
            icm_ins_correct_velocity(FUSION_VEL_GAIN * innov_vx,
                                     FUSION_VEL_GAIN * innov_vy);
        }
    }

    g_debug.correction_active = do_correct;
    if (do_correct)
    {
        g_debug.correction_count++;
    }

    /* 刷新校正后的 INS 快照 */
    icm_ins_get_position(&g_debug.ins_x_m, &g_debug.ins_y_m);
    icm_ins_get_velocity(&g_debug.ins_vx_ms, &g_debug.ins_vy_ms, NULL);
}

const icm_gps_fusion_debug_t *icm_gps_fusion_get_debug(void)
{
    return &g_debug;
}

void icm_gps_fusion_get_position(float *px_m, float *py_m)
{
    /* 融合后位置 = INS 位置（已被 GPS 校正） */
    icm_ins_get_position(px_m, py_m);
}

void icm_gps_fusion_get_velocity(float *vx_ms, float *vy_ms)
{
    icm_ins_get_velocity(vx_ms, vy_ms, NULL);
}
