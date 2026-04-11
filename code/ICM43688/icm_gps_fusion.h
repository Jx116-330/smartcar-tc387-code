/**
 * @file icm_gps_fusion.h
 * @brief GPS+INS 松耦合融合校正层（v0）
 * @author JX116
 * @date 2026-04-11
 *
 * 坐标约定（ENU）：x=East, y=North, z=Up
 *
 * 功能：
 *   - 以 GNSS 首次有效定位为原点，建立局部平面坐标
 *   - 在 GNSS 更新到来时，计算 GPS-INS 位置/速度创新量
 *   - 用小增益缓慢校正 icm_ins 的水平位置和速度
 *   - ZUPT 成立时跳过校正，保持速度归零
 *   - 不修改 roll/pitch/yaw
 *
 * 使用方式：
 *   1. 开机调用 icm_gps_fusion_init()
 *   2. 在主循环 gnss_flag 到来后调用 icm_gps_fusion_update()
 *   3. 融合后位置/速度可通过 getter 获取
 */

#ifndef __ICM_GPS_FUSION_H__
#define __ICM_GPS_FUSION_H__

#include "zf_common_typedef.h"

/* ---------- 融合状态调试结构体 ---------- */
typedef struct
{
    /* GPS 局部坐标 */
    float gps_x_m;
    float gps_y_m;

    /* INS 当前位置（校正后快照） */
    float ins_x_m;
    float ins_y_m;

    /* GPS 速度分量 */
    float gps_vx_ms;
    float gps_vy_ms;

    /* INS 速度 */
    float ins_vx_ms;
    float ins_vy_ms;

    /* 位置创新量（GPS - INS） */
    float innov_px_m;
    float innov_py_m;

    /* 速度创新量 */
    float innov_vx_ms;
    float innov_vy_ms;

    /* 有效性标志 */
    uint8 origin_ready;        /* 原点已建立 */
    uint8 gps_valid;           /* 当前 GNSS fix 有效 */
    uint8 gps_speed_valid;     /* GPS 速度可信（>门限且方向有效） */
    uint8 gps_heading_valid;   /* GPS 航向可信（速度>门限） */
    uint8 correction_active;   /* 本次实际执行了校正 */
    uint8 fusion_enable;       /* 融合总使能 */

    /* 统计 */
    uint32 gps_update_count;   /* 已处理的 GNSS 更新次数 */
    uint32 correction_count;   /* 实际执行校正的次数 */
} icm_gps_fusion_debug_t;

/* ---------- 公开接口 ---------- */

/**
 * @brief 初始化融合模块（复位所有状态）
 */
void icm_gps_fusion_init(void);

/**
 * @brief 收到新 GNSS 数据后调用（主循环，非 ISR）
 *
 * 内部流程：
 *   1. 检查 GNSS fix 有效性
 *   2. 首次有效定位时锁定原点
 *   3. 计算 GPS 局部坐标和速度
 *   4. 计算创新量并执行校正
 *
 * @param lat_deg  纬度（十进制度）
 * @param lon_deg  经度（十进制度）
 * @param speed_kph  GPS 地速（km/h）
 * @param direction_deg  GPS 航迹角（0=North, 顺时针, 度）
 * @param gps_state  GNSS fix 状态（1=有效）
 * @param sat_count  使用卫星数
 */
void icm_gps_fusion_update(double lat_deg, double lon_deg,
                           float speed_kph, float direction_deg,
                           uint8 gps_state, uint8 sat_count);

/**
 * @brief 获取融合后调试信息（只读快照）
 */
const icm_gps_fusion_debug_t *icm_gps_fusion_get_debug(void);

/**
 * @brief 获取融合后位置（= INS 位置，已被 GPS 校正）
 */
void icm_gps_fusion_get_position(float *px_m, float *py_m);

/**
 * @brief 获取融合后速度（= INS 速度，已被 GPS 校正）
 */
void icm_gps_fusion_get_velocity(float *vx_ms, float *vy_ms);

/**
 * @brief 融合总使能开关（默认开启）
 */
void icm_gps_fusion_set_enable(uint8 enable);

/**
 * @brief 手动重置原点（下次有效 GPS 重新锁定）
 */
void icm_gps_fusion_reset_origin(void);

#endif /* __ICM_GPS_FUSION_H__ */
