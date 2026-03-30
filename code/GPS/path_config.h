/**
 * @file path_config.h
 * @brief 轨迹记录模块配置文件
 * @author JX116
 * @date 2026-03-21
 * @version 2.0
 *
 * @details 集中管理轨迹记录相关的配置与调参项。
 **/

#ifndef _PATH_CONFIG_H_
#define _PATH_CONFIG_H_

// -----------------------------------------------------------------------------
// 轨迹记录参数
// -----------------------------------------------------------------------------

/** @brief 最大轨迹点数量 */
#define MAX_PATH_POINTS             300
/** @brief 最小记录距离，单位米 */
#define MIN_RECORD_DISTANCE         0.2f
/** @brief 最小记录时间间隔，单位毫秒 */
#define MIN_RECORD_INTERVAL_MS      50
/** @brief GPS最小有效卫星数 */
#define GPS_MIN_SATELLITES          4
/** @brief 最大记录速度，单位km/h */
#define MAX_RECORD_SPEED_KPH        50.0f
/** @brief 地图显示时用于绘制的最大采样点数 */
#define PATH_DISPLAY_SAMPLE_POINTS  50

#endif
