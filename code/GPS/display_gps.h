/**
 * @file display_gps.h
 * @brief GPS 坐标转换与屏幕坐标映射接口
 * @author JX116
 *
 * 这个模块专门负责两件事：
 * 1. 把经纬度转换成局部平面坐标
 * 2. 把局部平面坐标按当前显示区域缩放到屏幕坐标
 *
 * 轨迹线和当前位置都应该复用这里的转换结果，
 * 这样地图上的点和线才能处于同一坐标系。
 */

#ifndef _SHOW_GPS_H_
#define _SHOW_GPS_H_

#include "path_config.h"
#include "zf_common_headfile.h"

/** 地球半径，单位：米 */
#define EARTH_RADIUS   (6371000.0)

/** 浮点比较时使用的最小误差 */
#define EPSILON        (1e-9)

/** 圆周率 */
#define USER_PI        (3.1415926535898)

/** 当前地图模式最多参与显示变换的轨迹点数 */
#define DISPLAY_POINT_MAX  (PATH_DISPLAY_SAMPLE_POINTS)

/**
 * @brief GPS 经纬度点
 */
typedef struct
{
    double lat;    /**< 纬度 */
    double lon;    /**< 经度 */
} gps_point;

/**
 * @brief 屏幕坐标点
 */
typedef struct
{
    int16 x;       /**< 屏幕 X 坐标 */
    int16 y;       /**< 屏幕 Y 坐标 */
} screen_point;

/**
 * @brief 最近一次批量坐标变换后的屏幕点缓存
 *
 * 轨迹绘制后，当前位置和回放点可以直接复用同一套映射参数。
 */
extern screen_point screen_point_data[DISPLAY_POINT_MAX];

/**
 * @brief 设置当前坐标映射使用的显示区域
 * @param start_x 映射区域起始 X
 * @param start_y 映射区域起始 Y
 * @param width 映射区域宽度
 * @param height 映射区域高度
 */
void gps_set_display_area(int16 start_x, int16 start_y, int16 width, int16 height);

/**
 * @brief 指定本次屏幕映射要使用的外部包围盒
 *
 * 该接口通常由上层在“只采样绘制部分点、但需要按全轨迹缩放”时使用。
 */
void gps_set_xy_bounds(double min_x, double max_x, double min_y, double max_y);

/** @brief 清除外部指定的包围盒，恢复自动根据输入点计算范围 */
void gps_clear_xy_bounds(void);

/**
 * @brief 将一组 GPS 点完成从经纬度到屏幕坐标的完整转换
 * @param gps GPS 点数组
 * @param point_num 点数
 */
void user_gps_transition(gps_point *gps, int point_num);

/**
 * @brief 将 GPS 点转换为局部平面坐标
 * @param gps GPS 点数组
 * @param x 输出的 X 坐标数组
 * @param y 输出的 Y 坐标数组
 */
void gps_to_xy(gps_point *gps, double *x, double *y);

/**
 * @brief 将局部平面坐标缩放到屏幕坐标
 * @param x 局部 X 坐标数组
 * @param y 局部 Y 坐标数组
 * @param point_num 点数
 */
void xy_to_screen(double *x, double *y, int point_num);

/**
 * @brief 执行一次完整的坐标转换流程
 * @param gps GPS 点数组
 * @param point_num 点数
 */
void get_transition(gps_point *gps, int point_num);

/**
 * @brief 使用最近一次轨迹变换参数，将单个 GPS 点投影到屏幕坐标
 * @param gps 输入的经纬度点
 * @param point 输出的屏幕点
 * @return 1 表示转换成功，0 表示当前没有可复用的映射参数
 */
uint8 gps_point_to_screen(const gps_point *gps, screen_point *point);

/**
 * @brief 将缓存的屏幕点直接画到屏幕上
 *
 * 这个函数更适合调试坐标转换结果，
 * 正常轨迹绘制优先使用上层的 path_display 模块。
 */
void screen_print_gps_point(void);

#endif
