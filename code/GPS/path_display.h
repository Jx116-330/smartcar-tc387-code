/**
 * @file path_display.h
 * @brief GPS 轨迹地图显示接口
 * @author JX116
 *
 * 当前模块只负责两件事：
 * 1. 在指定区域内绘制已记录的轨迹
 * 2. 在同一坐标系中叠加当前 GPS 位置
 */

#ifndef _PATH_DISPLAY_H_
#define _PATH_DISPLAY_H_

#include "zf_common_headfile.h"

void path_display_init(void);
void path_display_set_area(uint16 x, uint16 y, uint16 width, uint16 height);
void path_display_draw_map(uint16 color);

#endif
