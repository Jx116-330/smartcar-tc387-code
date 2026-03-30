/***
 * @file path_types.h
 * @brief 轨迹模块共享类型定义
 * @author JX116
 * @date 2026-03-21
 * @version 2.0
 *
 * @details 定义轨迹记录、压缩存储与显示相关的公共数据类型。
 ***/

#ifndef _PATH_TYPES_H_
#define _PATH_TYPES_H_

#include "zf_common_typedef.h"
#include "path_config.h"

typedef uint8 bool_t;
#define TRUE    1
#define FALSE   0

typedef uint32 timestamp_t;
typedef float distance_t;
typedef float speed_t;
typedef float angle_t;

#if USE_COMPRESSED_STORAGE
typedef struct
{
    int16 x;
    int16 y;
    uint16 timestamp_delta;
    uint8 speed_compressed;
    uint8 flags;
} compressed_point_t;
#endif

#endif
