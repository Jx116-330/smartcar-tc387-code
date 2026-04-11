#ifndef __MENU_PARAMS_H__
#define __MENU_PARAMS_H__

#include "zf_common_typedef.h"
#include "path_config.h"
#include "path_recorder.h"

/* 参数结构体：用于保存 PID 和菜单相关配置 */
typedef struct
{
    float pid_p;
    float pid_i;
    float pid_d;
    float record_min_distance;
    uint32 record_min_interval_ms;
    float record_max_speed_kph;
    uint8 record_min_satellites;
    uint8 threshold_mode;
    uint8 threshold_val;
    uint32_t magic_code;
} MyParams_t;

#define PARAM_PID_P_MIN             0.0f
#define PARAM_PID_P_MAX             50.0f
#define PARAM_PID_I_MIN             0.0f
#define PARAM_PID_I_MAX             10.0f
#define PARAM_PID_D_MIN             0.0f
#define PARAM_PID_D_MAX             20.0f
#define PARAM_THRESHOLD_MODE_MAX    3U
#define PARAM_RECORD_DISTANCE_MIN   0.05f
#define PARAM_RECORD_DISTANCE_MAX   5.0f
#define PARAM_RECORD_INTERVAL_MIN   10U
#define PARAM_RECORD_INTERVAL_MAX   1000U
#define PARAM_RECORD_SPEED_MIN      5.0f
#define PARAM_RECORD_SPEED_MAX      120.0f
#define PARAM_RECORD_SAT_MIN        3U
#define PARAM_RECORD_SAT_MAX        12U

uint8 menu_params_are_valid(const MyParams_t *params);
uint8 menu_params_load_from_flash(MyParams_t *params);
uint8 menu_params_save_to_flash(const MyParams_t *params);
void menu_params_apply_record_config(const MyParams_t *params);
void menu_params_capture_record_config(MyParams_t *params);
void menu_params_set_default(MyParams_t *params);
uint8 menu_params_load_or_default(MyParams_t *params);

#endif
