/*********************************************************************************************************************
* File: menu_icm.h
* Brief: ICM42688 菜单模块头文件
* Author: JX116
*********************************************************************************************************************/

#ifndef __MENU_ICM_H__
#define __MENU_ICM_H__

#include "menu.h"
#include "MyKey.h"
#include "MyEncoder.h"
#include "menu_ui_utils.h"

typedef enum
{
    ICM_VIEW_NONE = 0U,
    ICM_VIEW_RAW,
    ICM_VIEW_ATTITUDE,
    ICM_VIEW_GYRO_BIAS_CALIB,
    ICM_VIEW_INS_DEBUG,
    ICM_VIEW_INS_MAP,
} icm_view_mode_t;

void  menu_icm_action_enter(menu_view_ctx_t *ctx, uint8 target_mode);
uint8 menu_icm_handle_view(menu_view_ctx_t *ctx);

#endif /* __MENU_ICM_H__ */
