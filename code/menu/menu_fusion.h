/*********************************************************************************************************************
* File: menu_fusion.h
* Brief: GPS+INS 融合菜单模块头文件
* Author: JX116
*********************************************************************************************************************/

#ifndef __MENU_FUSION_H__
#define __MENU_FUSION_H__

#include "menu.h"
#include "MyKey.h"
#include "MyEncoder.h"
#include "menu_ui_utils.h"

typedef enum
{
    FUSION_VIEW_NONE = 0U,
    FUSION_VIEW_DEBUG,
} fusion_view_mode_t;

void  menu_fusion_action_enter(menu_view_ctx_t *ctx, uint8 target_mode);
uint8 menu_fusion_handle_view(menu_view_ctx_t *ctx);

#endif /* __MENU_FUSION_H__ */
