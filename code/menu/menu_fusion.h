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

typedef enum
{
    FUSION_VIEW_NONE = 0U,
    FUSION_VIEW_DEBUG,
} fusion_view_mode_t;

void menu_fusion_action_debug(fusion_view_mode_t *fusion_mode,
                              uint8 *menu_full_redraw,
                              void (*drain_encoder_events)(void),
                              void (*request_redraw)(uint8 full_redraw),
                              void (*reset_dynamic_region)(void));

uint8 menu_fusion_handle_view(fusion_view_mode_t *fusion_mode,
                              uint8 *menu_full_redraw,
                              void (*drain_encoder_events)(void),
                              void (*request_redraw)(uint8 full_redraw),
                              void (*reset_dynamic_region)(void));

#endif /* __MENU_FUSION_H__ */
