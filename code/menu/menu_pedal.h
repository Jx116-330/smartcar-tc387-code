/*********************************************************************************************************************
* File: menu_pedal.h
* Brief: 踏板调试菜单头文件
* Author: JX116
*********************************************************************************************************************/

#ifndef __MENU_PEDAL_H__
#define __MENU_PEDAL_H__

#include "menu.h"
#include "MyKey.h"

typedef enum
{
    PEDAL_VIEW_NONE = 0U,
    PEDAL_VIEW_DEBUG,
} pedal_view_mode_t;

void menu_pedal_action_debug(pedal_view_mode_t *pedal_mode,
                             uint8 *menu_full_redraw,
                             void (*drain_encoder_events)(void),
                             void (*request_redraw)(uint8 full_redraw),
                             void (*reset_dynamic_region)(void));

uint8 menu_pedal_handle_view(pedal_view_mode_t *pedal_mode,
                             uint8 *menu_full_redraw,
                             void (*drain_encoder_events)(void),
                             void (*request_redraw)(uint8 full_redraw),
                             void (*reset_dynamic_region)(void));

#endif /* __MENU_PEDAL_H__ */
