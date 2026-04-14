/*********************************************************************************************************************
* File: menu_pedal.h
* Brief: 踏板调试菜单头文件
* Author: JX116
*********************************************************************************************************************/

#ifndef __MENU_PEDAL_H__
#define __MENU_PEDAL_H__

#include "menu.h"
#include "MyKey.h"
#include "menu_ui_utils.h"

typedef enum
{
    PEDAL_VIEW_NONE = 0U,
    PEDAL_VIEW_DEBUG,
    PEDAL_VIEW_DRIVE_CTRL,   /* 驱动控制页：开关 + PWM上限 + 实时出力 */
} pedal_view_mode_t;

void  menu_pedal_action_enter(menu_view_ctx_t *ctx, uint8 target_mode);
uint8 menu_pedal_handle_view(menu_view_ctx_t *ctx);

#endif /* __MENU_PEDAL_H__ */
