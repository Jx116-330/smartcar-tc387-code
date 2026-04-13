/*********************************************************************************************************************
* File: menu_link.h
* Brief: TC264 板间通信 Link Debug 菜单页头文件
*        只读展示 board_comm 模块的缓存状态，不直接读 UART，不碰 ISR
* Author: JX116
*********************************************************************************************************************/

#ifndef __MENU_LINK_H__
#define __MENU_LINK_H__

#include "menu.h"
#include "MyKey.h"
#include "menu_ui_utils.h"

typedef enum
{
    LINK_VIEW_NONE = 0U,
    LINK_VIEW_DEBUG,
    LINK_VIEW_HQ_STATUS,
} link_view_mode_t;

void  menu_link_action_enter(menu_view_ctx_t *ctx, uint8 target_mode);
uint8 menu_link_handle_view(menu_view_ctx_t *ctx);

#endif /* __MENU_LINK_H__ */
