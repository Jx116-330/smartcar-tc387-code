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

typedef enum
{
    LINK_VIEW_NONE = 0U,
    LINK_VIEW_DEBUG,
    LINK_VIEW_HQ_STATUS,
} link_view_mode_t;

void  menu_link_action_debug(link_view_mode_t *link_mode,
                             uint8 *menu_full_redraw,
                             void (*drain_encoder_events)(void),
                             void (*request_redraw)(uint8 full_redraw),
                             void (*reset_dynamic_region)(void));

/* HQ 状态只读查看页（LINK_VIEW_HQ_STATUS）的进入动作，
 * 和 menu_link_action_debug 同型，只是把 *link_mode 切到 HQ_STATUS。 */
void  menu_link_action_hq_status(link_view_mode_t *link_mode,
                                 uint8 *menu_full_redraw,
                                 void (*drain_encoder_events)(void),
                                 void (*request_redraw)(uint8 full_redraw),
                                 void (*reset_dynamic_region)(void));

uint8 menu_link_handle_view(link_view_mode_t *link_mode,
                            uint8 *menu_full_redraw,
                            void (*drain_encoder_events)(void),
                            void (*request_redraw)(uint8 full_redraw),
                            void (*reset_dynamic_region)(void));

#endif /* __MENU_LINK_H__ */
