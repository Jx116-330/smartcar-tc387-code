/*********************************************************************************************************************
* File: wifi_menu.h
* Brief: WiFi 菜单模块头文件
*        提供 Net Info / Send Test / Config / Reconnect 四项功能的菜单页面
*********************************************************************************************************************/

#ifndef _wifi_menu_h_
#define _wifi_menu_h_

#include "zf_common_typedef.h"
#include "menu.h"

/* 对外暴露的菜单页面，供 menu.c 的 main_items[] 直接引用 */
extern MenuPage wifi_page;

/* 由 cpu0_main.c 在 TCP 连接成功/失败后调用，更新界面显示状态 */
void wifi_menu_set_tcp_status(uint8 connected);

/* 返回当前是否处于 WiFi 视图（供 menu.c 的 menu_task 判断是否拦截输入）*/
uint8 wifi_menu_is_active(void);

/* 在 menu_task() 中调用，处理 WiFi 视图的按键与刷新逻辑
 * 返回 1 表示视图已接管当前帧，menu_task 应直接 return  */
uint8 wifi_menu_handle_view(void);

#endif
