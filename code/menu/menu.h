/*********************************************************************************************************************
* File: menu.h
* Brief: 菜单系统头文件
* Author: JX116
*********************************************************************************************************************/

#ifndef _menu_h_
#define _menu_h_

#include "pid_runtime.h"
#include "zf_common_typedef.h"

typedef struct MenuPage MenuPage;

typedef struct
{
    const char *name;
    void (*function)(void);
    MenuPage *sub_page;
} MenuItem;

struct MenuPage
{
    const char *title;
    MenuItem *items;
    int num_items;
    MenuPage *parent;
};

typedef void (*menu_dynamic_draw_t)(uint16 x, uint16 y, uint16 w, uint16 h);

void menu_task(void);
void menu_init(void);
void menu_set_dynamic_draw(menu_dynamic_draw_t callback);
void menu_set_dynamic_area(uint16 x, uint16 y, uint16 w, uint16 h);
void menu_set_dynamic_clear(uint8 enable);
const pid_param_t *menu_get_pid_param(void);

#endif
