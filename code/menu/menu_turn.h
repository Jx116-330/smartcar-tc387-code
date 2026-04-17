#ifndef __MENU_TURN_H__
#define __MENU_TURN_H__

#include "menu.h"
#include "Turn.h"
#include "zf_common_typedef.h"

extern MenuPage turn_menu;

void menu_turn_sync_labels(void);
uint8 menu_turn_handle_view(void);
uint8 menu_turn_is_active(void);

#endif /* __MENU_TURN_H__ */
