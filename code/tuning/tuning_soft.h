#ifndef __TUNING_SOFT_H__
#define __TUNING_SOFT_H__

#include "menu.h"
#include "zf_common_typedef.h"

extern MenuPage tuning_menu;

void tuning_soft_init(void);
void tuning_soft_task(void);
uint8 tuning_soft_handle_view(void);
uint8 tuning_soft_is_active(void);

#endif
