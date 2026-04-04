#ifndef __MOCK_GNSS_H__
#define __MOCK_GNSS_H__

#include "zf_common_typedef.h"

void mock_gnss_init(void);
void mock_gnss_task(void);
void mock_gnss_set_enabled(uint8 enabled);
uint8 mock_gnss_is_enabled(void);
void mock_gnss_restart(void);

#endif
