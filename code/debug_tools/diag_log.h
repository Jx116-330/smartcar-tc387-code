#ifndef __DIAG_LOG_H__
#define __DIAG_LOG_H__

#include "zf_common_typedef.h"

void diag_log_init(void);
void diag_log_task(void);
void diag_log_force_dump(void);
uint8 diag_log_is_enabled(void);
void diag_log_set_enabled(uint8 enabled);

#endif
