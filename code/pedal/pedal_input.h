/*********************************************************************************************************************
* File: pedal_input.h
* Brief: 踏板 ADC 采样模块（第一步：A45/A47 原始值读取）
* Author: JX116
*********************************************************************************************************************/

#ifndef __PEDAL_INPUT_H__
#define __PEDAL_INPUT_H__

#include "zf_common_typedef.h"

void   pedal_input_init(void);
void   pedal_input_task(void);
uint16 pedal_input_get_a45(void);
uint16 pedal_input_get_a47(void);

#endif /* __PEDAL_INPUT_H__ */
