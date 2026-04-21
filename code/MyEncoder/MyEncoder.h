#ifndef __MyEncoder_H
#define __MyEncoder_H

#include "myhead.h"

#define SWITCH_ENCODER_A_PIN     P20_3
#define SWITCH_ENCODER_B_PIN     P20_0

/* 共享于 5kHz ISR 与主循环之间，必须 volatile 防止编译器优化掉读写 */
extern volatile int   switch_encoder_num;
extern volatile int   switch_encoder_change_num;
extern volatile uint8 switch_encode_bring_flag;
extern volatile uint8 switch_encode_change_get_buff_flag;

void MyEncoder_Init(void);
void Get_Switch_Num(void);              /* 由 CCU61_CH1 (5kHz) ISR 调用 */
uint8 If_Switch_Encoder_Change(void);   /* 由 menu_task 主循环调用 */

#endif
