#ifndef __MyEncoder_H
#define __MyEncoder_H

#include "myhead.h"

#define SWITCH_ENCODER_A_PIN     P20_3
#define SWITCH_ENCODER_B_PIN     P20_0

extern int switch_encoder_num;
extern int switch_encoder_change_num;
extern uint8 switch_encode_bring_flag;
extern uint8 switch_encode_change_get_buff_flag;

void MyEncoder_Init(void);
void Get_Switch_Num(void);
uint8 If_Switch_Encoder_Change(void);

#endif
