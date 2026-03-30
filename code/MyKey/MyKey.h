#ifndef CODE_MYKEY_MYKEY_H_
#define CODE_MYKEY_MYKEY_H_

#include "zf_driver_gpio.h"
#include "myhead.h"

#define MY_KEY_LIST                    {P20_2, P20_8, P20_6, P20_7}

#define MY_KEY_RELEASE_LEVEL           (GPIO_HIGH)
#define MY_KEY_MAX_SHOCK_PERIOD        (20)
#define MY_KEY_LONG_PRESS_PERIOD       (250)

typedef enum
{
    MY_KEY_1,
    MY_KEY_2,
    MY_KEY_3,
    MY_KEY_4,
    MY_KEY_NUMBER,
} my_key_index_enum;

typedef enum
{
    MY_KEY_RELEASE,
    MY_KEY_SHORT_PRESS,
    MY_KEY_LONG_PRESS,
} my_key_state_enum;

extern uint8 key_long_press_flag[];

void my_key_scanner(void);
my_key_state_enum my_key_get_state(my_key_index_enum key_n);
void my_key_clear_state(my_key_index_enum key_n);
void my_key_clear_all_state(void);
void my_key_init(uint32 period);
uint8 My_Key_IfEnter(void);

#endif
