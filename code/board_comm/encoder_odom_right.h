/*********************************************************************************************************************
 * File: encoder_odom_right.h
 * Brief: Right rear wheel odometry to INS fusion module.
 *        Runs in parallel with the left rear ENCL fusion path, using local GPIO-polled right rear data.
 *********************************************************************************************************************/

#ifndef __ENCODER_ODOM_RIGHT_H__
#define __ENCODER_ODOM_RIGHT_H__

#include "zf_common_typedef.h"

void  encoder_odom_right_init(void);
void  encoder_odom_right_task(void);

void  encoder_odom_right_reset(void);
void  encoder_odom_right_set_enable(uint8 en);
uint8 encoder_odom_right_is_enabled(void);

float encoder_odom_right_get_px_m(void);
float encoder_odom_right_get_py_m(void);
float encoder_odom_right_get_speed_ms(void);
uint8 encoder_odom_right_is_active(void);

#endif /* __ENCODER_ODOM_RIGHT_H__ */
