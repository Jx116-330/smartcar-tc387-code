/*********************************************************************************************************************
* File: menu_icm.h
* Brief: ICM42688 菜单模块头文件
* Author: JX116
*********************************************************************************************************************/

#ifndef __MENU_ICM_H__
#define __MENU_ICM_H__

#include "menu.h"
#include "MyKey.h"
#include "MyEncoder.h"

typedef enum
{
    ICM_VIEW_NONE = 0U,
    ICM_VIEW_RAW,
    ICM_VIEW_ATTITUDE,
    ICM_VIEW_GYRO_BIAS_CALIB,
    ICM_VIEW_INS_DEBUG,
    ICM_VIEW_INS_MAP,
} icm_view_mode_t;

void menu_icm_action_raw_data(icm_view_mode_t *icm_mode,
                              uint8 *menu_full_redraw,
                              void (*drain_encoder_events)(void),
                              void (*request_redraw)(uint8 full_redraw),
                              void (*reset_dynamic_region)(void));

void menu_icm_action_attitude(icm_view_mode_t *icm_mode,
                              uint8 *menu_full_redraw,
                              void (*drain_encoder_events)(void),
                              void (*request_redraw)(uint8 full_redraw),
                              void (*reset_dynamic_region)(void));

void menu_icm_action_gyro_bias_calib(icm_view_mode_t *icm_mode,
                                     uint8 *menu_full_redraw,
                                     void (*drain_encoder_events)(void),
                                     void (*request_redraw)(uint8 full_redraw),
                                     void (*reset_dynamic_region)(void));

void menu_icm_action_ins_debug(icm_view_mode_t *icm_mode,
                              uint8 *menu_full_redraw,
                              void (*drain_encoder_events)(void),
                              void (*request_redraw)(uint8 full_redraw),
                              void (*reset_dynamic_region)(void));

void menu_icm_action_ins_track_map(icm_view_mode_t *icm_mode,
                                   uint8 *menu_full_redraw,
                                   void (*drain_encoder_events)(void),
                                   void (*request_redraw)(uint8 full_redraw),
                                   void (*reset_dynamic_region)(void));

uint8 menu_icm_handle_view(icm_view_mode_t *icm_mode,
                           uint8 *menu_full_redraw,
                           void (*drain_encoder_events)(void),
                           void (*request_redraw)(uint8 full_redraw),
                           void (*reset_dynamic_region)(void));

#endif /* __MENU_ICM_H__ */
