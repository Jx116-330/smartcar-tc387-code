#ifndef __MENU_PID_H__
#define __MENU_PID_H__

#include "menu.h"
#include "menu_params.h"
#include "MyKey.h"
#include "MyEncoder.h"

#define MENU_PID_INTEGRAL_LIMIT 500.0f
#define MENU_PID_OUTPUT_LIMIT   1000.0f

typedef enum
{
    PID_VIEW_NONE = 0U,
    PID_VIEW_EDIT_KP,
    PID_VIEW_EDIT_KI,
    PID_VIEW_EDIT_KD,
    PID_VIEW_PREVIEW,
} pid_view_mode_t;

void menu_pid_sync_runtime_param(MyParams_t *params, pid_param_t *runtime_param, pid_controller_t *preview_controller);
uint8 menu_pid_handle_view(pid_view_mode_t *mode,
                           MyParams_t *params,
                           pid_param_t *runtime_param,
                           pid_controller_t *preview_controller,
                           uint8 *pid_param_dirty,
                           uint8 *menu_full_redraw,
                           void (*request_redraw)(uint8 full_redraw),
                           void (*set_status_message)(const char *text, uint32 duration_ms),
                           uint32 status_show_ms);
void menu_pid_action_edit_kp(pid_view_mode_t *mode,
                             MyParams_t *params,
                             pid_param_t *runtime_param,
                             pid_controller_t *preview_controller,
                             uint8 *menu_full_redraw,
                             void (*request_redraw)(uint8 full_redraw));
void menu_pid_action_edit_ki(pid_view_mode_t *mode,
                             MyParams_t *params,
                             pid_param_t *runtime_param,
                             pid_controller_t *preview_controller,
                             uint8 *menu_full_redraw,
                             void (*request_redraw)(uint8 full_redraw));
void menu_pid_action_edit_kd(pid_view_mode_t *mode,
                             MyParams_t *params,
                             pid_param_t *runtime_param,
                             pid_controller_t *preview_controller,
                             uint8 *menu_full_redraw,
                             void (*request_redraw)(uint8 full_redraw));
void menu_pid_action_preview(pid_view_mode_t *mode,
                             MyParams_t *params,
                             pid_param_t *runtime_param,
                             pid_controller_t *preview_controller,
                             uint8 *menu_full_redraw,
                             void (*request_redraw)(uint8 full_redraw));
void menu_pid_action_reset(MyParams_t *params,
                           pid_view_mode_t *mode,
                           pid_param_t *runtime_param,
                           pid_controller_t *preview_controller,
                           uint8 *pid_param_dirty,
                           uint8 *menu_full_redraw,
                           void (*set_status_message)(const char *text, uint32 duration_ms),
                           uint32 status_show_ms);

#endif
