#include "menu_pid.h"

#include <stdio.h>

#include "zf_common_headfile.h"
#include "zf_device_ips200.h"
#include "menu_ui_utils.h"

static const float pid_kp_steps[] = {0.01f, 0.10f, 0.50f, 1.00f};
static const float pid_ki_steps[] = {0.001f, 0.005f, 0.010f, 0.050f};
static const float pid_kd_steps[] = {0.01f, 0.05f, 0.10f, 0.20f};
static menu_step_desc_t pid_step_descs[3] = {
    { pid_kp_steps, 4U, 1U, 0U },  /* float, 4 entries, start at index 1 */
    { pid_ki_steps, 4U, 2U, 0U },  /* float, 4 entries, start at index 2 */
    { pid_kd_steps, 4U, 1U, 0U },  /* float, 4 entries, start at index 1 */
};
static float pid_preview_target = 100.0f;
static float pid_preview_feedback = 0.0f;
static float pid_preview_output = 0.0f;

static void menu_pid_request_enter(pid_view_mode_t *mode,
                                   pid_view_mode_t next_mode,
                                   MyParams_t *params,
                                   pid_param_t *runtime_param,
                                   pid_controller_t *preview_controller,
                                   uint8 *menu_full_redraw,
                                   void (*request_redraw)(uint8 full_redraw));
static void menu_pid_reset_preview_state(pid_controller_t *preview_controller);
static void menu_pid_draw_active_view(pid_view_mode_t mode, const MyParams_t *params, pid_controller_t *preview_controller, uint8 *menu_full_redraw);
static void menu_pid_draw_edit_page(pid_view_mode_t mode, const char *title, float value, float step, uint8 *menu_full_redraw);
static void menu_pid_draw_preview_page(const MyParams_t *params, pid_controller_t *preview_controller, uint8 *menu_full_redraw);
static void menu_pid_show_line(uint16 x, uint16 y, const char *text);
static uint8 menu_pid_save_if_dirty(MyParams_t *params,
                                    uint8 *pid_param_dirty,
                                    void (*set_status_message)(const char *text, uint32 duration_ms),
                                    uint32 status_show_ms);

void menu_pid_sync_runtime_param(MyParams_t *params, pid_param_t *runtime_param, pid_controller_t *preview_controller)
{
    if ((NULL == params) || (NULL == runtime_param) || (NULL == preview_controller))
    {
        return;
    }

    runtime_param->kp = params->pid_p;
    runtime_param->ki = params->pid_i;
    runtime_param->kd = params->pid_d;
    runtime_param->integral_limit = MENU_PID_INTEGRAL_LIMIT;
    runtime_param->output_limit = MENU_PID_OUTPUT_LIMIT;
    pid_set_param(preview_controller, runtime_param);
}

static void menu_pid_reset_preview_state(pid_controller_t *preview_controller)
{
    if (NULL == preview_controller)
    {
        return;
    }

    pid_reset(preview_controller);
    pid_preview_feedback = 0.0f;
    pid_preview_output = 0.0f;
}

static void menu_pid_request_enter(pid_view_mode_t *mode,
                                   pid_view_mode_t next_mode,
                                   MyParams_t *params,
                                   pid_param_t *runtime_param,
                                   pid_controller_t *preview_controller,
                                   uint8 *menu_full_redraw,
                                   void (*request_redraw)(uint8 full_redraw))
{
    if (NULL == mode)
    {
        return;
    }

    *mode = next_mode;
    menu_pid_sync_runtime_param(params, runtime_param, preview_controller);
    menu_pid_reset_preview_state(preview_controller);
    my_key_clear_state(MY_KEY_1);
    if (NULL != menu_full_redraw)
    {
        *menu_full_redraw = 1U;
    }
    if (NULL != request_redraw)
    {
        request_redraw(1U);
    }
}

void menu_pid_action_enter(pid_view_mode_t *mode,
                           pid_view_mode_t target_mode,
                           MyParams_t *params,
                           pid_param_t *runtime_param,
                           pid_controller_t *preview_controller,
                           uint8 *menu_full_redraw,
                           void (*request_redraw)(uint8 full_redraw))
{
    menu_pid_request_enter(mode, target_mode, params, runtime_param, preview_controller, menu_full_redraw, request_redraw);
}

static uint8 menu_pid_save_if_dirty(MyParams_t *params,
                                    uint8 *pid_param_dirty,
                                    void (*set_status_message)(const char *text, uint32 duration_ms),
                                    uint32 status_show_ms)
{
    if ((NULL == pid_param_dirty) || !(*pid_param_dirty))
    {
        return 1U;
    }

    if (menu_params_save_to_flash(params))
    {
        *pid_param_dirty = 0U;
        if (NULL != set_status_message)
        {
            set_status_message("PID saved", status_show_ms);
        }
        return 1U;
    }

    if (NULL != set_status_message)
    {
        set_status_message("Param flash full", status_show_ms);
    }
    return 0U;
}

void menu_pid_action_reset(MyParams_t *params,
                           pid_view_mode_t *mode,
                           pid_param_t *runtime_param,
                           pid_controller_t *preview_controller,
                           uint8 *pid_param_dirty,
                           uint8 *menu_full_redraw,
                           void (*set_status_message)(const char *text, uint32 duration_ms),
                           uint32 status_show_ms)
{
    if (NULL == params)
    {
        return;
    }

    params->pid_p = 1.2f;
    params->pid_i = 0.01f;
    params->pid_d = 0.5f;
    if (NULL != pid_param_dirty)
    {
        *pid_param_dirty = 1U;
    }
    menu_pid_sync_runtime_param(params, runtime_param, preview_controller);
    menu_pid_reset_preview_state(preview_controller);
    (void)menu_pid_save_if_dirty(params, pid_param_dirty, set_status_message, status_show_ms);
    if (NULL != menu_full_redraw)
    {
        *menu_full_redraw = 1U;
    }
    if (NULL != mode)
    {
        *mode = PID_VIEW_NONE;
    }
}

static void menu_pid_show_line(uint16 x, uint16 y, const char *text)
{
    ips200_show_string(x, y, (char *)text);
}

static void menu_pid_draw_edit_page(pid_view_mode_t mode, const char *title, float value, float step, uint8 *menu_full_redraw)
{
    char value_line[32];
    char step_line[32];
    static pid_view_mode_t last_draw_mode = PID_VIEW_NONE;
    static float last_draw_value = -1.0f;
    static float last_draw_step = -1.0f;

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) &&
        last_draw_mode == mode &&
        last_draw_value == value &&
        last_draw_step == step)
    {
        return;
    }

    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        menu_pid_show_line(10, 10, title);
        ips200_draw_line(0, 30, ips200_width_max - 1, 30, RGB565_GRAY);
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(10, 40, (uint16)(ips200_width_max - 20U), "ENC: Adjust parameter");
        menu_ui_show_pad(10, 56, (uint16)(ips200_width_max - 20U), "K1: Change step   LONG: Exit");
        menu_ui_show_pad(18, 82, (uint16)(ips200_width_max - 36U), "Current Value");
        menu_ui_show_pad(18, 138, (uint16)(ips200_width_max - 36U), "Step Size");
        menu_ui_show_pad(18, 194, (uint16)(ips200_width_max - 36U), "Rotate encoder to tune");
        menu_ui_show_pad(18, 210, (uint16)(ips200_width_max - 36U), "Exit auto-saves PID");
    }

    snprintf(value_line, sizeof(value_line), "%.4f", value);
    snprintf(step_line, sizeof(step_line), "%.4f", step);

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    menu_ui_show_pad(18, 104, (uint16)(ips200_width_max - 36U), value_line);

    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    menu_ui_show_pad(18, 160, (uint16)(ips200_width_max - 36U), step_line);

    last_draw_mode = mode;
    last_draw_value = value;
    last_draw_step = step;
    if (NULL != menu_full_redraw)
    {
        *menu_full_redraw = 0U;
    }
}

static void menu_pid_draw_preview_page(const MyParams_t *params, pid_controller_t *preview_controller, uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char val[16];
    uint32 now_ms = system_getval_ms();
    float error;

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) && ((now_ms - last_refresh_ms) < 120U))
    {
        return;
    }

    last_refresh_ms = now_ms;

    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        menu_pid_show_line(5, 10, "PID Preview");
        ips200_draw_line(0, 30, ips200_width_max - 1, 30, RGB565_GRAY);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        menu_ui_show_pad(5, (uint16)(ips200_height_max - 20U), (uint16)(ips200_width_max - 10U), "K1 LONG Exit");

        /* Static labels -- drawn once */
        ips200_show_string(10, 56, "Kp:         Ki:");
        ips200_show_string(10, 80, "Kd:         Target:");
        ips200_show_string(10, 104, "Feedback:");
        ips200_show_string(10, 128, "Error:      Output:");
        ips200_show_string(10, 152, "Adjust gains in PID menu");
    }

    /* PID simulation step */
    pid_preview_output = pid_calculate(preview_controller, pid_preview_target, pid_preview_feedback);
    pid_preview_feedback += pid_preview_output * 0.02f;
    pid_preview_feedback *= 0.995f;
    error = pid_preview_target - pid_preview_feedback;

    /* Per-frame: update value slots only */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    /* y=56: Kp value at x=34 (3*8+10), Ki value at x=130 (15*8+10) */
    snprintf(val, sizeof(val), "%.3f", params->pid_p);
    menu_ui_show_pad(34, 56, 88, val);   /* 11 chars * 8 = 88 */
    snprintf(val, sizeof(val), "%.3f", params->pid_i);
    menu_ui_show_pad(130, 56, 80, val);  /* 10 chars * 8 = 80 */

    /* y=80: Kd value at x=34, Target value at x=162 (19*8+10) */
    snprintf(val, sizeof(val), "%.3f", params->pid_d);
    menu_ui_show_pad(34, 80, 120, val);  /* 15 chars * 8 = 120 */
    snprintf(val, sizeof(val), "%.1f", pid_preview_target);
    menu_ui_show_pad(162, 80, 56, val);  /* 7 chars * 8 = 56 */

    /* y=104: Feedback value at x=82 (9*8+10) */
    snprintf(val, sizeof(val), "%.2f", pid_preview_feedback);
    menu_ui_show_pad(82, 104, 128, val); /* 16 chars * 8 = 128 */

    /* y=128: Error value at x=58 (6*8+10), Output value at x=178 (21*8+10) */
    snprintf(val, sizeof(val), "%.2f", error);
    menu_ui_show_pad(58, 128, 112, val); /* 14 chars * 8 = 112 */
    snprintf(val, sizeof(val), "%.2f", pid_preview_output);
    menu_ui_show_pad(178, 128, 56, val); /* 7 chars * 8 = 56 */

    /* y=152: fully static, no per-frame draw */

    if (NULL != menu_full_redraw)
    {
        *menu_full_redraw = 0U;
    }
}

static void menu_pid_draw_active_view(pid_view_mode_t mode, const MyParams_t *params, pid_controller_t *preview_controller, uint8 *menu_full_redraw)
{
    if (PID_VIEW_EDIT_KP == mode)
    {
        menu_pid_draw_edit_page(mode, "Edit Kp", params->pid_p, menu_step_get_float(&pid_step_descs[mode - PID_VIEW_EDIT_KP]), menu_full_redraw);
    }
    else if (PID_VIEW_EDIT_KI == mode)
    {
        menu_pid_draw_edit_page(mode, "Edit Ki", params->pid_i, menu_step_get_float(&pid_step_descs[mode - PID_VIEW_EDIT_KP]), menu_full_redraw);
    }
    else if (PID_VIEW_EDIT_KD == mode)
    {
        menu_pid_draw_edit_page(mode, "Edit Kd", params->pid_d, menu_step_get_float(&pid_step_descs[mode - PID_VIEW_EDIT_KP]), menu_full_redraw);
    }
    else if (PID_VIEW_PREVIEW == mode)
    {
        menu_pid_draw_preview_page(params, preview_controller, menu_full_redraw);
    }
}

uint8 menu_pid_handle_view(pid_view_mode_t *mode,
                           MyParams_t *params,
                           pid_param_t *runtime_param,
                           pid_controller_t *preview_controller,
                           uint8 *pid_param_dirty,
                           uint8 *menu_full_redraw,
                           void (*request_redraw)(uint8 full_redraw),
                           void (*set_status_message)(const char *text, uint32 duration_ms),
                           uint32 status_show_ms)
{
    float step;
    uint8 updated = 0U;

    if ((NULL == mode) || (PID_VIEW_NONE == *mode))
    {
        return 0U;
    }

    if (MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1))
    {
        my_key_clear_state(MY_KEY_1);
        if ((*mode <= PID_VIEW_EDIT_KD) && (NULL != pid_param_dirty) && *pid_param_dirty)
        {
            (void)menu_pid_save_if_dirty(params, pid_param_dirty, set_status_message, status_show_ms);
        }
        *mode = PID_VIEW_NONE;
        if (NULL != request_redraw)
        {
            request_redraw(1U);
        }
        return 1U;
    }

    /* 旋钮调节 PID 参数：必须主动调 If_Switch_Encoder_Change() 把 pending
     * 转成 change_num（View 激活时 menu_update_selection_from_encoder 不运行）。
     * while 循环抽干队列，避免快转后参数还在继续跳的滞后感。 */
    if (*mode <= PID_VIEW_EDIT_KD)
    {
        step = menu_step_get_float(&pid_step_descs[*mode - PID_VIEW_EDIT_KP]);
        while (If_Switch_Encoder_Change())
        {
            uint8 is_add = (switch_encoder_change_num < 0) ? 1U : 0U;
            if (PID_VIEW_EDIT_KP == *mode)
            {
                menu_param_adjust_float(&params->pid_p, step, PARAM_PID_P_MIN, PARAM_PID_P_MAX, is_add);
            }
            else if (PID_VIEW_EDIT_KI == *mode)
            {
                menu_param_adjust_float(&params->pid_i, step, PARAM_PID_I_MIN, PARAM_PID_I_MAX, is_add);
            }
            else if (PID_VIEW_EDIT_KD == *mode)
            {
                menu_param_adjust_float(&params->pid_d, step, PARAM_PID_D_MIN, PARAM_PID_D_MAX, is_add);
            }
            updated = 1U;
        }

        if (updated)
        {
            if (NULL != pid_param_dirty)
            {
                *pid_param_dirty = 1U;
            }
            menu_pid_sync_runtime_param(params, runtime_param, preview_controller);
        }
    }

    if (MY_KEY_SHORT_PRESS == my_key_get_state(MY_KEY_1))
    {
        my_key_clear_state(MY_KEY_1);
        if (*mode <= PID_VIEW_EDIT_KD)
        {
            menu_step_cycle(&pid_step_descs[*mode - PID_VIEW_EDIT_KP]);
            if (NULL != menu_full_redraw)
            {
                *menu_full_redraw = 1U;
            }
        }
    }

    menu_pid_draw_active_view(*mode, params, preview_controller, menu_full_redraw);
    return 1U;
}
