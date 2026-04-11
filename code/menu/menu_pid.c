#include "menu_pid.h"

#include <stdio.h>

#include "zf_common_headfile.h"
#include "zf_device_ips200.h"

static const float pid_kp_steps[] = {0.01f, 0.10f, 0.50f, 1.00f};
static const float pid_ki_steps[] = {0.001f, 0.005f, 0.010f, 0.050f};
static const float pid_kd_steps[] = {0.01f, 0.05f, 0.10f, 0.20f};
static uint8 pid_kp_step_index = 1U;
static uint8 pid_ki_step_index = 2U;
static uint8 pid_kd_step_index = 1U;
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
static float menu_pid_get_current_step(uint8 mode);
static void menu_pid_cycle_step(uint8 mode);
static void menu_pid_adjust_param(float *ptr, float step, float min, float max, uint8 is_add);
static void menu_pid_draw_active_view(pid_view_mode_t mode, const MyParams_t *params, pid_controller_t *preview_controller, uint8 *menu_full_redraw);
static void menu_pid_draw_edit_page(pid_view_mode_t mode, const char *title, float value, float step, uint8 *menu_full_redraw);
static void menu_pid_draw_preview_page(const MyParams_t *params, pid_controller_t *preview_controller, uint8 *menu_full_redraw);
static void menu_pid_show_line(uint16 x, uint16 y, const char *text);
static void menu_pid_show_line_pad(uint16 x, uint16 y, const char *text, uint16 width);
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

void menu_pid_action_edit_kp(pid_view_mode_t *mode,
                             MyParams_t *params,
                             pid_param_t *runtime_param,
                             pid_controller_t *preview_controller,
                             uint8 *menu_full_redraw,
                             void (*request_redraw)(uint8 full_redraw))
{
    menu_pid_request_enter(mode, PID_VIEW_EDIT_KP, params, runtime_param, preview_controller, menu_full_redraw, request_redraw);
}

void menu_pid_action_edit_ki(pid_view_mode_t *mode,
                             MyParams_t *params,
                             pid_param_t *runtime_param,
                             pid_controller_t *preview_controller,
                             uint8 *menu_full_redraw,
                             void (*request_redraw)(uint8 full_redraw))
{
    menu_pid_request_enter(mode, PID_VIEW_EDIT_KI, params, runtime_param, preview_controller, menu_full_redraw, request_redraw);
}

void menu_pid_action_edit_kd(pid_view_mode_t *mode,
                             MyParams_t *params,
                             pid_param_t *runtime_param,
                             pid_controller_t *preview_controller,
                             uint8 *menu_full_redraw,
                             void (*request_redraw)(uint8 full_redraw))
{
    menu_pid_request_enter(mode, PID_VIEW_EDIT_KD, params, runtime_param, preview_controller, menu_full_redraw, request_redraw);
}

void menu_pid_action_preview(pid_view_mode_t *mode,
                             MyParams_t *params,
                             pid_param_t *runtime_param,
                             pid_controller_t *preview_controller,
                             uint8 *menu_full_redraw,
                             void (*request_redraw)(uint8 full_redraw))
{
    menu_pid_request_enter(mode, PID_VIEW_PREVIEW, params, runtime_param, preview_controller, menu_full_redraw, request_redraw);
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

static void menu_pid_adjust_param(float *ptr, float step, float min, float max, uint8 is_add)
{
    if (NULL == ptr)
    {
        return;
    }

    if (is_add)
    {
        *ptr = (*ptr + step <= max) ? (*ptr + step) : max;
    }
    else
    {
        *ptr = (*ptr - step >= min) ? (*ptr - step) : min;
    }
}

static float menu_pid_get_current_step(uint8 mode)
{
    if (1U == mode)
    {
        return pid_kp_steps[pid_kp_step_index];
    }
    else if (2U == mode)
    {
        return pid_ki_steps[pid_ki_step_index];
    }
    else
    {
        return pid_kd_steps[pid_kd_step_index];
    }
}

static void menu_pid_cycle_step(uint8 mode)
{
    if (1U == mode)
    {
        pid_kp_step_index = (uint8)((pid_kp_step_index + 1U) % (sizeof(pid_kp_steps) / sizeof(pid_kp_steps[0])));
    }
    else if (2U == mode)
    {
        pid_ki_step_index = (uint8)((pid_ki_step_index + 1U) % (sizeof(pid_ki_steps) / sizeof(pid_ki_steps[0])));
    }
    else if (3U == mode)
    {
        pid_kd_step_index = (uint8)((pid_kd_step_index + 1U) % (sizeof(pid_kd_steps) / sizeof(pid_kd_steps[0])));
    }
}

static void menu_pid_show_line(uint16 x, uint16 y, const char *text)
{
    ips200_show_string(x, y, (char *)text);
}

static void menu_pid_show_line_pad(uint16 x, uint16 y, const char *text, uint16 width)
{
    char line[40];
    uint16 max_chars = (uint16)(width / 8U);
    uint16 i = 0U;

    if (max_chars > (sizeof(line) - 1U))
    {
        max_chars = (uint16)(sizeof(line) - 1U);
    }

    while ((i < max_chars) && (NULL != text) && (text[i] != '\0'))
    {
        line[i] = text[i];
        i++;
    }
    while (i < max_chars)
    {
        line[i++] = ' ';
    }
    line[i] = '\0';
    ips200_show_string(x, y, line);
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
        menu_pid_show_line_pad(10, 40, "ENC: Adjust parameter", (uint16)(ips200_width_max - 20U));
        menu_pid_show_line_pad(10, 56, "K1: Change step   LONG: Exit", (uint16)(ips200_width_max - 20U));
        menu_pid_show_line_pad(18, 82, "Current Value", (uint16)(ips200_width_max - 36U));
        menu_pid_show_line_pad(18, 138, "Step Size", (uint16)(ips200_width_max - 36U));
        menu_pid_show_line_pad(18, 194, "Rotate encoder to tune", (uint16)(ips200_width_max - 36U));
        menu_pid_show_line_pad(18, 210, "Exit auto-saves PID", (uint16)(ips200_width_max - 36U));
    }

    snprintf(value_line, sizeof(value_line), "%.4f", value);
    snprintf(step_line, sizeof(step_line), "%.4f", step);

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    menu_pid_show_line_pad(18, 104, value_line, (uint16)(ips200_width_max - 36U));

    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    menu_pid_show_line_pad(18, 160, step_line, (uint16)(ips200_width_max - 36U));

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
    char line0[64];
    char line1[64];
    char line2[64];
    char line3[64];
    char line4[64];
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
        menu_pid_show_line_pad(5, (uint16)(ips200_height_max - 20U), "K1 LONG Exit", (uint16)(ips200_width_max - 10U));
    }

    pid_preview_output = pid_calculate(preview_controller, pid_preview_target, pid_preview_feedback);
    pid_preview_feedback += pid_preview_output * 0.02f;
    pid_preview_feedback *= 0.995f;
    error = pid_preview_target - pid_preview_feedback;

    snprintf(line0, sizeof(line0), "Kp:%.3f Ki:%.3f", params->pid_p, params->pid_i);
    snprintf(line1, sizeof(line1), "Kd:%.3f Target:%.1f", params->pid_d, pid_preview_target);
    snprintf(line2, sizeof(line2), "Feedback:%.2f", pid_preview_feedback);
    snprintf(line3, sizeof(line3), "Error:%.2f Output:%.2f", error, pid_preview_output);
    snprintf(line4, sizeof(line4), "Adjust gains in PID menu");

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    menu_pid_show_line_pad(10, 56, line0, (uint16)(ips200_width_max - 20U));
    menu_pid_show_line_pad(10, 80, line1, (uint16)(ips200_width_max - 20U));
    menu_pid_show_line_pad(10, 104, line2, (uint16)(ips200_width_max - 20U));
    menu_pid_show_line_pad(10, 128, line3, (uint16)(ips200_width_max - 20U));
    menu_pid_show_line_pad(10, 152, line4, (uint16)(ips200_width_max - 20U));

    if (NULL != menu_full_redraw)
    {
        *menu_full_redraw = 0U;
    }
}

static void menu_pid_draw_active_view(pid_view_mode_t mode, const MyParams_t *params, pid_controller_t *preview_controller, uint8 *menu_full_redraw)
{
    if (PID_VIEW_EDIT_KP == mode)
    {
        menu_pid_draw_edit_page(mode, "Edit Kp", params->pid_p, menu_pid_get_current_step(PID_VIEW_EDIT_KP), menu_full_redraw);
    }
    else if (PID_VIEW_EDIT_KI == mode)
    {
        menu_pid_draw_edit_page(mode, "Edit Ki", params->pid_i, menu_pid_get_current_step(PID_VIEW_EDIT_KI), menu_full_redraw);
    }
    else if (PID_VIEW_EDIT_KD == mode)
    {
        menu_pid_draw_edit_page(mode, "Edit Kd", params->pid_d, menu_pid_get_current_step(PID_VIEW_EDIT_KD), menu_full_redraw);
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

    if ((switch_encoder_change_num != 0) && (*mode <= PID_VIEW_EDIT_KD))
    {
        step = menu_pid_get_current_step(*mode);
        if (PID_VIEW_EDIT_KP == *mode)
        {
            menu_pid_adjust_param(&params->pid_p, step, PARAM_PID_P_MIN, PARAM_PID_P_MAX, (switch_encoder_change_num < 0) ? 1U : 0U);
            updated = 1U;
        }
        else if (PID_VIEW_EDIT_KI == *mode)
        {
            menu_pid_adjust_param(&params->pid_i, step, PARAM_PID_I_MIN, PARAM_PID_I_MAX, (switch_encoder_change_num < 0) ? 1U : 0U);
            updated = 1U;
        }
        else if (PID_VIEW_EDIT_KD == *mode)
        {
            menu_pid_adjust_param(&params->pid_d, step, PARAM_PID_D_MIN, PARAM_PID_D_MAX, (switch_encoder_change_num < 0) ? 1U : 0U);
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
            menu_pid_cycle_step(*mode);
            if (NULL != menu_full_redraw)
            {
                *menu_full_redraw = 1U;
            }
        }
    }

    menu_pid_draw_active_view(*mode, params, preview_controller, menu_full_redraw);
    return 1U;
}
