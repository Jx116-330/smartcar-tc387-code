/*********************************************************************************************************************
* File: menu_turn.c
* Brief: Steering motor menu page.
*********************************************************************************************************************/

#include "menu_turn.h"

#include <stdio.h>

#include "Turn.h"
#include "MyEncoder.h"
#include "MyKey.h"
#include "menu_ui_utils.h"
#include "zf_common_headfile.h"
#include "zf_device_ips200.h"

#define TURN_MENU_REFRESH_MS       100U
#define TURN_TARGET_MIN_DEG        (-60.0f)
#define TURN_TARGET_MAX_DEG        (60.0f)
#define TURN_KP_MIN                0.0f
#define TURN_KP_MAX                50.0f
#define TURN_KI_MIN                0.0f
#define TURN_KI_MAX                10.0f
#define TURN_KD_MIN                0.0f
#define TURN_KD_MAX                20.0f

typedef enum
{
    TURN_VIEW_NONE = 0U,
    TURN_VIEW_TARGET,
    TURN_VIEW_KP,
    TURN_VIEW_KI,
    TURN_VIEW_KD,
    TURN_VIEW_MONITOR,
} turn_view_mode_t;

static const float turn_target_steps[] = {0.1f, 0.5f, 1.0f, 5.0f};
static const float turn_kp_steps[] = {0.01f, 0.10f, 0.50f, 1.00f};
static const float turn_ki_steps[] = {0.001f, 0.005f, 0.010f, 0.050f};
static const float turn_kd_steps[] = {0.01f, 0.05f, 0.10f, 0.20f};

static menu_step_desc_t turn_step_descs[4] = {
    { turn_target_steps, 4U, 2U, 0U },
    { turn_kp_steps,     4U, 1U, 0U },
    { turn_ki_steps,     4U, 2U, 0U },
    { turn_kd_steps,     4U, 1U, 0U },
};

static turn_view_mode_t turn_view_mode = TURN_VIEW_NONE;
static uint8 turn_full_redraw = 1U;
static uint32 turn_exit_hold_timer = 0U;

static char turn_target_label[48] = "1. Target [0.0deg]";
static char turn_kp_label[48] = "2. Kp [8.000]";
static char turn_ki_label[48] = "3. Ki [0.020]";
static char turn_kd_label[48] = "4. Kd [0.400]";
static char turn_enable_label[48] = "5. Enable [OFF]";
static char turn_encoder_label[48] = "6. Encoder [0]";
static char turn_angle_label[48] = "7. Angle [0.0deg]";

static void turn_action_target(void);
static void turn_action_kp(void);
static void turn_action_ki(void);
static void turn_action_kd(void);
static void turn_action_enable(void);
static void turn_action_monitor(void);
static void turn_action_zero(void);
static void turn_enter_view(turn_view_mode_t mode);
static void turn_drain_encoder_events(void);
static uint8 turn_update_labels(void);
static void turn_draw_edit_page(turn_view_mode_t mode, const char *title, float value, float step);
static void turn_draw_monitor_page(void);
static uint8 turn_handle_edit_view(turn_view_mode_t mode);

static MenuItem turn_items[] = {
    {turn_target_label,  turn_action_target,  NULL},
    {turn_kp_label,      turn_action_kp,      NULL},
    {turn_ki_label,      turn_action_ki,      NULL},
    {turn_kd_label,      turn_action_kd,      NULL},
    {turn_enable_label,  turn_action_enable,  NULL},
    {turn_encoder_label, turn_action_monitor, NULL},
    {turn_angle_label,   turn_action_monitor, NULL},
    {"8. Zero Current",  turn_action_zero,    NULL},
};

MenuPage turn_menu = {
    "Turn",
    turn_items,
    sizeof(turn_items) / sizeof(MenuItem),
    NULL
};

static void turn_drain_encoder_events(void)
{
    while (If_Switch_Encoder_Change())
    {
    }

    switch_encoder_change_num = 0;
    switch_encode_bring_flag = 0U;
    switch_encode_change_get_buff_flag = 0U;
}

static void turn_enter_view(turn_view_mode_t mode)
{
    turn_view_mode = mode;
    turn_full_redraw = 1U;
    menu_ui_consume_key1();
    turn_drain_encoder_events();
}

static void turn_action_target(void)
{
    turn_enter_view(TURN_VIEW_TARGET);
}

static void turn_action_kp(void)
{
    turn_enter_view(TURN_VIEW_KP);
}

static void turn_action_ki(void)
{
    turn_enter_view(TURN_VIEW_KI);
}

static void turn_action_kd(void)
{
    turn_enter_view(TURN_VIEW_KD);
}

static void turn_action_enable(void)
{
    uint8 *enable_ptr = Turn_GetMenuMotorEnablePtr();

    if (NULL == enable_ptr)
    {
        return;
    }

    *enable_ptr = (0U == *enable_ptr) ? 1U : 0U;
    Turn_SetMotorEnable(*enable_ptr);
    (void)turn_update_labels();
    menu_request_full_redraw();
}

static void turn_action_monitor(void)
{
    turn_enter_view(TURN_VIEW_MONITOR);
}

static void turn_action_zero(void)
{
    Turn_SetCurrentAngleAsZero();
    Turn_MenuRuntimeUpdate();
    (void)turn_update_labels();
    menu_request_full_redraw();
}

static uint8 turn_update_labels(void)
{
    char buf[48];
    uint8 changed = 0U;
    float *target_ptr;
    float *encoder_ptr;
    float *angle_ptr;
    uint8 *enable_ptr;

    Turn_MenuRuntimeUpdate();

    target_ptr = Turn_GetMenuTargetAngleDegPtr();
    encoder_ptr = Turn_GetMenuEncoderValuePtr();
    angle_ptr = Turn_GetMenuAngleDegPtr();
    enable_ptr = Turn_GetMenuMotorEnablePtr();

    snprintf(buf, sizeof(buf), "1. Target [%.1fdeg]", (NULL != target_ptr) ? *target_ptr : 0.0f);
    changed |= menu_ui_update_label(turn_target_label, sizeof(turn_target_label), buf);

    snprintf(buf, sizeof(buf), "2. Kp [%.3f]", *Turn_GetMenuKpPtr());
    changed |= menu_ui_update_label(turn_kp_label, sizeof(turn_kp_label), buf);

    snprintf(buf, sizeof(buf), "3. Ki [%.3f]", *Turn_GetMenuKiPtr());
    changed |= menu_ui_update_label(turn_ki_label, sizeof(turn_ki_label), buf);

    snprintf(buf, sizeof(buf), "4. Kd [%.3f]", *Turn_GetMenuKdPtr());
    changed |= menu_ui_update_label(turn_kd_label, sizeof(turn_kd_label), buf);

    snprintf(buf, sizeof(buf), "5. Enable [%s]", ((NULL != enable_ptr) && (0U != *enable_ptr)) ? "ON" : "OFF");
    changed |= menu_ui_update_label(turn_enable_label, sizeof(turn_enable_label), buf);

    snprintf(buf, sizeof(buf), "6. Encoder [%ld]", (long)((NULL != encoder_ptr) ? *encoder_ptr : 0.0f));
    changed |= menu_ui_update_label(turn_encoder_label, sizeof(turn_encoder_label), buf);

    snprintf(buf, sizeof(buf), "7. Angle [%.1fdeg]", (NULL != angle_ptr) ? *angle_ptr : 0.0f);
    changed |= menu_ui_update_label(turn_angle_label, sizeof(turn_angle_label), buf);

    return changed;
}

void menu_turn_sync_labels(void)
{
    static uint32 last_sync_ms = 0U;
    uint32 now_ms = system_getval_ms();

    if ((uint32)(now_ms - last_sync_ms) < TURN_MENU_REFRESH_MS)
    {
        return;
    }

    last_sync_ms = now_ms;
    (void)turn_update_labels();
}

void Turn_Page_Init(void)
{
    turn_view_mode = TURN_VIEW_NONE;
    turn_full_redraw = 1U;
    turn_exit_hold_timer = 0U;
    (void)turn_update_labels();
}

uint8 menu_turn_is_active(void)
{
    return (TURN_VIEW_NONE != turn_view_mode) ? 1U : 0U;
}

static void turn_draw_edit_page(turn_view_mode_t mode, const char *title, float value, float step)
{
    char value_line[32];
    char step_line[32];
    static turn_view_mode_t last_draw_mode = TURN_VIEW_NONE;
    static float last_draw_value = -100000.0f;
    static float last_draw_step = -1.0f;

    if (!turn_full_redraw &&
        (last_draw_mode == mode) &&
        (last_draw_value == value) &&
        (last_draw_step == step))
    {
        return;
    }

    if (turn_full_redraw)
    {
        ips200_full(RGB565_BLACK);
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(10, 10, (char *)title);
        ips200_draw_line(0, 30, ips200_width_max - 1, 30, RGB565_GRAY);
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(10, 40, (uint16)(ips200_width_max - 20U), "ENC: Adjust parameter");
        menu_ui_show_pad(10, 56, (uint16)(ips200_width_max - 20U), "K1: Change step   LONG: Exit");
        menu_ui_show_pad(18, 82, (uint16)(ips200_width_max - 36U), "Current Value");
        menu_ui_show_pad(18, 138, (uint16)(ips200_width_max - 36U), "Step Size");
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
    turn_full_redraw = 0U;
}

static uint8 turn_handle_edit_view(turn_view_mode_t mode)
{
    float *value_ptr = NULL;
    float min_val = 0.0f;
    float max_val = 0.0f;
    const char *title = "";
    uint8 step_index = 0U;
    uint8 updated = 0U;

    if (TURN_VIEW_TARGET == mode)
    {
        value_ptr = Turn_GetMenuTargetAngleDegPtr();
        min_val = TURN_TARGET_MIN_DEG;
        max_val = TURN_TARGET_MAX_DEG;
        title = "Turn Target";
        step_index = 0U;
    }
    else if (TURN_VIEW_KP == mode)
    {
        value_ptr = Turn_GetMenuKpPtr();
        min_val = TURN_KP_MIN;
        max_val = TURN_KP_MAX;
        title = "Turn Kp";
        step_index = 1U;
    }
    else if (TURN_VIEW_KI == mode)
    {
        value_ptr = Turn_GetMenuKiPtr();
        min_val = TURN_KI_MIN;
        max_val = TURN_KI_MAX;
        title = "Turn Ki";
        step_index = 2U;
    }
    else if (TURN_VIEW_KD == mode)
    {
        value_ptr = Turn_GetMenuKdPtr();
        min_val = TURN_KD_MIN;
        max_val = TURN_KD_MAX;
        title = "Turn Kd";
        step_index = 3U;
    }

    if (NULL == value_ptr)
    {
        return 1U;
    }

    if (If_Switch_Encoder_Change())
    {
        menu_param_adjust_float(value_ptr,
                                menu_step_get_float(&turn_step_descs[step_index]),
                                min_val,
                                max_val,
                                (switch_encoder_change_num < 0) ? 1U : 0U);
        updated = 1U;
    }

    if (MY_KEY_SHORT_PRESS == my_key_get_state(MY_KEY_1))
    {
        menu_ui_consume_key1();
        menu_step_cycle(&turn_step_descs[step_index]);
        turn_full_redraw = 1U;
    }

    if (updated)
    {
        if (TURN_VIEW_TARGET == mode)
        {
            Turn_MenuTargetAngleSync();
        }
        else
        {
            Turn_MenuPidSync();
        }
        (void)turn_update_labels();
    }

    turn_draw_edit_page(mode, title, *value_ptr, menu_step_get_float(&turn_step_descs[step_index]));
    return 1U;
}

static void turn_draw_monitor_page(void)
{
    static uint32 last_refresh_ms = 0U;
    char line[40];
    uint32 now_ms = system_getval_ms();
    uint16 val_w = (uint16)(ips200_width_max - 98U - 10U);

    if (!turn_full_redraw && ((uint32)(now_ms - last_refresh_ms) < TURN_MENU_REFRESH_MS))
    {
        return;
    }
    last_refresh_ms = now_ms;

    Turn_MenuRuntimeUpdate();

    if (turn_full_redraw)
    {
        ips200_full(RGB565_BLACK);
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(10, 2, "Turn Monitor");
        ips200_draw_line(0, 16, ips200_width_max - 1, 16, RGB565_GRAY);

        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 40,  "Target : ");
        ips200_show_string(10, 60,  "Current: ");
        ips200_show_string(10, 80,  "Encoder: ");
        ips200_show_string(10, 100, "Enable : ");
        ips200_show_string(10, 120, "Kp/Ki/Kd:");

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(5U, (uint16)(ips200_height_max - 16U),
                         (uint16)(ips200_width_max - 10U), "LONG:Exit");
        turn_full_redraw = 0U;
    }

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    snprintf(line, sizeof(line), "%.2fdeg", turn_target_angle_deg);
    menu_ui_show_pad(98, 40, val_w, line);

    snprintf(line, sizeof(line), "%.2fdeg", turn_current_angle_deg);
    menu_ui_show_pad(98, 60, val_w, line);

    snprintf(line, sizeof(line), "%ld", (long)Turn_GetEncoderCount());
    menu_ui_show_pad(98, 80, val_w, line);

    snprintf(line, sizeof(line), "%s", Turn_GetMotorEnable() ? "ON" : "OFF");
    ips200_set_color(Turn_GetMotorEnable() ? RGB565_GREEN : RGB565_RED, RGB565_BLACK);
    menu_ui_show_pad(98, 100, val_w, line);

    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    snprintf(line, sizeof(line), "%.2f %.3f %.2f",
             *Turn_GetMenuKpPtr(),
             *Turn_GetMenuKiPtr(),
             *Turn_GetMenuKdPtr());
    menu_ui_show_pad(98, 120, val_w, line);
}

uint8 menu_turn_handle_view(void)
{
    if (TURN_VIEW_NONE == turn_view_mode)
    {
        return 0U;
    }

    if ((MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1)) ||
        menu_ui_check_exit_hold(&turn_exit_hold_timer, 250U))
    {
        menu_ui_consume_key1();
        turn_drain_encoder_events();
        turn_view_mode = TURN_VIEW_NONE;
        turn_full_redraw = 1U;
        (void)turn_update_labels();
        menu_request_full_redraw();
        return 1U;
    }

    if (TURN_VIEW_MONITOR == turn_view_mode)
    {
        turn_draw_monitor_page();
        return 1U;
    }

    return turn_handle_edit_view(turn_view_mode);
}
