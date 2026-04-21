#include "menu_gps.h"

#include <stdio.h>
#include <string.h>

#include "zf_common_headfile.h"
#include "zf_device_ips200.h"
#include "menu_ui_utils.h"
#include "display_gps.h"
#include "path_display.h"
#include "path_recorder.h"

static const float record_distance_steps[] = {0.05f, 0.10f, 0.20f, 0.50f};
static const uint16 record_interval_steps[] = {10U, 20U, 50U, 100U};
static const uint8 record_sat_steps[] = {1U, 2U, 3U};
static menu_step_desc_t record_step_descs[3] = {
    { record_distance_steps, 4U, 1U, 0U },  /* float, 4 entries, start at index 1 */
    { record_interval_steps, 4U, 1U, 1U },  /* uint16, 4 entries, start at index 1 */
    { record_sat_steps,      3U, 0U, 2U },  /* uint8, 3 entries, start at index 0 */
};

static void menu_gps_set_status_hint(char *status_hint, uint32 status_hint_size, const char *text);
static const char *menu_gps_get_record_state_text(void);
static void menu_gps_enter_view(gps_view_mode_t *gps_mode, gps_view_mode_t mode, uint8 *menu_full_redraw, void (*drain_encoder_events)(void), void (*request_redraw)(uint8 full_redraw), void (*reset_dynamic_region)(void));
static void menu_gps_return_to_menu(gps_view_mode_t *gps_mode, void (*request_redraw)(uint8 full_redraw));
static void menu_gps_return_to_menu_with_hint(gps_view_mode_t *gps_mode, char *status_hint, uint32 status_hint_size, const char *text, void (*request_redraw)(uint8 full_redraw));
static void menu_gps_return_to_menu_with_message(gps_view_mode_t *gps_mode, const char *text, void (*set_status_message)(const char *text, uint32 duration_ms), uint32 status_show_ms, void (*request_redraw)(uint8 full_redraw));
static void menu_gps_draw_page_header(const char *title);
static void menu_gps_clear_current_track(char *status_hint, uint32 status_hint_size, void (*request_redraw)(uint8 full_redraw));
static void menu_gps_draw_data_page(const char *status_hint, uint8 *menu_full_redraw);
static void menu_gps_draw_map_page(const char *status_hint, uint8 *menu_full_redraw);
static void menu_gps_draw_raw_debug_page(uint8 *menu_full_redraw);
static void menu_gps_record_param_draw_edit_page(record_param_view_mode_t mode, uint8 *menu_full_redraw, const char *title, const char *value_text, const char *step_text);
static void menu_gps_record_param_draw_active_view(record_param_view_mode_t mode, uint8 *menu_full_redraw);
static void menu_gps_record_param_apply_encoder_adjustment(record_param_view_mode_t mode, void (*sync_record_param_items)(void));

void menu_gps_init_state(char *status_hint, uint32 status_hint_size)
{
    menu_gps_set_status_hint(status_hint, status_hint_size, "");
}

static void menu_gps_set_status_hint(char *status_hint, uint32 status_hint_size, const char *text)
{
    menu_ui_copy_text(status_hint, status_hint_size, text);
}

uint8 menu_gps_menu_gnss_ready(void)
{
    const path_record_config_t *config = path_recorder_get_config();

    return ((gnss.state == 1) &&
            (gnss.satellite_used >= config->min_satellites) &&
            (gnss.speed <= config->max_record_speed_kph) &&
            (gnss.latitude != 0.0) &&
            (gnss.longitude != 0.0));
}

static const char *menu_gps_get_record_state_text(void)
{
    switch (path_recorder_get_state())
    {
        case PATH_STATE_RECORDING: return "REC";
        case PATH_STATE_COMPLETED: return "DONE";
        case PATH_STATE_IDLE:
        default: return "IDLE";
    }
}

static void menu_gps_enter_view(gps_view_mode_t *gps_mode, gps_view_mode_t mode, uint8 *menu_full_redraw, void (*drain_encoder_events)(void), void (*request_redraw)(uint8 full_redraw), void (*reset_dynamic_region)(void))
{
    *gps_mode = mode;
    if (NULL != drain_encoder_events) drain_encoder_events();
    my_key_clear_state(MY_KEY_1);
    my_key_clear_state(MY_KEY_4);
    if (NULL != reset_dynamic_region) reset_dynamic_region();
    if (NULL != menu_full_redraw) *menu_full_redraw = 1U;
    if (NULL != request_redraw) request_redraw(1U);
}

static void menu_gps_return_to_menu(gps_view_mode_t *gps_mode, void (*request_redraw)(uint8 full_redraw))
{
    *gps_mode = GPS_VIEW_NONE;
    if (NULL != request_redraw) request_redraw(1U);
}

static void menu_gps_return_to_menu_with_hint(gps_view_mode_t *gps_mode, char *status_hint, uint32 status_hint_size, const char *text, void (*request_redraw)(uint8 full_redraw))
{
    menu_gps_set_status_hint(status_hint, status_hint_size, text);
    menu_gps_return_to_menu(gps_mode, request_redraw);
}

static void menu_gps_return_to_menu_with_message(gps_view_mode_t *gps_mode, const char *text, void (*set_status_message)(const char *text, uint32 duration_ms), uint32 status_show_ms, void (*request_redraw)(uint8 full_redraw))
{
    if (NULL != set_status_message) set_status_message(text, status_show_ms);
    menu_gps_return_to_menu(gps_mode, request_redraw);
}

static void menu_gps_draw_page_header(const char *title)
{
    menu_ui_draw_page_header(title, 10, 30);
}

static void menu_gps_clear_current_track(char *status_hint, uint32 status_hint_size, void (*request_redraw)(uint8 full_redraw))
{
    path_state_enum state = path_recorder_get_state();
    path_recorder_clear();

    if ((state == PATH_STATE_RECORDING) && menu_gps_menu_gnss_ready())
    {
        path_recorder_start();
        menu_gps_set_status_hint(status_hint, status_hint_size, "Track reset");
    }
    else
    {
        menu_gps_set_status_hint(status_hint, status_hint_size, "Track cleared");
    }

    path_display_init();
    if (NULL != request_redraw) request_redraw(1U);
}

static void menu_gps_draw_data_page(const char *status_hint, uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char val[64];
    uint32 now_ms = system_getval_ms();
    float total_distance = 0.0f;

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) && (now_ms - last_refresh_ms < 200U)) return;
    last_refresh_ms = now_ms;
    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        menu_ui_fill_rect(0, 32, ips200_width_max - 1, ips200_height_max - 1, RGB565_BLACK);
        menu_gps_draw_page_header("GPS Data");
        menu_ui_show_pad(5, (uint16)(ips200_height_max - 20U), (uint16)(ips200_width_max - 10U), "K1 LONG Exit");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 60, "Fix:    Sat:    Rec:");
        ips200_show_string(10, 84, "Lat:");
        ips200_show_string(10, 108, "Lon:");
        ips200_show_string(10, 132, "Spd:        Dir:");
    }

    path_recorder_get_stats(&total_distance, NULL, NULL);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    /* y=60: Fix at x=42, Sat at x=106, Rec at x=170 */
    snprintf(val, sizeof(val), "%d", (int)gnss.state);
    menu_ui_show_pad(42, 60, 56, val);
    snprintf(val, sizeof(val), "%d", (int)gnss.satellite_used);
    menu_ui_show_pad(106, 60, 56, val);
    snprintf(val, sizeof(val), "%s", menu_gps_get_record_state_text());
    menu_ui_show_pad(170, 60, (uint16)(ips200_width_max - 180U), val);

    /* y=84: Lat at x=42 */
    snprintf(val, sizeof(val), "%.6f", gnss.latitude);
    menu_ui_show_pad(42, 84, (uint16)(ips200_width_max - 52U), val);

    /* y=108: Lon at x=42 */
    snprintf(val, sizeof(val), "%.6f", gnss.longitude);
    menu_ui_show_pad(42, 108, (uint16)(ips200_width_max - 52U), val);

    /* y=132: Spd at x=42, Dir at x=154 */
    snprintf(val, sizeof(val), "%.2f", gnss.speed);
    menu_ui_show_pad(42, 132, 104, val);
    snprintf(val, sizeof(val), "%.1f", gnss.direction);
    menu_ui_show_pad(154, 132, (uint16)(ips200_width_max - 164U), val);

    /* y=156: fully dynamic line */
    if ((NULL != status_hint) && (status_hint[0] != '\0')) snprintf(val, sizeof(val), "%s", status_hint);
    else snprintf(val, sizeof(val), "Pts:%d Dist:%.1fm", path_recorder_get_point_count(), total_distance);
    menu_ui_show_pad(10, 156, (uint16)(ips200_width_max - 20U), val);

    if (NULL != menu_full_redraw) *menu_full_redraw = 0U;
}

static void menu_gps_draw_map_page(const char *status_hint, uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    char val[64];
    uint16 map_x = 5U, map_y = 105U, map_w = (uint16)(ips200_width_max - 10U), map_h;
    uint32 now_ms = system_getval_ms();
    float total_distance = 0.0f;

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) && (now_ms - last_refresh_ms < 800U)) return;
    last_refresh_ms = now_ms;
    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        menu_ui_fill_rect(0, 32, ips200_width_max - 1, ips200_height_max - 1, RGB565_BLACK);
        menu_gps_draw_page_header("GPS Map");
        menu_ui_show_pad(5, (uint16)(ips200_height_max - 20U), (uint16)(ips200_width_max - 10U), "K1 Clear  LONG Exit");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 36, "Sat:    Pts:    Fix:");
        ips200_show_string(10, 76, "Spd:        H:");
    }

    path_recorder_get_stats(&total_distance, NULL, NULL);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    /* y=36: Sat at x=42, Pts at x=106, Fix at x=170 */
    snprintf(val, sizeof(val), "%d", (int)gnss.satellite_used);
    menu_ui_show_pad(42, 36, 56, val);
    snprintf(val, sizeof(val), "%d", path_recorder_get_point_count());
    menu_ui_show_pad(106, 36, 56, val);
    snprintf(val, sizeof(val), "%d", (int)gnss.state);
    menu_ui_show_pad(170, 36, (uint16)(ips200_width_max - 180U), val);

    /* y=56: fully dynamic line */
    if ((NULL != status_hint) && (status_hint[0] != '\0')) snprintf(val, sizeof(val), "%s", status_hint);
    else snprintf(val, sizeof(val), "Rec:%s Dist:%.1fm", menu_gps_get_record_state_text(), total_distance);
    menu_ui_show_pad(10, 56, (uint16)(ips200_width_max - 20U), val);

    /* y=76: Spd at x=42, H at x=122 */
    snprintf(val, sizeof(val), "%.1fkm/h", gnss.speed);
    menu_ui_show_pad(42, 76, 72, val);
    snprintf(val, sizeof(val), "%.1fm", gnss.height);
    menu_ui_show_pad(122, 76, (uint16)(ips200_width_max - 132U), val);

    map_h = (ips200_height_max > 190U) ? (uint16)(ips200_height_max - 190U) : (uint16)(ips200_height_max / 2U);
    if (map_h < 60U) map_h = 60U;
    if ((uint32)map_y + (uint32)map_h > (uint32)(ips200_height_max - 25U)) map_h = (uint16)(ips200_height_max - map_y - 25U);
    if (map_h < 60U) map_h = 60U;

    path_display_set_area(map_x, map_y, map_w, map_h);
    path_display_draw_map(RGB565_CYAN);
    if (NULL != menu_full_redraw) *menu_full_redraw = 0U;
}

static void menu_gps_draw_raw_debug_page(uint8 *menu_full_redraw)
{
    static uint32 last_refresh_ms = 0U;
    gnss_debug_info_t debug_info;
    char line[96];
    uint32 now_ms = system_getval_ms();

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) && (now_ms - last_refresh_ms < 300U)) return;
    last_refresh_ms = now_ms;
    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        menu_ui_fill_rect(0, 32, ips200_width_max - 1, ips200_height_max - 1, RGB565_BLACK);
        menu_gps_draw_page_header("GPS RAW/DEBUG");
        menu_ui_show_pad(5, (uint16)(ips200_height_max - 20U), (uint16)(ips200_width_max - 10U), "K1 LONG Exit");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(10, 112, "RMC:");
        ips200_show_string(10, 136, "GGA:");
        ips200_show_string(10, 160, "THS:");
    }

    gnss_get_debug_info(&debug_info);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    /* y=40,60,80: too many interleaved labels, keep as full show_pad */
    snprintf(line, sizeof(line), "Fix:%d Sat:%d Flag:%d Err:%lu", (int)gnss.state, (int)gnss.satellite_used, (int)debug_info.rx_flag, (unsigned long)debug_info.parse_error_count);
    menu_ui_show_pad(10, 40, (uint16)(ips200_width_max - 20U), line);
    snprintf(line, sizeof(line), "Init:%d R:%lu G:%lu T:%lu", (int)debug_info.initialized, (unsigned long)debug_info.rmc_frame_count, (unsigned long)debug_info.gga_frame_count, (unsigned long)debug_info.ths_frame_count);
    menu_ui_show_pad(10, 60, (uint16)(ips200_width_max - 20U), line);
    snprintf(line, sizeof(line), "St R:%u G:%u T:%u", debug_info.rmc_state, debug_info.gga_state, debug_info.ths_state);
    menu_ui_show_pad(10, 80, (uint16)(ips200_width_max - 20U), line);

    /* y=112,136,160: labels drawn once, update values from x=42 */
    snprintf(line, sizeof(line), "%s", (debug_info.rmc_sentence[0] != '\0') ? debug_info.rmc_sentence : "<none>");
    menu_ui_show_pad(42, 112, (uint16)(ips200_width_max - 52U), line);
    snprintf(line, sizeof(line), "%s", (debug_info.gga_sentence[0] != '\0') ? debug_info.gga_sentence : "<none>");
    menu_ui_show_pad(42, 136, (uint16)(ips200_width_max - 52U), line);
    snprintf(line, sizeof(line), "%s", (debug_info.ths_sentence[0] != '\0') ? debug_info.ths_sentence : "<none>");
    menu_ui_show_pad(42, 160, (uint16)(ips200_width_max - 52U), line);

    if (NULL != menu_full_redraw) *menu_full_redraw = 0U;
}

void menu_gps_action_display_data(gps_view_mode_t *gps_mode, uint8 *menu_full_redraw, void (*drain_encoder_events)(void), void (*request_redraw)(uint8 full_redraw), void (*reset_dynamic_region)(void))
{
    menu_gps_enter_view(gps_mode, GPS_VIEW_DATA, menu_full_redraw, drain_encoder_events, request_redraw, reset_dynamic_region);
}

void menu_gps_action_toggle_record(gps_view_mode_t *gps_mode, char *status_hint, uint32 status_hint_size, void (*set_status_message)(const char *text, uint32 duration_ms), uint32 status_show_ms, void (*request_redraw)(uint8 full_redraw))
{
    path_state_enum state = path_recorder_get_state();
    (void)set_status_message; (void)status_show_ms;

    if (state == PATH_STATE_RECORDING)
    {
        path_recorder_stop();
        menu_gps_return_to_menu_with_hint(gps_mode, status_hint, status_hint_size, "Track stopped", request_redraw);
    }
    else if (menu_gps_menu_gnss_ready())
    {
        if (path_recorder_start_new()) menu_gps_return_to_menu_with_hint(gps_mode, status_hint, status_hint_size, "Track recording", request_redraw);
        else menu_gps_return_to_menu_with_hint(gps_mode, status_hint, status_hint_size, "Start failed", request_redraw);
    }
    else
    {
        menu_gps_return_to_menu_with_hint(gps_mode, status_hint, status_hint_size, "GPS not ready", request_redraw);
    }
}

void menu_gps_action_map(gps_view_mode_t *gps_mode, uint8 *menu_full_redraw, void (*drain_encoder_events)(void), void (*request_redraw)(uint8 full_redraw), void (*reset_dynamic_region)(void))
{
    path_display_init();
    menu_gps_enter_view(gps_mode, GPS_VIEW_MAP, menu_full_redraw, drain_encoder_events, request_redraw, reset_dynamic_region);
}

void menu_gps_action_save_track_flash(gps_view_mode_t *gps_mode, void (*set_status_message)(const char *text, uint32 duration_ms), uint32 status_show_ms, void (*request_redraw)(uint8 full_redraw))
{
    if (path_recorder_save_to_flash()) menu_gps_return_to_menu_with_message(gps_mode, "Track saved", set_status_message, status_show_ms, request_redraw);
    else menu_gps_return_to_menu_with_message(gps_mode, "No track to save", set_status_message, status_show_ms, request_redraw);
}

void menu_gps_action_load_track_flash(gps_view_mode_t *gps_mode, void (*set_status_message)(const char *text, uint32 duration_ms), uint32 status_show_ms, void (*request_redraw)(uint8 full_redraw))
{
    if (path_recorder_get_state() == PATH_STATE_RECORDING) menu_gps_return_to_menu_with_message(gps_mode, "Stop record first", set_status_message, status_show_ms, request_redraw);
    else if (path_recorder_load_from_flash())
    {
        path_display_init();
        menu_gps_return_to_menu_with_message(gps_mode, "Track loaded", set_status_message, status_show_ms, request_redraw);
    }
    else menu_gps_return_to_menu_with_message(gps_mode, "Load failed", set_status_message, status_show_ms, request_redraw);
}

void menu_gps_action_raw_debug(gps_view_mode_t *gps_mode, uint8 *menu_full_redraw, void (*drain_encoder_events)(void), void (*request_redraw)(uint8 full_redraw), void (*reset_dynamic_region)(void))
{
    menu_gps_enter_view(gps_mode, GPS_VIEW_RAW_DEBUG, menu_full_redraw, drain_encoder_events, request_redraw, reset_dynamic_region);
}

void menu_gps_action_record_param_distance(record_param_view_mode_t *record_mode, uint8 *menu_full_redraw, void (*request_redraw)(uint8 full_redraw))
{
    *record_mode = RECORD_PARAM_VIEW_DISTANCE; my_key_clear_state(MY_KEY_1); if (NULL != menu_full_redraw) *menu_full_redraw = 1U; if (NULL != request_redraw) request_redraw(1U);
}
void menu_gps_action_record_param_interval(record_param_view_mode_t *record_mode, uint8 *menu_full_redraw, void (*request_redraw)(uint8 full_redraw))
{
    *record_mode = RECORD_PARAM_VIEW_INTERVAL; my_key_clear_state(MY_KEY_1); if (NULL != menu_full_redraw) *menu_full_redraw = 1U; if (NULL != request_redraw) request_redraw(1U);
}
void menu_gps_action_record_param_min_sat(record_param_view_mode_t *record_mode, uint8 *menu_full_redraw, void (*request_redraw)(uint8 full_redraw))
{
    *record_mode = RECORD_PARAM_VIEW_MIN_SAT; my_key_clear_state(MY_KEY_1); if (NULL != menu_full_redraw) *menu_full_redraw = 1U; if (NULL != request_redraw) request_redraw(1U);
}


static void menu_gps_record_param_apply_encoder_adjustment(record_param_view_mode_t mode, void (*sync_record_param_items)(void))
{
    const path_record_config_t *config = path_recorder_get_config();
    const menu_step_desc_t *sd = &record_step_descs[mode - RECORD_PARAM_VIEW_DISTANCE];
    uint8 is_add = (switch_encoder_change_num < 0) ? 1U : 0U;
    if (RECORD_PARAM_VIEW_DISTANCE == mode)
    {
        float value = config->min_record_distance;
        menu_param_adjust_float(&value, menu_step_get_float(sd), PARAM_RECORD_DISTANCE_MIN, PARAM_RECORD_DISTANCE_MAX, is_add);
        path_recorder_set_min_distance(value);
    }
    else if (RECORD_PARAM_VIEW_INTERVAL == mode)
    {
        uint32 value = config->min_record_interval_ms;
        menu_param_adjust_uint32(&value, menu_step_get_uint(sd), PARAM_RECORD_INTERVAL_MIN, PARAM_RECORD_INTERVAL_MAX, is_add);
        path_recorder_set_min_interval_ms(value);
    }
    else if (RECORD_PARAM_VIEW_MIN_SAT == mode)
    {
        uint32 value = config->min_satellites;
        menu_param_adjust_uint32(&value, menu_step_get_uint(sd), PARAM_RECORD_SAT_MIN, PARAM_RECORD_SAT_MAX, is_add);
        path_recorder_set_min_satellites((uint8)value);
    }
    if (NULL != sync_record_param_items) sync_record_param_items();
}

static void menu_gps_record_param_draw_edit_page(record_param_view_mode_t mode, uint8 *menu_full_redraw, const char *title, const char *value_text, const char *step_text)
{
    static record_param_view_mode_t last_draw_mode = RECORD_PARAM_VIEW_NONE;
    static char last_value_text[32] = "";
    static char last_step_text[32] = "";

    if ((NULL != menu_full_redraw) && !(*menu_full_redraw) && last_draw_mode == mode && 0 == strcmp(last_value_text, value_text) && 0 == strcmp(last_step_text, step_text)) return;

    if ((NULL != menu_full_redraw) && *menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        menu_ui_show_fit(10, 10, title);
        ips200_draw_line(0, 30, ips200_width_max - 1, 30, RGB565_GRAY);
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        menu_ui_show_pad(10, 40, (uint16)(ips200_width_max - 20U), "ENC: Adjust record param");
        menu_ui_show_pad(10, 56, (uint16)(ips200_width_max - 20U), "K1: Change step   LONG: Back");
        menu_ui_show_pad(18, 88, (uint16)(ips200_width_max - 36U), "Current Value");
        menu_ui_show_pad(18, 144, (uint16)(ips200_width_max - 36U), "Step Size");
    }

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    menu_ui_show_pad(18, 110, (uint16)(ips200_width_max - 36U), value_text);
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    menu_ui_show_pad(18, 166, (uint16)(ips200_width_max - 36U), step_text);
    strncpy(last_value_text, value_text, sizeof(last_value_text) - 1U); last_value_text[sizeof(last_value_text) - 1U] = '\0';
    strncpy(last_step_text, step_text, sizeof(last_step_text) - 1U); last_step_text[sizeof(last_step_text) - 1U] = '\0';
    last_draw_mode = mode;
    if (NULL != menu_full_redraw) *menu_full_redraw = 0U;
}

static void menu_gps_record_param_draw_active_view(record_param_view_mode_t mode, uint8 *menu_full_redraw)
{
    const path_record_config_t *config = path_recorder_get_config();
    const menu_step_desc_t *sd = &record_step_descs[mode - RECORD_PARAM_VIEW_DISTANCE];
    char value_text[32], step_text[32];
    if (RECORD_PARAM_VIEW_DISTANCE == mode)
    {
        snprintf(value_text, sizeof(value_text), "%.2f m", config->min_record_distance);
        snprintf(step_text, sizeof(step_text), "%.2f m", menu_step_get_float(sd));
        menu_gps_record_param_draw_edit_page(mode, menu_full_redraw, "Point Distance", value_text, step_text);
    }
    else if (RECORD_PARAM_VIEW_INTERVAL == mode)
    {
        snprintf(value_text, sizeof(value_text), "%lu ms", (unsigned long)config->min_record_interval_ms);
        snprintf(step_text, sizeof(step_text), "%lu ms", (unsigned long)menu_step_get_uint(sd));
        menu_gps_record_param_draw_edit_page(mode, menu_full_redraw, "Point Interval", value_text, step_text);
    }
    else if (RECORD_PARAM_VIEW_MIN_SAT == mode)
    {
        snprintf(value_text, sizeof(value_text), "%u", config->min_satellites);
        snprintf(step_text, sizeof(step_text), "%lu", (unsigned long)menu_step_get_uint(sd));
        menu_gps_record_param_draw_edit_page(mode, menu_full_redraw, "Min Satellites", value_text, step_text);
    }
}

uint8 menu_gps_handle_record_param_view(record_param_view_mode_t *record_mode, MyParams_t *params, uint8 *menu_full_redraw, void (*request_redraw)(uint8 full_redraw), void (*set_status_message)(const char *text, uint32 duration_ms), uint32 status_show_ms, void (*sync_record_param_items)(void))
{
    if ((NULL == record_mode) || (RECORD_PARAM_VIEW_NONE == *record_mode)) return 0U;

    if (MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1))
    {
        my_key_clear_state(MY_KEY_1);
        *record_mode = RECORD_PARAM_VIEW_NONE;
        menu_params_capture_record_config(params);
        if (menu_params_save_to_flash(params)) set_status_message("Record saved", status_show_ms);
        else set_status_message("Param flash full", status_show_ms);
        if (NULL != sync_record_param_items) sync_record_param_items();
        if (NULL != request_redraw) request_redraw(1U);
        return 1U;
    }

    /* 抽干 pending 队列：快转不滞后 */
    while (If_Switch_Encoder_Change()) menu_gps_record_param_apply_encoder_adjustment(*record_mode, sync_record_param_items);
    if (MY_KEY_SHORT_PRESS == my_key_get_state(MY_KEY_1))
    {
        my_key_clear_state(MY_KEY_1);
        menu_step_cycle(&record_step_descs[*record_mode - RECORD_PARAM_VIEW_DISTANCE]);
        if (NULL != menu_full_redraw) *menu_full_redraw = 1U;
    }

    menu_gps_record_param_draw_active_view(*record_mode, menu_full_redraw);
    return 1U;
}

uint8 menu_gps_handle_view(gps_view_mode_t *gps_mode, char *status_hint, uint32 status_hint_size, uint8 *menu_full_redraw, void (*drain_encoder_events)(void), void (*request_redraw)(uint8 full_redraw), void (*reset_dynamic_region)(void))
{
    if ((NULL == gps_mode) || (GPS_VIEW_NONE == *gps_mode)) return 0U;

    if (MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1))
    {
        my_key_clear_state(MY_KEY_1);
        *gps_mode = GPS_VIEW_NONE;
        if (NULL != drain_encoder_events) drain_encoder_events();
        if (NULL != reset_dynamic_region) reset_dynamic_region();
        if (NULL != request_redraw) request_redraw(1U);
        return 1U;
    }

    if ((GPS_VIEW_MAP == *gps_mode) && (MY_KEY_SHORT_PRESS == my_key_get_state(MY_KEY_1)))
    {
        my_key_clear_state(MY_KEY_1);
        menu_gps_clear_current_track(status_hint, status_hint_size, request_redraw);
        return 1U;
    }

    if (GPS_VIEW_DATA == *gps_mode) menu_gps_draw_data_page(status_hint, menu_full_redraw);
    else if (GPS_VIEW_MAP == *gps_mode) menu_gps_draw_map_page(status_hint, menu_full_redraw);
    else if (GPS_VIEW_RAW_DEBUG == *gps_mode) menu_gps_draw_raw_debug_page(menu_full_redraw);
    return 1U;
}
