/*********************************************************************************************************************
* File: menu.c
* Brief: 菜单系统实现文件
* Author: JX116
* Note: 负责菜单绘制、编码器和按键交互，以及 GPS / PID 页面切换与参数加载保存逻辑
*********************************************************************************************************************/

#include "menu.h"
#include "zf_device_ips200.h"
#include "zf_common_headfile.h"
#include "zf_driver_gpio.h"
#include "zf_driver_flash.h"
#include "path_config.h"
#include "path_recorder.h"
#include "path_display.h"
#include "MyKey.h"
#include "MyEncoder.h"
#include "zf_device_wifi_spi.h"


/* 参数结构体：用于保存 PID 和菜单相关配置 */
typedef struct {
    float pid_p;
    float pid_i;
    float pid_d;
    float record_min_distance;
    uint32 record_min_interval_ms;
    float record_max_speed_kph;
    uint8 record_min_satellites;
    uint8 threshold_mode;
    uint8 threshold_val;
    uint32_t magic_code;
} MyParams_t;

typedef enum
{
    GPS_VIEW_NONE = 0U,
    GPS_VIEW_DATA,
    GPS_VIEW_MAP,
    GPS_VIEW_RAW_DEBUG,
} gps_view_mode_t;

typedef enum
{
    PID_VIEW_NONE = 0U,
    PID_VIEW_EDIT_KP,
    PID_VIEW_EDIT_KI,
    PID_VIEW_EDIT_KD,
    PID_VIEW_PREVIEW,
} pid_view_mode_t;

typedef enum
{
    RECORD_PARAM_VIEW_NONE = 0U,
    RECORD_PARAM_VIEW_DISTANCE,
    RECORD_PARAM_VIEW_INTERVAL,
    RECORD_PARAM_VIEW_MIN_SAT,
} record_param_view_mode_t;

MyParams_t g_params;

/* Flash 参数存储位置定义 */
#define FLASH_SECTOR    0
#define FLASH_PAGE      127
#define MAGIC_NUM       0x5A5A5A5A
#define SAVE_LEN        ((sizeof(MyParams_t) + 3) / 4)
#define PARAM_PID_P_MIN             0.0f
#define PARAM_PID_P_MAX             50.0f
#define PARAM_PID_I_MIN             0.0f
#define PARAM_PID_I_MAX             10.0f
#define PARAM_PID_D_MIN             0.0f
#define PARAM_PID_D_MAX             20.0f
#define PARAM_THRESHOLD_MODE_MAX    3U
#define PARAM_RECORD_DISTANCE_MIN   0.05f
#define PARAM_RECORD_DISTANCE_MAX   5.0f
#define PARAM_RECORD_INTERVAL_MIN   10U
#define PARAM_RECORD_INTERVAL_MAX   1000U
#define PARAM_RECORD_SPEED_MIN      5.0f
#define PARAM_RECORD_SPEED_MAX      120.0f
#define PARAM_RECORD_SAT_MIN        3U
#define PARAM_RECORD_SAT_MAX        12U
#define MENU_STATUS_SHOW_MS         1200U
#define MENU_TITLE_HEIGHT           30U
#define MENU_ITEM_START_Y           40U
#define MENU_ITEM_HEIGHT            20U
#define MENU_ITEM_TEXT_HEIGHT       16U
#define MENU_FOOTER_HEIGHT          20U

/* 菜单显示状态与运行时缓存 */
static uint8 menu_needs_update = 1;
static uint16 current_font_color = RGB565_WHITE;
static int current_selection = 0;
static int last_selection = -1;
static uint8 menu_full_redraw = 1;
static menu_dynamic_draw_t menu_dynamic_draw_cb = NULL;
static uint16 menu_dynamic_x = 0;
static uint16 menu_dynamic_y = 0;
static uint16 menu_dynamic_w = 0;
static uint16 menu_dynamic_h = 0;
static uint8 menu_dynamic_clear_enable = 1;
static gps_view_mode_t gps_display_mode = GPS_VIEW_NONE;
static pid_view_mode_t pid_display_mode = PID_VIEW_NONE;
static record_param_view_mode_t record_param_view_mode = RECORD_PARAM_VIEW_NONE;
static char gps_status_hint[64] = "";
static MenuPage *current_page = NULL;
static MenuPage gps_menu;
static pid_param_t menu_pid_param_cache;
static pid_controller_t pid_preview_controller;
static float pid_preview_target = 100.0f;
static float pid_preview_feedback = 0.0f;
static float pid_preview_output = 0.0f;
static const float pid_kp_steps[] = {0.01f, 0.10f, 0.50f, 1.00f};
static const float pid_ki_steps[] = {0.001f, 0.005f, 0.010f, 0.050f};
static const float pid_kd_steps[] = {0.01f, 0.05f, 0.10f, 0.20f};
static uint8 pid_kp_step_index = 1U;
static uint8 pid_ki_step_index = 2U;
static uint8 pid_kd_step_index = 1U;
static char menu_status_message[32] = "";
static uint32 menu_status_expire_ms = 0U;
static uint8 menu_footer_needs_update = 1U;
static uint8 pid_param_dirty = 0U;
static char gps_record_menu_label[48] = "2. Track Record [IDLE]";
static char gps_record_distance_label[48] = "1. Point Dist [0.20m]";
static char gps_record_interval_label[48] = "2. Point Intv [50ms]";
static char gps_record_sat_label[48] = "3. Min Sat [4]";
static const float record_distance_steps[] = {0.05f, 0.10f, 0.20f, 0.50f};
static const uint16 record_interval_steps[] = {10U, 20U, 50U, 100U};
static const uint8 record_sat_steps[] = {1U, 2U, 3U};
static uint8 record_distance_step_index = 1U;
static uint8 record_interval_step_index = 1U;
static uint8 record_sat_step_index = 0U;

/* ======================== WiFi 相关定义 ======================== */
typedef enum {
    WIFI_VIEW_NONE = 0U,
    WIFI_VIEW_LIST,
    WIFI_VIEW_CONNECTING,
    WIFI_VIEW_STATUS,
    WIFI_VIEW_TEST,
} wifi_view_mode_t;

typedef struct {
    const char *ssid;
    const char *password;   /* NULL = 无密码 */
    const char *label;      /* 菜单显示名称 */
} wifi_preset_t;

static const wifi_preset_t wifi_presets[] = {
    {"Jx116",  "777888999", "HOT SPOT"},
    {"JX116_Lab", "12345678",    "JX116 Lab"},
    {"Phone_AP",  "88888888",    "Phone Hotspot"},
};
#define WIFI_PRESET_COUNT  ((int)(sizeof(wifi_presets) / sizeof(wifi_preset_t)))

static wifi_view_mode_t wifi_view_mode = WIFI_VIEW_NONE;
static uint8 wifi_initialized = 0U;
static uint8 wifi_connected = 0U;
static int   wifi_list_selection = 0;
static int   wifi_list_last_sel = -1;
static uint8 wifi_list_full_redraw = 1U;
static uint8 wifi_connect_result = 1U;     /* 0-成功 1-失败 */
static char  wifi_connected_ssid[32] = "";
static uint8 wifi_test_ver_ok = 0U;
static uint8 wifi_test_mac_ok = 0U;
static uint8 wifi_test_ip_ok = 0U;
static uint8 wifi_test_stage_ver = 0xFFU;
static uint8 wifi_test_stage_mac = 0xFFU;
static uint8 wifi_test_stage_wifi = 0xFFU;
/* ============================================================== */

static void ips200_fill_rect(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, uint16 color);
static void show_string_fit(uint16 x, uint16 y, const char *s);
static void show_string_fit_width(uint16 x, uint16 y, uint16 max_width, const char *s);
static void show_string_fit_width_pad(uint16 x, uint16 y, uint16 max_width, const char *s);
static void menu_set_status_message(const char *text, uint32 duration_ms);
static void menu_draw_footer(void);
static void menu_drain_encoder_events(void);
static void menu_reset_dynamic_region(void);
static void menu_request_redraw(uint8 full_redraw);
static void menu_process_status_timeout(uint32 now_ms);
static void menu_update_selection_from_encoder(void);
static void menu_sync_gps_record_item(void);
static void menu_sync_gps_record_param_items(void);
static void menu_draw_item(int index, uint8 selected);
static void menu_draw_page(void);
static void menu_enter_page(MenuPage *page);
static void menu_return_to_parent(void);
static void menu_execute_current_item(void);
static uint8 menu_handle_gps_view(void);
static uint8 menu_handle_pid_view(void);
static uint8 menu_handle_record_param_view(void);
static uint8 menu_handle_wifi_view(void);
static void  wifi_enter_view(wifi_view_mode_t mode);
static void  wifi_exit_view(void);
static uint8 params_are_valid(const MyParams_t *params);
static void params_save_to_flash(void);
static void params_apply_record_config(void);
static void params_capture_record_config(void);
static void gps_clear_status_hint(void);
static void gps_set_status_hint(const char *text);
static const char *gps_get_record_state_text(void);
static void gps_return_to_menu(void);
static void gps_enter_view(gps_view_mode_t mode);
static void gps_exit_view(void);
static void gps_clear_current_track(void);
static void gps_draw_page_header(const char *title);
static void gps_draw_data_page(void);
static void gps_draw_map_page(void);
static void gps_draw_raw_debug_page(void);
static void record_param_enter_view(record_param_view_mode_t mode);
static void record_param_exit_view(void);
static void record_param_apply_encoder_adjustment(void);
static void record_param_draw_active_view(void);
static void record_param_draw_edit_page(const char *title, const char *value_text, const char *step_text);
static float record_param_get_distance_step(void);
static uint16 record_param_get_interval_step(void);
static uint8 record_param_get_sat_step(void);
static void record_param_cycle_step(void);
static void gps_action_record_param_distance(void);
static void gps_action_record_param_interval(void);
static void gps_action_record_param_min_sat(void);
static void pid_sync_runtime_param(void);
static void pid_save_if_dirty(void);
static void pid_reset_preview_state(void);
static void pid_enter_view(pid_view_mode_t mode);
static void pid_exit_view(void);
static void pid_apply_encoder_adjustment(void);
static void pid_draw_active_view(void);
static void pid_draw_edit_page(const char *title, float value, float step);
static void pid_draw_preview_page(void);
static float pid_get_current_step(uint8 mode);
static void pid_cycle_step(uint8 mode);
static void pid_action_edit_kp(void);
static void pid_action_edit_ki(void);
static void pid_action_edit_kd(void);
static void pid_action_preview(void);
static void pid_action_reset(void);
static void gps_action_raw_debug(void);

static uint8 gps_menu_gnss_ready(void)
{
    const path_record_config_t *config = path_recorder_get_config();

    return ((gnss.state == 1) &&
            (gnss.satellite_used >= config->min_satellites) &&
            (gnss.speed <= config->max_record_speed_kph) &&
            (gnss.latitude != 0.0) &&
            (gnss.longitude != 0.0));
}

/* 检查从 Flash 读取的参数是否有效，避免加载异常数据 */
static uint8 params_are_valid(const MyParams_t *params)
{
    if (NULL == params)
    {
        return 0;
    }

    if (params->magic_code != MAGIC_NUM)
    {
        return 0;
    }

    if (!(params->pid_p >= PARAM_PID_P_MIN && params->pid_p <= PARAM_PID_P_MAX))
    {
        return 0;
    }

    if (!(params->pid_i >= PARAM_PID_I_MIN && params->pid_i <= PARAM_PID_I_MAX))
    {
        return 0;
    }

    if (!(params->pid_d >= PARAM_PID_D_MIN && params->pid_d <= PARAM_PID_D_MAX))
    {
        return 0;
    }

    if (!(params->record_min_distance >= PARAM_RECORD_DISTANCE_MIN && params->record_min_distance <= PARAM_RECORD_DISTANCE_MAX))
    {
        return 0;
    }

    if (!(params->record_min_interval_ms >= PARAM_RECORD_INTERVAL_MIN && params->record_min_interval_ms <= PARAM_RECORD_INTERVAL_MAX))
    {
        return 0;
    }

    if (!(params->record_max_speed_kph >= PARAM_RECORD_SPEED_MIN && params->record_max_speed_kph <= PARAM_RECORD_SPEED_MAX))
    {
        return 0;
    }

    if (!(params->record_min_satellites >= PARAM_RECORD_SAT_MIN && params->record_min_satellites <= PARAM_RECORD_SAT_MAX))
    {
        return 0;
    }

    if (params->threshold_mode > PARAM_THRESHOLD_MODE_MAX)
    {
        return 0;
    }

    return 1;
}

static void params_save_to_flash(void)
{
    g_params.magic_code = MAGIC_NUM;
    flash_write_page(FLASH_SECTOR, FLASH_PAGE, (const uint32 *)&g_params, SAVE_LEN);
}

static void params_apply_record_config(void)
{
    path_recorder_set_min_distance(g_params.record_min_distance);
    path_recorder_set_min_interval_ms(g_params.record_min_interval_ms);
    path_recorder_set_max_speed_kph(g_params.record_max_speed_kph);
    path_recorder_set_min_satellites(g_params.record_min_satellites);
}

static void params_capture_record_config(void)
{
    const path_record_config_t *config = path_recorder_get_config();

    g_params.record_min_distance = config->min_record_distance;
    g_params.record_min_interval_ms = config->min_record_interval_ms;
    g_params.record_max_speed_kph = config->max_record_speed_kph;
    g_params.record_min_satellites = config->min_satellites;
}
/* 恢复默认 PID 与菜单参数，并同步到运行时缓存 */
static void params_set_default(void)
{
    g_params.pid_p = 1.2f;
    g_params.pid_i = 0.01f;
    g_params.pid_d = 0.5f;
    g_params.record_min_distance = MIN_RECORD_DISTANCE;
    g_params.record_min_interval_ms = MIN_RECORD_INTERVAL_MS;
    g_params.record_max_speed_kph = MAX_RECORD_SPEED_KPH;
    g_params.record_min_satellites = GPS_MIN_SATELLITES;
    g_params.threshold_mode = 0;
    g_params.threshold_val = 128;
    g_params.magic_code = MAGIC_NUM;
    pid_sync_runtime_param();
    params_apply_record_config();
}

/* 按步进调整参数值，并限制在给定最小值和最大值之间 */
static void adjust_param(float *ptr, float step, float min, float max, uint8_t is_add)
{
    if (is_add)
    {
        if (*ptr + step <= max)
        {
            *ptr += step;
        }
        else
        {
            *ptr = max;
        }
    }
    else
    {
        if (*ptr - step >= min)
        {
            *ptr -= step;
        }
        else
        {
            *ptr = min;
        }
    }
}

/* 上电时从 Flash 加载参数，若无效则回退到默认配置 */
static void Init_Load_Params(void)
{
    MyParams_t temp_read;

    flash_read_page(FLASH_SECTOR, FLASH_PAGE, (uint32 *)&temp_read, SAVE_LEN);

    if (params_are_valid(&temp_read))
    {
        g_params = temp_read;
    }
    else
    {
        params_set_default();
    }
    pid_sync_runtime_param();
    params_apply_record_config();
}

static void gps_clear_status_hint(void)
{
    gps_status_hint[0] = '\0';
}

static void gps_set_status_hint(const char *text)
{
    uint32 i = 0U;
    const char *source = (NULL != text) ? text : "";

    while (source[i] != '\0' && i < (sizeof(gps_status_hint) - 1U))
    {
        gps_status_hint[i] = source[i];
        i++;
    }
    gps_status_hint[i] = '\0';
}

static const char *gps_get_record_state_text(void)
{
    switch (path_recorder_get_state())
    {
        case PATH_STATE_RECORDING:
            return "REC";
        case PATH_STATE_COMPLETED:
            return "DONE";
        case PATH_STATE_IDLE:
        default:
            return "IDLE";
    }
}

static void gps_return_to_menu(void)
{
    gps_display_mode = GPS_VIEW_NONE;
    menu_request_redraw(1U);
}

/* 进入 GPS 详情页前清空输入状态，并请求整页刷新 */
static void gps_enter_view(gps_view_mode_t mode)
{
    gps_display_mode = mode;
    menu_drain_encoder_events();
    my_key_clear_state(MY_KEY_1);
    my_key_clear_state(MY_KEY_4);
    menu_reset_dynamic_region();
    menu_request_redraw(1U);
}

static void gps_exit_view(void)
{
    gps_display_mode = GPS_VIEW_NONE;
    menu_drain_encoder_events();
    menu_reset_dynamic_region();
    menu_request_redraw(1U);
}

static void gps_clear_current_track(void)
{
    path_state_enum state = path_recorder_get_state();

    path_recorder_clear();

    if ((state == PATH_STATE_RECORDING) && gps_menu_gnss_ready())
    {
        path_recorder_start();
        gps_set_status_hint("Track reset");
    }
    else
    {
        gps_set_status_hint("Track cleared");
    }

    path_display_init();
    menu_request_redraw(1U);
}

static void gps_draw_page_header(const char *title)
{
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    show_string_fit(10, 10, title);
    ips200_draw_line(0, 30, ips200_width_max - 1, 30, RGB565_GRAY);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

static void gps_draw_data_page(void)
{
    static uint32 last_refresh_ms = 0;
    char line0[64];
    char line1[64];
    char line2[64];
    char line3[64];
    char line4[64];
    uint32 now_ms = system_getval_ms();
    float total_distance = 0.0f;

    if (!menu_full_redraw && (now_ms - last_refresh_ms < 200U))
    {
        return;
    }

    last_refresh_ms = now_ms;
    if (menu_full_redraw)
    {
        ips200_fill_rect(0, 32, ips200_width_max - 1, ips200_height_max - 1, RGB565_BLACK);
        gps_draw_page_header("GPS Data");
        show_string_fit_width(5, (uint16)(ips200_height_max - 20U), (uint16)(ips200_width_max - 10U), "K1 LONG Exit");
    }

    path_recorder_get_stats(&total_distance, NULL, NULL);

    sprintf(line0, "Fix:%d Sat:%d Rec:%s", (int)gnss.state, (int)gnss.satellite_used, gps_get_record_state_text());
    sprintf(line1, "Lat:%.6f", gnss.latitude);
    sprintf(line2, "Lon:%.6f", gnss.longitude);
    sprintf(line3, "Spd:%.2f Dir:%.1f", gnss.speed, gnss.direction);
    if (gps_status_hint[0] != '\0')
    {
        sprintf(line4, "%s", gps_status_hint);
    }
    else
    {
        sprintf(line4, "Pts:%d Dist:%.1fm", path_recorder_get_point_count(), total_distance);
    }

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    show_string_fit_width_pad(10, 60, (uint16)(ips200_width_max - 20U), line0);
    show_string_fit_width_pad(10, 84, (uint16)(ips200_width_max - 20U), line1);
    show_string_fit_width_pad(10, 108, (uint16)(ips200_width_max - 20U), line2);
    show_string_fit_width_pad(10, 132, (uint16)(ips200_width_max - 20U), line3);
    show_string_fit_width_pad(10, 156, (uint16)(ips200_width_max - 20U), line4);
    menu_full_redraw = 0;
}

static void gps_draw_map_page(void)
{
    static uint32 last_refresh_ms = 0;
    char line0[64];
    char line1[64];
    char line2[64];
    uint16 map_x = 5U;
    uint16 map_y = 105U;
    uint16 map_w = (uint16)(ips200_width_max - 10U);
    uint16 map_h;
    uint32 now_ms = system_getval_ms();
    float total_distance = 0.0f;

    if (!menu_full_redraw && (now_ms - last_refresh_ms < 800U))
    {
        return;
    }

    last_refresh_ms = now_ms;
    if (menu_full_redraw)
    {
        ips200_fill_rect(0, 32, ips200_width_max - 1, ips200_height_max - 1, RGB565_BLACK);
        gps_draw_page_header("GPS Map");
        show_string_fit_width(5, (uint16)(ips200_height_max - 20U), (uint16)(ips200_width_max - 10U), "K1 Clear  LONG Exit");
    }

    path_recorder_get_stats(&total_distance, NULL, NULL);
    sprintf(line0, "Sat:%d Pts:%d Fix:%d", (int)gnss.satellite_used, path_recorder_get_point_count(), (int)gnss.state);
    if (gps_status_hint[0] != '\0')
    {
        sprintf(line1, "%s", gps_status_hint);
    }
    else
    {
        sprintf(line1, "Rec:%s Dist:%.1fm", gps_get_record_state_text(), total_distance);
    }
    sprintf(line2, "Spd:%.1fkm/h H:%.1fm", gnss.speed, gnss.height);

    if (ips200_height_max > 190U)
    {
        map_h = (uint16)(ips200_height_max - 190U);
    }
    else
    {
        map_h = (uint16)(ips200_height_max / 2U);
    }

    if (map_h < 60U)
    {
        map_h = 60U;
    }

    if ((uint32)map_y + (uint32)map_h > (uint32)(ips200_height_max - 25U))
    {
        map_h = (uint16)(ips200_height_max - map_y - 25U);
    }

    if (map_h < 60U)
    {
        map_h = 60U;
    }

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    show_string_fit_width_pad(10, 36, (uint16)(ips200_width_max - 20U), line0);
    show_string_fit_width_pad(10, 56, (uint16)(ips200_width_max - 20U), line1);
    show_string_fit_width_pad(10, 76, (uint16)(ips200_width_max - 20U), line2);

    path_display_set_area(map_x, map_y, map_w, map_h);
    path_display_draw_map(RGB565_CYAN);
    menu_full_redraw = 0;
}

static void gps_draw_raw_debug_page(void)
{
    static uint32 last_refresh_ms = 0;
    gnss_debug_info_t debug_info;
    char line0[64];
    char line1[64];
    char line2[64];
    char line3[96];
    char line4[96];
    char line5[96];
    uint32 now_ms = system_getval_ms();

    if (!menu_full_redraw && (now_ms - last_refresh_ms < 300U))
    {
        return;
    }

    last_refresh_ms = now_ms;
    if (menu_full_redraw)
    {
        ips200_fill_rect(0, 32, ips200_width_max - 1, ips200_height_max - 1, RGB565_BLACK);
        gps_draw_page_header("GPS RAW/DEBUG");
        show_string_fit_width(5, (uint16)(ips200_height_max - 20U), (uint16)(ips200_width_max - 10U), "K1 LONG Exit");
    }

    gnss_get_debug_info(&debug_info);
    sprintf(line0, "Fix:%d Sat:%d Flag:%d Err:%lu", (int)gnss.state, (int)gnss.satellite_used, (int)debug_info.rx_flag, (unsigned long)debug_info.parse_error_count);
    sprintf(line1, "Init:%d R:%lu G:%lu T:%lu", (int)debug_info.initialized, (unsigned long)debug_info.rmc_frame_count, (unsigned long)debug_info.gga_frame_count, (unsigned long)debug_info.ths_frame_count);
    sprintf(line2, "St R:%u G:%u T:%u", debug_info.rmc_state, debug_info.gga_state, debug_info.ths_state);
    sprintf(line3, "RMC:%s", (debug_info.rmc_sentence[0] != '\0') ? debug_info.rmc_sentence : "<none>");
    sprintf(line4, "GGA:%s", (debug_info.gga_sentence[0] != '\0') ? debug_info.gga_sentence : "<none>");
    sprintf(line5, "THS:%s", (debug_info.ths_sentence[0] != '\0') ? debug_info.ths_sentence : "<none>");

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    show_string_fit_width_pad(10, 40, (uint16)(ips200_width_max - 20U), line0);
    show_string_fit_width_pad(10, 60, (uint16)(ips200_width_max - 20U), line1);
    show_string_fit_width_pad(10, 80, (uint16)(ips200_width_max - 20U), line2);
    show_string_fit_width_pad(10, 112, (uint16)(ips200_width_max - 20U), line3);
    show_string_fit_width_pad(10, 136, (uint16)(ips200_width_max - 20U), line4);
    show_string_fit_width_pad(10, 160, (uint16)(ips200_width_max - 20U), line5);
    menu_full_redraw = 0;
}

static void record_param_enter_view(record_param_view_mode_t mode)
{
    record_param_view_mode = mode;
    my_key_clear_state(MY_KEY_1);
    menu_request_redraw(1U);
}

static void record_param_exit_view(void)
{
    if (RECORD_PARAM_VIEW_NONE == record_param_view_mode)
    {
        return;
    }

    record_param_view_mode = RECORD_PARAM_VIEW_NONE;
    params_capture_record_config();
    params_save_to_flash();
    menu_set_status_message("Record saved", MENU_STATUS_SHOW_MS);
    menu_sync_gps_record_param_items();
    menu_request_redraw(1U);
}

static float record_param_get_distance_step(void)
{
    return record_distance_steps[record_distance_step_index];
}

static uint16 record_param_get_interval_step(void)
{
    return record_interval_steps[record_interval_step_index];
}

static uint8 record_param_get_sat_step(void)
{
    return record_sat_steps[record_sat_step_index];
}

static void record_param_cycle_step(void)
{
    if (RECORD_PARAM_VIEW_DISTANCE == record_param_view_mode)
    {
        record_distance_step_index = (uint8)((record_distance_step_index + 1U) % (sizeof(record_distance_steps) / sizeof(record_distance_steps[0])));
    }
    else if (RECORD_PARAM_VIEW_INTERVAL == record_param_view_mode)
    {
        record_interval_step_index = (uint8)((record_interval_step_index + 1U) % (sizeof(record_interval_steps) / sizeof(record_interval_steps[0])));
    }
    else if (RECORD_PARAM_VIEW_MIN_SAT == record_param_view_mode)
    {
        record_sat_step_index = (uint8)((record_sat_step_index + 1U) % (sizeof(record_sat_steps) / sizeof(record_sat_steps[0])));
    }
}

static void record_param_apply_encoder_adjustment(void)
{
    const path_record_config_t *config = path_recorder_get_config();

    if (RECORD_PARAM_VIEW_DISTANCE == record_param_view_mode)
    {
        float value = config->min_record_distance;
        adjust_param(&value, record_param_get_distance_step(), PARAM_RECORD_DISTANCE_MIN, PARAM_RECORD_DISTANCE_MAX, (switch_encoder_change_num < 0) ? 1U : 0U);
        path_recorder_set_min_distance(value);
    }
    else if (RECORD_PARAM_VIEW_INTERVAL == record_param_view_mode)
    {
        uint32 value = config->min_record_interval_ms;
        uint16 step = record_param_get_interval_step();

        if (switch_encoder_change_num < 0)
        {
            value += step;
        }
        else if (value > step)
        {
            value -= step;
        }
        else
        {
            value = PARAM_RECORD_INTERVAL_MIN;
        }

        path_recorder_set_min_interval_ms(value);
    }
    else if (RECORD_PARAM_VIEW_MIN_SAT == record_param_view_mode)
    {
        uint32 value = config->min_satellites;
        uint8 step = record_param_get_sat_step();

        if (switch_encoder_change_num < 0)
        {
            value += step;
        }
        else if (value > step)
        {
            value -= step;
        }
        else
        {
            value = PARAM_RECORD_SAT_MIN;
        }

        path_recorder_set_min_satellites((uint8)value);
    }

    menu_sync_gps_record_param_items();
}

static void record_param_draw_edit_page(const char *title, const char *value_text, const char *step_text)
{
    static record_param_view_mode_t last_draw_mode = RECORD_PARAM_VIEW_NONE;
    static char last_value_text[32] = "";
    static char last_step_text[32] = "";

    if (!menu_full_redraw &&
        last_draw_mode == record_param_view_mode &&
        0 == strcmp(last_value_text, value_text) &&
        0 == strcmp(last_step_text, step_text))
    {
        return;
    }

    if (menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        show_string_fit(10, 10, title);
        ips200_draw_line(0, 30, ips200_width_max - 1, 30, RGB565_GRAY);
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        show_string_fit_width_pad(10, 40, (uint16)(ips200_width_max - 20U), "ENC: Adjust record param");
        show_string_fit_width_pad(10, 56, (uint16)(ips200_width_max - 20U), "K1: Change step   LONG: Back");
        show_string_fit_width_pad(18, 88, (uint16)(ips200_width_max - 36U), "Current Value");
        show_string_fit_width_pad(18, 144, (uint16)(ips200_width_max - 36U), "Step Size");
    }

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    show_string_fit_width_pad(18, 110, (uint16)(ips200_width_max - 36U), value_text);

    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    show_string_fit_width_pad(18, 166, (uint16)(ips200_width_max - 36U), step_text);

    strncpy(last_value_text, value_text, sizeof(last_value_text) - 1U);
    last_value_text[sizeof(last_value_text) - 1U] = '\0';
    strncpy(last_step_text, step_text, sizeof(last_step_text) - 1U);
    last_step_text[sizeof(last_step_text) - 1U] = '\0';
    last_draw_mode = record_param_view_mode;
    menu_full_redraw = 0U;
}

static void record_param_draw_active_view(void)
{
    const path_record_config_t *config = path_recorder_get_config();
    char value_text[32];
    char step_text[32];

    if (RECORD_PARAM_VIEW_DISTANCE == record_param_view_mode)
    {
        sprintf(value_text, "%.2f m", config->min_record_distance);
        sprintf(step_text, "%.2f m", record_param_get_distance_step());
        record_param_draw_edit_page("Point Distance", value_text, step_text);
    }
    else if (RECORD_PARAM_VIEW_INTERVAL == record_param_view_mode)
    {
        sprintf(value_text, "%lu ms", (unsigned long)config->min_record_interval_ms);
        sprintf(step_text, "%u ms", record_param_get_interval_step());
        record_param_draw_edit_page("Point Interval", value_text, step_text);
    }
    else if (RECORD_PARAM_VIEW_MIN_SAT == record_param_view_mode)
    {
        sprintf(value_text, "%u", config->min_satellites);
        sprintf(step_text, "%u", record_param_get_sat_step());
        record_param_draw_edit_page("Min Satellites", value_text, step_text);
    }
}

static void gps_action_record_param_distance(void)
{
    record_param_enter_view(RECORD_PARAM_VIEW_DISTANCE);
}

static void gps_action_record_param_interval(void)
{
    record_param_enter_view(RECORD_PARAM_VIEW_INTERVAL);
}

static void gps_action_record_param_min_sat(void)
{
    record_param_enter_view(RECORD_PARAM_VIEW_MIN_SAT);
}

static void pid_sync_runtime_param(void)
{
    menu_pid_param_cache.kp = g_params.pid_p;
    menu_pid_param_cache.ki = g_params.pid_i;
    menu_pid_param_cache.kd = g_params.pid_d;
    menu_pid_param_cache.integral_limit = 500.0f;
    menu_pid_param_cache.output_limit = 1000.0f;
    pid_set_param(&pid_preview_controller, &menu_pid_param_cache);
}

static void pid_save_if_dirty(void)
{
    if (!pid_param_dirty)
    {
        return;
    }

    params_save_to_flash();
    pid_param_dirty = 0U;
    menu_set_status_message("PID saved", MENU_STATUS_SHOW_MS);
}

static void pid_reset_preview_state(void)
{
    pid_reset(&pid_preview_controller);
    pid_preview_feedback = 0.0f;
    pid_preview_output = 0.0f;
}

static void pid_enter_view(pid_view_mode_t mode)
{
    pid_display_mode = mode;
    pid_sync_runtime_param();
    pid_reset_preview_state();
    my_key_clear_state(MY_KEY_1);
    menu_request_redraw(1U);
}

static void pid_exit_view(void)
{
    if ((PID_VIEW_NONE != pid_display_mode) && (pid_display_mode <= PID_VIEW_EDIT_KD) && pid_param_dirty)
    {
        pid_save_if_dirty();
    }

    pid_display_mode = PID_VIEW_NONE;
    menu_request_redraw(1U);
}

static void pid_apply_encoder_adjustment(void)
{
    float step = pid_get_current_step(pid_display_mode);
    uint8 updated = 0U;

    if (PID_VIEW_EDIT_KP == pid_display_mode)
    {
        adjust_param(&g_params.pid_p, step, PARAM_PID_P_MIN, PARAM_PID_P_MAX, (switch_encoder_change_num < 0) ? 1U : 0U);
        updated = 1U;
    }
    else if (PID_VIEW_EDIT_KI == pid_display_mode)
    {
        adjust_param(&g_params.pid_i, step, PARAM_PID_I_MIN, PARAM_PID_I_MAX, (switch_encoder_change_num < 0) ? 1U : 0U);
        updated = 1U;
    }
    else if (PID_VIEW_EDIT_KD == pid_display_mode)
    {
        adjust_param(&g_params.pid_d, step, PARAM_PID_D_MIN, PARAM_PID_D_MAX, (switch_encoder_change_num < 0) ? 1U : 0U);
        updated = 1U;
    }

    if (updated)
    {
        pid_param_dirty = 1U;
        pid_sync_runtime_param();
    }
}

static void pid_draw_active_view(void)
{
    if (PID_VIEW_EDIT_KP == pid_display_mode)
    {
        pid_draw_edit_page("Edit Kp", g_params.pid_p, pid_get_current_step(PID_VIEW_EDIT_KP));
    }
    else if (PID_VIEW_EDIT_KI == pid_display_mode)
    {
        pid_draw_edit_page("Edit Ki", g_params.pid_i, pid_get_current_step(PID_VIEW_EDIT_KI));
    }
    else if (PID_VIEW_EDIT_KD == pid_display_mode)
    {
        pid_draw_edit_page("Edit Kd", g_params.pid_d, pid_get_current_step(PID_VIEW_EDIT_KD));
    }
    else if (PID_VIEW_PREVIEW == pid_display_mode)
    {
        pid_draw_preview_page();
    }
}

static void pid_draw_edit_page(const char *title, float value, float step)
{
    char value_line[32];
    char step_line[32];
    static pid_view_mode_t last_draw_mode = PID_VIEW_NONE;
    static float last_draw_value = -1.0f;
    static float last_draw_step = -1.0f;

    if (!menu_full_redraw &&
        last_draw_mode == pid_display_mode &&
        last_draw_value == value &&
        last_draw_step == step)
    {
        return;
    }

    if (menu_full_redraw)
    {
        ips200_full(RGB565_BLACK);
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        show_string_fit(10, 10, title);
        ips200_draw_line(0, 30, ips200_width_max - 1, 30, RGB565_GRAY);
        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        show_string_fit_width_pad(10, 40, (uint16)(ips200_width_max - 20U), "ENC: Adjust parameter");
        show_string_fit_width_pad(10, 56, (uint16)(ips200_width_max - 20U), "K1: Change step   LONG: Exit");
        show_string_fit_width_pad(18, 82, (uint16)(ips200_width_max - 36U), "Current Value");
        show_string_fit_width_pad(18, 138, (uint16)(ips200_width_max - 36U), "Step Size");
        show_string_fit_width_pad(18, 194, (uint16)(ips200_width_max - 36U), "Rotate encoder to tune");
        show_string_fit_width_pad(18, 210, (uint16)(ips200_width_max - 36U), "Exit auto-saves PID");
    }

    sprintf(value_line, "%.4f", value);
    sprintf(step_line, "%.4f", step);

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    show_string_fit_width_pad(18, 104, (uint16)(ips200_width_max - 36U), value_line);

    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    show_string_fit_width_pad(18, 160, (uint16)(ips200_width_max - 36U), step_line);

    last_draw_mode = pid_display_mode;
    last_draw_value = value;
    last_draw_step = step;
    menu_full_redraw = 0;
}

static void pid_draw_preview_page(void)
{
    static uint32 last_refresh_ms = 0;
    char line0[64];
    char line1[64];
    char line2[64];
    char line3[64];
    char line4[64];
    uint32 now_ms = system_getval_ms();
    float error;

    if (!menu_full_redraw && (now_ms - last_refresh_ms < 120U))
    {
        return;
    }

    last_refresh_ms = now_ms;

    if (menu_full_redraw)
    {
        ips200_fill_rect(0, 32, ips200_width_max - 1, ips200_height_max - 1, RGB565_BLACK);
        gps_draw_page_header("PID Preview");
        show_string_fit_width(5, (uint16)(ips200_height_max - 20U), (uint16)(ips200_width_max - 10U), "K1 LONG Exit");
    }

    pid_preview_output = pid_calculate(&pid_preview_controller, pid_preview_target, pid_preview_feedback);
    pid_preview_feedback += pid_preview_output * 0.02f;
    pid_preview_feedback *= 0.995f;
    error = pid_preview_target - pid_preview_feedback;

    sprintf(line0, "Kp:%.3f Ki:%.3f", g_params.pid_p, g_params.pid_i);
    sprintf(line1, "Kd:%.3f Target:%.1f", g_params.pid_d, pid_preview_target);
    sprintf(line2, "Feedback:%.2f", pid_preview_feedback);
    sprintf(line3, "Error:%.2f Output:%.2f", error, pid_preview_output);
    sprintf(line4, "Adjust gains in PID menu");

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    show_string_fit_width_pad(10, 56, (uint16)(ips200_width_max - 20U), line0);
    show_string_fit_width_pad(10, 80, (uint16)(ips200_width_max - 20U), line1);
    show_string_fit_width_pad(10, 104, (uint16)(ips200_width_max - 20U), line2);
    show_string_fit_width_pad(10, 128, (uint16)(ips200_width_max - 20U), line3);
    show_string_fit_width_pad(10, 152, (uint16)(ips200_width_max - 20U), line4);
    menu_full_redraw = 0;
}

static float pid_get_current_step(uint8 mode)
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

static void pid_cycle_step(uint8 mode)
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

static void pid_action_edit_kp(void)
{
    pid_enter_view(PID_VIEW_EDIT_KP);
}

static void pid_action_edit_ki(void)
{
    pid_enter_view(PID_VIEW_EDIT_KI);
}

static void pid_action_edit_kd(void)
{
    pid_enter_view(PID_VIEW_EDIT_KD);
}

static void pid_action_preview(void)
{
    pid_enter_view(PID_VIEW_PREVIEW);
}

static void pid_action_reset(void)
{
    g_params.pid_p = 1.2f;
    g_params.pid_i = 0.01f;
    g_params.pid_d = 0.5f;
    pid_param_dirty = 1U;
    pid_sync_runtime_param();
    pid_reset_preview_state();
    pid_save_if_dirty();
    menu_full_redraw = 1;
}

/* 打开 GPS 数据页，显示当前定位与轨迹统计信息 */
static void gps_action_display_data(void)
{
    gps_clear_status_hint();
    gps_enter_view(GPS_VIEW_DATA);
}

/* 根据当前记录状态启动或停止轨迹记录，并更新状态提示 */
static void gps_action_toggle_record(void)
{
    path_state_enum state = path_recorder_get_state();

    if (state == PATH_STATE_RECORDING)
    {
        path_recorder_stop();
        gps_set_status_hint("Track stopped");
    }
    else
    {
        if (gps_menu_gnss_ready())
        {
            if (path_recorder_start_new())
            {
                gps_set_status_hint("Track recording");
            }
            else
            {
                gps_set_status_hint("Start failed");
            }
        }
        else
        {
            gps_set_status_hint("GPS not ready");
        }
    }

    gps_return_to_menu();
}

/* 初始化轨迹显示区域并进入 GPS 地图页面 */
static void gps_action_map(void)
{
    gps_clear_status_hint();
    path_display_init();
    gps_enter_view(GPS_VIEW_MAP);
}

static void gps_action_raw_debug(void)
{
    gps_clear_status_hint();
    gps_enter_view(GPS_VIEW_RAW_DEBUG);
}

/* ======================== WiFi 功能实现 ======================== */

/*
 * wifi_do_connect —— 连接指定预存WiFi
 * 使用 wifi_spi_init() 完成完整的初始化+连接流程
 */
static void wifi_do_connect(int index)
{
    int i;
    const char *src;

    if (index < 0 || index >= WIFI_PRESET_COUNT)
    {
        wifi_connect_result = 1U;
        return;
    }

    /* wifi_spi_init 内部会执行 SPI 初始化 + 复位 + 获取版本 + 连接 */
    wifi_connect_result = wifi_spi_init(
        (char *)wifi_presets[index].ssid,
        (char *)wifi_presets[index].password);

    if (0U == wifi_connect_result)
    {
        wifi_connected = 1U;
        wifi_initialized = 1U;
        /* 保存已连接的SSID */
        i = 0;
        src = wifi_presets[index].ssid;
        while (src[i] != '\0' && i < (int)sizeof(wifi_connected_ssid) - 1)
        {
            wifi_connected_ssid[i] = src[i];
            i++;
        }
        wifi_connected_ssid[i] = '\0';
    }
    else
    {
        wifi_connected = 0U;
        wifi_connected_ssid[0] = '\0';
    }
}

/*
 * wifi_do_test —— 测试WiFi模块是否在线
 * 通过 wifi_spi_init() 触发底层的 get_version / get_mac 查询
 * wifi_spi_init 内部流程：SPI初始化 → 硬件复位 → 读版本 → 读MAC → 连WiFi
 * 即使WiFi连接失败，只要模块在线，版本和MAC就能读到
 */
static void wifi_do_test(void)
{
    /* 清空旧数据，确保结果可靠 */
    wifi_spi_version[0] = '\0';
    wifi_spi_mac_addr[0] = '\0';
    wifi_spi_ip_addr_port[0] = '\0';
    wifi_connected = 0U;
    wifi_initialized = 0U;
    wifi_connected_ssid[0] = '\0';
    wifi_test_stage_ver = 0xFFU;
    wifi_test_stage_mac = 0xFFU;
    wifi_test_stage_wifi = 0xFFU;

    /*
     * 当前底层驱动不再提供 wifi_spi_probe，
     * 这里用 wifi_spi_init(NULL, NULL) 只做模块初始化 + 读取版本/MAC，
     * 不主动连接热点。
     */
    wifi_spi_init(NULL, NULL);

    wifi_test_stage_ver = wifi_spi_diag_stage_version;
    wifi_test_stage_mac = wifi_spi_diag_stage_mac;
    wifi_test_stage_wifi = wifi_spi_diag_stage_wifi;
    wifi_test_ver_ok = (wifi_spi_version[0] != '\0') ? 1U : 0U;
    wifi_test_mac_ok = (wifi_spi_mac_addr[0] != '\0') ? 1U : 0U;
    wifi_test_ip_ok  = (wifi_spi_ip_addr_port[0] != '\0') ? 1U : 0U;
    wifi_initialized = (wifi_test_ver_ok || wifi_test_mac_ok) ? 1U : 0U;
}

/* ---------- WiFi 视图管理 ---------- */

static void wifi_enter_view(wifi_view_mode_t mode)
{
    wifi_view_mode = mode;
    wifi_list_full_redraw = 1U;
    wifi_list_last_sel = -1;
    ips200_full(RGB565_BLACK);
    menu_drain_encoder_events();
    my_key_clear_all_state();
    menu_needs_update = 0U;
    menu_footer_needs_update = 0U;
}

static void wifi_exit_view(void)
{
    wifi_view_mode = WIFI_VIEW_NONE;
    menu_drain_encoder_events();
    my_key_clear_all_state();
    menu_request_redraw(1U);
}

/* ---------- WiFi 绘制函数 ---------- */

/* 绘制单个WiFi列表项（selected=1 高亮，=0 普通） */
static void wifi_draw_list_item(int index, uint8 selected)
{
    char buf[48];
    uint16 y;

    if (index < 0 || index >= WIFI_PRESET_COUNT) return;

    y = (uint16)(MENU_ITEM_START_Y + (uint16)index * MENU_ITEM_HEIGHT);

    if (selected)
    {
        ips200_fill_rect(0, y, ips200_width_max - 1, (uint16)(y + MENU_ITEM_TEXT_HEIGHT - 1), RGB565_WHITE);
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    }
    else
    {
        ips200_fill_rect(0, y, ips200_width_max - 1, (uint16)(y + MENU_ITEM_TEXT_HEIGHT - 1), RGB565_BLACK);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    }

    sprintf(buf, "%d. %s", index + 1, wifi_presets[index].label);
    show_string_fit(4, y, buf);
}

/* 全量绘制WiFi列表页（仅在首次进入时调用） */
static void wifi_draw_list_full(void)
{
    int i;
    uint16 footer_y;

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0, 2, "WiFi List");
    ips200_draw_line(0, MENU_TITLE_HEIGHT - 2, ips200_width_max - 1, MENU_TITLE_HEIGHT - 2, RGB565_GRAY);

    for (i = 0; i < WIFI_PRESET_COUNT; i++)
    {
        wifi_draw_list_item(i, (uint8)(i == wifi_list_selection));
    }

    /* 底部提示 */
    footer_y = (uint16)(ips200_height_max - MENU_FOOTER_HEIGHT);
    ips200_fill_rect(0, footer_y, ips200_width_max - 1, ips200_height_max - 1, RGB565_BLACK);
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    ips200_show_string(0, (uint16)(footer_y + 2), "ENC:Sel K1:Connect LONG:BK");

    wifi_list_last_sel = wifi_list_selection;
    wifi_list_full_redraw = 0U;
}

static void wifi_draw_connecting_page(void)
{
    char buf[48];

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0, 2, "WiFi Connect");
    ips200_draw_line(0, MENU_TITLE_HEIGHT - 2, ips200_width_max - 1, MENU_TITLE_HEIGHT - 2, RGB565_GRAY);

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    sprintf(buf, "Connecting to:");
    ips200_show_string(4, 50, buf);

    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    show_string_fit(4, 70, wifi_presets[wifi_list_selection].ssid);

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(4, 100, "Please wait...");
}

static void wifi_draw_status_page(void)
{
    char buf[64];
    uint16 footer_y;

    ips200_full(RGB565_BLACK);
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0, 2, "Connect Result");
    ips200_draw_line(0, MENU_TITLE_HEIGHT - 2, ips200_width_max - 1, MENU_TITLE_HEIGHT - 2, RGB565_GRAY);

    if (0U == wifi_connect_result)
    {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        ips200_show_string(4, 44, "Connected OK!");

        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        sprintf(buf, "SSID: %s", wifi_connected_ssid);
        show_string_fit(4, 68, buf);

        if (wifi_spi_ip_addr_port[0] != '\0')
        {
            sprintf(buf, "IP: %s", wifi_spi_ip_addr_port);
            show_string_fit(4, 88, buf);
        }

        if (wifi_spi_mac_addr[0] != '\0')
        {
            sprintf(buf, "MAC: %s", wifi_spi_mac_addr);
            show_string_fit(4, 108, buf);
        }
    }
    else
    {
        ips200_set_color(RGB565_RED, RGB565_BLACK);
        ips200_show_string(4, 44, "Connect FAILED!");

        ips200_set_color(RGB565_GRAY, RGB565_BLACK);
        sprintf(buf, "SSID: %s", wifi_presets[wifi_list_selection].ssid);
        show_string_fit(4, 68, buf);
        ips200_show_string(4, 92, "Check module/password");
    }

    /* 底部提示 */
    footer_y = (uint16)(ips200_height_max - MENU_FOOTER_HEIGHT);
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    ips200_show_string(0, (uint16)(footer_y + 2), "LONG K1: Back");
}

static void wifi_draw_test_page(void)
{
    char buf[64];
    uint8 all_ok;
    uint16 footer_y;

    ips200_full(RGB565_BLACK);
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0, 2, "Signal Test");
    ips200_draw_line(0, MENU_TITLE_HEIGHT - 2, ips200_width_max - 1, MENU_TITLE_HEIGHT - 2, RGB565_GRAY);

    /* 固件版本 */
    ips200_set_color(wifi_test_ver_ok ? RGB565_GREEN : RGB565_RED, RGB565_BLACK);
    sprintf(buf, "FW:  %s", wifi_test_ver_ok ? wifi_spi_version : "N/A");
    show_string_fit(4, 44, buf);

    /* MAC 地址 */
    ips200_set_color(wifi_test_mac_ok ? RGB565_GREEN : RGB565_RED, RGB565_BLACK);
    sprintf(buf, "MAC: %s", wifi_test_mac_ok ? wifi_spi_mac_addr : "N/A");
    show_string_fit(4, 64, buf);

    /* IP 地址 */
    ips200_set_color(wifi_test_ip_ok ? RGB565_GREEN : RGB565_RED, RGB565_BLACK);
    sprintf(buf, "IP:  %s", wifi_test_ip_ok ? wifi_spi_ip_addr_port : "N/A");
    show_string_fit(4, 84, buf);

    /* WiFi 连接状态 */
    ips200_set_color(wifi_connected ? RGB565_GREEN : RGB565_GRAY, RGB565_BLACK);
    sprintf(buf, "WiFi: %s", wifi_connected ? wifi_connected_ssid : "Not connected");
    show_string_fit(4, 108, buf);

    /* 综合判定 */
    all_ok = wifi_test_ver_ok & wifi_test_mac_ok;
    ips200_set_color(all_ok ? RGB565_GREEN : RGB565_RED, RGB565_BLACK);
    ips200_show_string(4, 136, all_ok ? "Module: OK" : "Module: ERROR");

    /* 底部提示 */
    footer_y = (uint16)(ips200_height_max - MENU_FOOTER_HEIGHT);
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    ips200_show_string(0, (uint16)(footer_y + 2), "LONG K1: Back");
}

/* ---------- WiFi 视图输入处理 ---------- */

static uint8 menu_handle_wifi_view(void)
{
    if (WIFI_VIEW_NONE == wifi_view_mode)
    {
        return 0U;
    }

    /* 所有WiFi视图：长按K1退出 */
    if (my_key_get_state(MY_KEY_1) == MY_KEY_LONG_PRESS)
    {
        my_key_clear_state(MY_KEY_1);
        wifi_exit_view();
        return 1U;
    }

    switch (wifi_view_mode)
    {
        case WIFI_VIEW_LIST:
        {
            /* 首次进入：全量绘制 */
            if (wifi_list_full_redraw)
            {
                wifi_draw_list_full();
            }

            /* 编码器切换选中项 */
            if (If_Switch_Encoder_Change())
            {
                wifi_list_selection -= switch_encoder_change_num;
                while (wifi_list_selection >= WIFI_PRESET_COUNT)
                    wifi_list_selection -= WIFI_PRESET_COUNT;
                while (wifi_list_selection < 0)
                    wifi_list_selection += WIFI_PRESET_COUNT;
            }

            /* 选中项变化时：只重绘旧项和新项（局部刷新） */
            if (wifi_list_selection != wifi_list_last_sel)
            {
                if (wifi_list_last_sel >= 0)
                {
                    wifi_draw_list_item(wifi_list_last_sel, 0U);
                }
                wifi_draw_list_item(wifi_list_selection, 1U);
                wifi_list_last_sel = wifi_list_selection;
            }

            /* 短按K1：连接选中的WiFi */
            if (my_key_get_state(MY_KEY_1) == MY_KEY_SHORT_PRESS)
            {
                my_key_clear_state(MY_KEY_1);

                /* 先画连接中页面 */
                wifi_view_mode = WIFI_VIEW_CONNECTING;
                wifi_draw_connecting_page();

                /* 执行连接（阻塞等待） */
                wifi_do_connect(wifi_list_selection);

                /* 跳转到结果页面（画一次就够） */
                wifi_view_mode = WIFI_VIEW_STATUS;
                wifi_draw_status_page();
                my_key_clear_all_state();
                menu_drain_encoder_events();
                return 1U;
            }

            break;
        }

        case WIFI_VIEW_CONNECTING:
        case WIFI_VIEW_STATUS:
        case WIFI_VIEW_TEST:
        {
            /* 静态页面，已在进入时绘制完毕，仅等待长按K1退出 */
            break;
        }

        default:
            break;
    }

    return 1U;
}

/* ---------- WiFi 菜单动作函数 ---------- */

static void wifi_action_list(void)
{
    wifi_list_selection = 0;
    wifi_enter_view(WIFI_VIEW_LIST);
}

static void wifi_action_test(void)
{
    wifi_enter_view(WIFI_VIEW_TEST);

    /* 先显示 "Testing..." */
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(4, 60, "Testing module...");

    wifi_do_test();
    wifi_draw_test_page();
}

/* ---------- WiFi 菜单结构 ---------- */

static MenuItem wifi_items[] = {
    {"1. WiFi List",   wifi_action_list, NULL},
    {"2. Signal Test", wifi_action_test, NULL},
};

static MenuPage wifi_menu = {
    "WiFi",
    wifi_items,
    sizeof(wifi_items) / sizeof(MenuItem),
    NULL
};

/* ============================================================== */

static MenuItem gps_record_param_items[] = {
    {gps_record_distance_label, gps_action_record_param_distance, NULL},
    {gps_record_interval_label, gps_action_record_param_interval, NULL},
    {gps_record_sat_label, gps_action_record_param_min_sat, NULL},
};

static MenuPage gps_record_param_menu = {
    "Record Params",
    gps_record_param_items,
    sizeof(gps_record_param_items) / sizeof(MenuItem),
    NULL
};

static MenuItem gps_items[] = {
    {"1. GPS Data", gps_action_display_data, NULL},
    {gps_record_menu_label, gps_action_toggle_record, NULL},
    {"3. Track Map", gps_action_map, NULL},
    {"4. Record Params", NULL, &gps_record_param_menu},
    {"5. GPS RAW/DEBUG", gps_action_raw_debug, NULL},
};

static MenuPage gps_menu = {
    "GPS",
    gps_items,
    sizeof(gps_items) / sizeof(MenuItem),
    NULL
};

static MenuItem threshold_algo_items[] = {
    {"1. Manual", NULL, NULL},
    {"2. Otsu", NULL, NULL},
    {"3. Kittler", NULL, NULL},
    {"4. Sauvola", NULL, NULL},
};

static MenuPage threshold_algo_menu = {
    "Threshold Mode",
    threshold_algo_items,
    sizeof(threshold_algo_items) / sizeof(MenuItem),
    NULL
};

static MenuItem camera_items[] = {
    {"1. Preview", NULL, NULL},
    {"2. Threshold", NULL, NULL},
    {"3. Color Preview", NULL, NULL},
    {"4. Threshold Mode", NULL, &threshold_algo_menu},
    {"5. Template Manage", NULL, NULL},
};

static MenuPage camera_menu = {
    "Camera",
    camera_items,
    sizeof(camera_items) / sizeof(MenuItem),
    NULL
};

static MenuItem pid_items[] = {
    {"1. Edit Kp", pid_action_edit_kp, NULL},
    {"2. Edit Ki", pid_action_edit_ki, NULL},
    {"3. Edit Kd", pid_action_edit_kd, NULL},
    {"4. Preview", pid_action_preview, NULL},
    {"5. Reset PID", pid_action_reset, NULL},
};

static MenuPage pid_menu = {
    "PID",
    pid_items,
    sizeof(pid_items) / sizeof(MenuItem),
    NULL
};

static MenuItem main_items[] = {
    {"1. GPS", NULL, &gps_menu},
    {"2. Camera", NULL, &camera_menu},
    {"3. PID", NULL, &pid_menu},
    {"4. WiFi", NULL, &wifi_menu},
};

static MenuPage main_menu = {
    "Jx116 Menu",
    main_items,
    sizeof(main_items) / sizeof(MenuItem),
    NULL
};

static void ips200_fill_rect(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, uint16 color)
{
    if (x_start >= ips200_width_max) x_start = ips200_width_max - 1;
    if (x_end >= ips200_width_max) x_end = ips200_width_max - 1;
    if (y_start >= ips200_height_max) y_start = ips200_height_max - 1;
    if (y_end >= ips200_height_max) y_end = ips200_height_max - 1;
    if (x_start > x_end) { uint16 t = x_start; x_start = x_end; x_end = t; }
    if (y_start > y_end) { uint16 t = y_start; y_start = y_end; y_end = t; }

    for (uint16 y = y_start; y <= y_end; y++)
    {
        ips200_draw_line(x_start, y, x_end, y, color);
    }
}

static void show_string_fit(uint16 x, uint16 y, const char *s)
{
    int max_chars = (int)((ips200_width_max - 1 - x) / 8);
    if (max_chars <= 0) return;

    char buf[64];
    int i = 0;
    while (i < max_chars && s[i] != '\0' && i < (int)sizeof(buf) - 1)
    {
        buf[i] = s[i];
        i++;
    }
    buf[i] = '\0';
    ips200_show_string(x, y, buf);
}

static void show_string_fit_width(uint16 x, uint16 y, uint16 max_width, const char *s)
{
    int max_chars;
    char buf[64];
    int i = 0;

    if (0U == max_width)
    {
        return;
    }

    max_chars = (int)(max_width / 8U);
    if (max_chars <= 0)
    {
        return;
    }

    while (i < max_chars && s[i] != '\0' && i < (int)sizeof(buf) - 1)
    {
        buf[i] = s[i];
        i++;
    }
    buf[i] = '\0';
    ips200_show_string(x, y, buf);
}

static void show_string_fit_width_pad(uint16 x, uint16 y, uint16 max_width, const char *s)
{
    int max_chars;
    char buf[64];
    int i = 0;

    if (0U == max_width)
    {
        return;
    }

    max_chars = (int)(max_width / 8U);
    if (max_chars <= 0)
    {
        return;
    }

    while (i < max_chars && s[i] != '\0' && i < (int)sizeof(buf) - 1)
    {
        buf[i] = s[i];
        i++;
    }

    while (i < max_chars && i < (int)sizeof(buf) - 1)
    {
        buf[i] = ' ';
        i++;
    }

    buf[i] = '\0';
    ips200_show_string(x, y, buf);
}

static void menu_set_status_message(const char *text, uint32 duration_ms)
{
    uint32 i = 0U;

    menu_status_message[0] = '\0';
    menu_status_expire_ms = 0U;

    if (NULL == text || '\0' == text[0])
    {
        menu_footer_needs_update = 1U;
        return;
    }

    while (text[i] != '\0' && i < (sizeof(menu_status_message) - 1U))
    {
        menu_status_message[i] = text[i];
        i++;
    }
    menu_status_message[i] = '\0';
    menu_status_expire_ms = system_getval_ms() + duration_ms;
    menu_footer_needs_update = 1U;
}

static void menu_draw_footer(void)
{
    const char *footer_text = "ENC:Move K1:OK/LONG:BK";
    uint16 text_color = RGB565_GRAY;

    if ('\0' != menu_status_message[0])
    {
        footer_text = menu_status_message;
        text_color = RGB565_YELLOW;
    }

    ips200_fill_rect(0U,
                     (uint16)(ips200_height_max - 20U),
                     (uint16)(ips200_width_max - 1U),
                     (uint16)(ips200_height_max - 1U),
                     RGB565_BLACK);
    ips200_set_color(text_color, RGB565_BLACK);
    show_string_fit_width_pad(5U,
                              (uint16)(ips200_height_max - 20U),
                              (uint16)(ips200_width_max - 10U),
                              footer_text);
    menu_footer_needs_update = 0U;
}

static void menu_drain_encoder_events(void)
{
    while (If_Switch_Encoder_Change())
    {
    }

    switch_encoder_change_num = 0;
    switch_encode_bring_flag = 0U;
    switch_encode_change_get_buff_flag = 0U;
}

static void menu_reset_dynamic_region(void)
{
    menu_set_dynamic_draw(NULL);
    menu_set_dynamic_area(0, 0, 0, 0);
    menu_set_dynamic_clear(0);
}

static void menu_request_redraw(uint8 full_redraw)
{
    menu_needs_update = 1U;
    if (full_redraw)
    {
        menu_full_redraw = 1U;
    }
}

static void menu_process_status_timeout(uint32 now_ms)
{
    if ('\0' != menu_status_message[0] &&
        (int32_t)(now_ms - menu_status_expire_ms) >= 0)
    {
        menu_status_message[0] = '\0';
        menu_status_expire_ms = 0U;
        menu_footer_needs_update = 1U;
    }
}

static void menu_update_selection_from_encoder(void)
{
    if (If_Switch_Encoder_Change())
    {
        current_selection -= switch_encoder_change_num;

        while (current_selection >= current_page->num_items)
        {
            current_selection -= current_page->num_items;
        }

        while (current_selection < 0)
        {
            current_selection += current_page->num_items;
        }

        menu_request_redraw(0U);
    }
}

static void menu_sync_gps_record_item(void)
{
    char new_label[48];
    uint32 point_count = (uint32)path_recorder_get_point_count();

    switch (path_recorder_get_state())
    {
        case PATH_STATE_RECORDING:
            sprintf(new_label, "2. Track Record [REC:%lu]", (unsigned long)point_count);
            break;
        case PATH_STATE_COMPLETED:
            sprintf(new_label, "2. Track Record [DONE:%lu]", (unsigned long)point_count);
            break;
        case PATH_STATE_IDLE:
        default:
            sprintf(new_label, "2. Track Record [IDLE]");
            break;
    }

    if (0 != strcmp(gps_record_menu_label, new_label))
    {
        strcpy(gps_record_menu_label, new_label);

        if ((current_page == &gps_menu) &&
            (GPS_VIEW_NONE == gps_display_mode) &&
            !menu_full_redraw)
        {
            menu_draw_item(1, (uint8)(current_selection == 1));
        }
    }
}

static void menu_sync_gps_record_param_items(void)
{
    const path_record_config_t *config = path_recorder_get_config();

    sprintf(gps_record_distance_label, "1. Point Dist [%.2fm]", config->min_record_distance);
    sprintf(gps_record_interval_label, "2. Point Intv [%lums]", (unsigned long)config->min_record_interval_ms);
    sprintf(gps_record_sat_label, "3. Min Sat [%u]", config->min_satellites);
}

static void menu_draw_item(int index, uint8 selected)
{
    uint16 y_pos;
    uint16 footer_top;

    if ((NULL == current_page) || (index < 0) || (index >= current_page->num_items))
    {
        return;
    }

    y_pos = (uint16)(MENU_ITEM_START_Y + index * MENU_ITEM_HEIGHT);
    footer_top = (uint16)(ips200_height_max - MENU_FOOTER_HEIGHT);
    if (y_pos + MENU_ITEM_TEXT_HEIGHT >= footer_top)
    {
        return;
    }

    ips200_fill_rect(5U,
                     (uint16)(y_pos - 2U),
                     (uint16)(ips200_width_max - 5U),
                     (uint16)(y_pos + MENU_ITEM_TEXT_HEIGHT),
                     selected ? current_font_color : RGB565_BLACK);
    if (selected)
    {
        ips200_set_color(RGB565_BLACK, current_font_color);
    }
    else
    {
        ips200_set_color(current_font_color, RGB565_BLACK);
    }
    show_string_fit_width(10U,
                          y_pos,
                          (uint16)(ips200_width_max - 20U),
                          current_page->items[index].name);
}

static void menu_draw_page(void)
{
    int i;

    ips200_full(RGB565_BLACK);

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    show_string_fit(10, 10, current_page->title);
    ips200_draw_line(0, MENU_TITLE_HEIGHT, ips200_width_max - 1, MENU_TITLE_HEIGHT, RGB565_GRAY);

    for (i = 0; i < current_page->num_items; i++)
    {
        menu_draw_item(i, (uint8)(i == current_selection));
    }

    menu_draw_footer();
    last_selection = current_selection;
    menu_full_redraw = 0U;
    menu_needs_update = 0U;
}

static void menu_enter_page(MenuPage *page)
{
    if (NULL == page)
    {
        return;
    }

    if (page == &gps_menu)
    {
        gps_display_mode = GPS_VIEW_NONE;
        gps_clear_status_hint();
        menu_reset_dynamic_region();
    }
    else if (page == &gps_record_param_menu)
    {
        menu_sync_gps_record_param_items();
    }

    page->parent = current_page;
    current_page = page;
    current_selection = 0;
    menu_request_redraw(1U);
}

static void menu_return_to_parent(void)
{
    if ((NULL == current_page) || (NULL == current_page->parent))
    {
        return;
    }

    current_page = current_page->parent;
    current_selection = 0;
    gps_exit_view();
    record_param_exit_view();
    gps_clear_status_hint();
    menu_reset_dynamic_region();
    menu_request_redraw(1U);
}

static void gps_draw_active_view(void)
{
    if (GPS_VIEW_DATA == gps_display_mode)
    {
        gps_draw_data_page();
    }
    else if (GPS_VIEW_MAP == gps_display_mode)
    {
        gps_draw_map_page();
    }
    else if (GPS_VIEW_RAW_DEBUG == gps_display_mode)
    {
        gps_draw_raw_debug_page();
    }
}

static uint8 menu_handle_gps_view(void)
{
    if (GPS_VIEW_NONE == gps_display_mode)
    {
        return 0U;
    }

    if (my_key_get_state(MY_KEY_1) == MY_KEY_LONG_PRESS)
    {
        my_key_clear_state(MY_KEY_1);
        gps_exit_view();
        return 1U;
    }

    if ((GPS_VIEW_MAP == gps_display_mode) &&
        (my_key_get_state(MY_KEY_1) == MY_KEY_SHORT_PRESS))
    {
        my_key_clear_state(MY_KEY_1);
        gps_clear_current_track();
        gps_draw_map_page();
        return 1U;
    }

    gps_draw_active_view();
    return 1U;
}

static uint8 menu_handle_record_param_view(void)
{
    if (RECORD_PARAM_VIEW_NONE == record_param_view_mode)
    {
        return 0U;
    }

    if (my_key_get_state(MY_KEY_1) == MY_KEY_LONG_PRESS)
    {
        my_key_clear_state(MY_KEY_1);
        record_param_exit_view();
        return 1U;
    }

    if (If_Switch_Encoder_Change())
    {
        record_param_apply_encoder_adjustment();
    }

    if (my_key_get_state(MY_KEY_1) == MY_KEY_SHORT_PRESS)
    {
        my_key_clear_state(MY_KEY_1);
        record_param_cycle_step();
    }

    record_param_draw_active_view();
    return 1U;
}

static uint8 menu_handle_pid_view(void)
{
    if (PID_VIEW_NONE == pid_display_mode)
    {
        return 0U;
    }

    if (my_key_get_state(MY_KEY_1) == MY_KEY_LONG_PRESS)
    {
        my_key_clear_state(MY_KEY_1);
        pid_exit_view();
        return 1U;
    }

    if (If_Switch_Encoder_Change())
    {
        pid_apply_encoder_adjustment();
    }

    if (my_key_get_state(MY_KEY_1) == MY_KEY_SHORT_PRESS)
    {
        my_key_clear_state(MY_KEY_1);
        if (pid_display_mode <= PID_VIEW_EDIT_KD)
        {
            pid_cycle_step(pid_display_mode);
        }
    }

    pid_draw_active_view();
    return 1U;
}

static void menu_execute_current_item(void)
{
    MenuItem *item;

    if ((NULL == current_page) || (current_selection < 0) || (current_selection >= current_page->num_items))
    {
        return;
    }

    item = &current_page->items[current_selection];
    if (NULL != item->function)
    {
        item->function();

        if (PID_VIEW_NONE != pid_display_mode)
        {
            pid_draw_active_view();
            return;
        }

        if (RECORD_PARAM_VIEW_NONE != record_param_view_mode)
        {
            record_param_draw_active_view();
            return;
        }

        if (GPS_VIEW_NONE != gps_display_mode)
        {
            gps_draw_active_view();
            return;
        }

        if (WIFI_VIEW_NONE != wifi_view_mode)
        {
            return;
        }

        menu_request_redraw(1U);
    }
    else if (NULL != item->sub_page)
    {
        menu_enter_page(item->sub_page);
    }
}

/* 初始化屏幕、按键、编码器和参数，并建立菜单初始状态 */
void menu_init(void)
{
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_dir(IPS200_DEFAULT_DISPLAY_DIR);
    ips200_set_font(IPS200_DEFAULT_DISPLAY_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_full(RGB565_BLACK);

    my_key_init(10);
    my_key_clear_all_state();
    MyEncoder_Init();

    Init_Load_Params();
    menu_sync_gps_record_item();
    menu_sync_gps_record_param_items();

    current_page = &main_menu;
    current_page->parent = NULL;
    current_selection = 0;
    last_selection = -1;
    menu_needs_update = 1;
    menu_full_redraw = 1;
    current_font_color = RGB565_WHITE;
    menu_reset_dynamic_region();
    menu_dynamic_clear_enable = 1U;
    gps_clear_status_hint();
    menu_status_message[0] = '\0';
    menu_status_expire_ms = 0U;
    menu_footer_needs_update = 1U;
}

void menu_set_dynamic_draw(menu_dynamic_draw_t callback)
{
    menu_dynamic_draw_cb = callback;
}

void menu_set_dynamic_area(uint16 x, uint16 y, uint16 w, uint16 h)
{
    menu_dynamic_x = x;
    menu_dynamic_y = y;
    menu_dynamic_w = w;
    menu_dynamic_h = h;
}

void menu_set_dynamic_clear(uint8 enable)
{
    menu_dynamic_clear_enable = enable ? 1 : 0;
}

const pid_param_t *menu_get_pid_param(void)
{
    return &menu_pid_param_cache;
}

/* 菜单周期任务：处理输入、切换页面并刷新当前显示内容 */
void menu_task(void)
{
    uint32 now_ms;

    system_delay_ms(10);
    now_ms = system_getval_ms();
    my_key_scanner();
    Get_Switch_Num();
    menu_process_status_timeout(now_ms);
    menu_sync_gps_record_item();

    if (current_page == NULL || current_page->num_items <= 0)
    {
        return;
    }

    if (menu_handle_gps_view())
    {
        return;
    }

    if (menu_handle_pid_view())
    {
        return;
    }

    if (menu_handle_record_param_view())
    {
        return;
    }

    if (menu_handle_wifi_view())
    {
        return;
    }

    menu_update_selection_from_encoder();

    if (my_key_get_state(MY_KEY_1) == MY_KEY_LONG_PRESS)
    {
        my_key_clear_state(MY_KEY_1);
        menu_return_to_parent();
        return;
    }

    if (my_key_get_state(MY_KEY_1) == MY_KEY_SHORT_PRESS)
    {
        my_key_clear_state(MY_KEY_1);
        menu_execute_current_item();

        if ((PID_VIEW_NONE != pid_display_mode) || (GPS_VIEW_NONE != gps_display_mode) || (WIFI_VIEW_NONE != wifi_view_mode))
        {
            menu_needs_update = 0U;
            menu_footer_needs_update = 0U;
            return;
        }
    }

    if (menu_full_redraw)
    {
        menu_draw_page();
    }
    else if (menu_needs_update)
    {
        menu_draw_item(last_selection, 0U);
        menu_draw_item(current_selection, 1U);
        last_selection = current_selection;
        menu_needs_update = 0U;
    }

    if (menu_footer_needs_update)
    {
        menu_draw_footer();
    }

    if (menu_dynamic_draw_cb && menu_dynamic_w && menu_dynamic_h)
    {
        if (menu_dynamic_clear_enable)
        {
            uint16 x_end = (uint16)(menu_dynamic_x + menu_dynamic_w - 1);
            uint16 y_end = (uint16)(menu_dynamic_y + menu_dynamic_h - 1);
            ips200_fill_rect(menu_dynamic_x, menu_dynamic_y, x_end, y_end, RGB565_BLACK);
        }
        menu_dynamic_draw_cb(menu_dynamic_x, menu_dynamic_y, menu_dynamic_w, menu_dynamic_h);
    }
}
