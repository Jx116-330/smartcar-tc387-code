/*********************************************************************************************************************
* File: menu.c
* Brief: 菜单系统实现文件
* Author: JX116
* Note: 负责菜单绘制、编码器和按键交互，以及 GPS / PID 页面切换与参数加载保存逻辑
*********************************************************************************************************************/

#include <ICM42688/icm_ins.h>
#include <ICM42688/ins_playback.h>
#include <ICM42688/ins_record.h>
#include "menu.h"
#include "menu_params.h"
#include "menu_pid.h"
#include "menu_gps.h"
#include "menu_fusion.h"
#include "zf_device_ips200.h"
#include "zf_common_headfile.h"
#include "zf_driver_gpio.h"
#include "path_recorder.h"
#include "path_display.h"
#include "MyKey.h"
#include "MyEncoder.h"
#include "wifi_menu.h"
#include "tuning_soft.h"
#include "menu_icm.h"
#include "menu_pedal.h"
#include "menu_link.h"
#include "menu_turn.h"
#include "encoder_odom.h"
#include "encoder_odom_right.h"
#include "menu_ui_utils.h"

MyParams_t g_params;

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
static uint8 icm_display_mode    = ICM_VIEW_NONE;
static uint8 fusion_display_mode = FUSION_VIEW_NONE;
static uint8 pedal_display_mode  = PEDAL_VIEW_NONE;
static uint8 link_display_mode   = LINK_VIEW_NONE;
static menu_view_ctx_t icm_ctx;
static menu_view_ctx_t fusion_ctx;
static menu_view_ctx_t pedal_ctx;
static menu_view_ctx_t link_ctx;
static char gps_status_hint[64] = "";
static MenuPage *current_page = NULL;
static MenuPage gps_menu;
static MenuPage icm_menu;
static MenuPage ins_replay_menu;
static MenuPage pedal_menu;
static pid_param_t menu_pid_param_cache;
static pid_controller_t pid_preview_controller;
static char menu_status_message[32] = "";
static uint32 menu_status_expire_ms = 0U;
static uint8 menu_footer_needs_update = 1U;
static uint8 pid_param_dirty = 0U;
static char gps_record_menu_label[48] = "2. Track Record [IDLE]";
static char gps_record_distance_label[48] = "1. Point Dist [0.20m]";
static char gps_record_interval_label[48] = "2. Point Intv [50ms]";
static char gps_record_sat_label[48] = "3. Min Sat [4]";

static void menu_set_status_message(const char *text, uint32 duration_ms);
static void menu_draw_footer(void);
static void menu_drain_encoder_events(void);
static void menu_reset_dynamic_region(void);
static void menu_request_redraw(uint8 full_redraw);
static void menu_process_status_timeout(uint32 now_ms);
static void menu_update_selection_from_encoder(void);
static uint8 menu_is_gps_root_idle(void);
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
static void gps_action_record_param_distance(void);
static void gps_action_record_param_interval(void);
static void gps_action_record_param_min_sat(void);
static void gps_action_save_track_flash(void);
static void gps_action_load_track_flash(void);
static void pid_action_edit_kp(void);
static void pid_action_edit_ki(void);
static void pid_action_edit_kd(void);
static void pid_action_preview(void);
static void pid_action_reset(void);
static void gps_action_raw_debug(void);
static void icm_action_raw_data(void);
static void icm_action_attitude(void);
static void icm_action_gyro_bias_calib(void);
static void icm_action_ins_debug(void);
static void icm_action_ins_track_map(void);
static void icm_action_ins_rec_toggle(void);
static void menu_sync_ins_rec_item(void);
static void ins_replay_action_load(void);
static void ins_replay_action_start(void);
static void ins_replay_action_stop(void);
static void ins_replay_action_clear(void);
static void menu_sync_ins_replay_labels(void);
/* static void fusion_action_debug(void); — Fusion 菜单已移除 */

/* 持久化参数已拆到 menu_params.c；这里保留运行时同步逻辑。 */
static void params_set_default(void)
{
    menu_params_set_default(&g_params);
    menu_pid_sync_runtime_param(&g_params, &menu_pid_param_cache, &pid_preview_controller);
    menu_params_apply_record_config(&g_params);
}

/* 上电时从 Flash 加载参数，若无效则回退到默认配置 */
static void Init_Load_Params(void)
{
    if (!menu_params_load_or_default(&g_params))
    {
        params_set_default();
        return;
    }

    menu_pid_sync_runtime_param(&g_params, &menu_pid_param_cache, &pid_preview_controller);
    menu_params_apply_record_config(&g_params);
    menu_gps_init_state(gps_status_hint, (uint32)sizeof(gps_status_hint));
}


static void gps_clear_status_hint(void)
{
    gps_status_hint[0] = '\0';
}

static void gps_action_record_param_distance(void)
{
    menu_gps_action_record_param_distance(&record_param_view_mode, &menu_full_redraw, menu_request_redraw);
}

static void gps_action_record_param_interval(void)
{
    menu_gps_action_record_param_interval(&record_param_view_mode, &menu_full_redraw, menu_request_redraw);
}

static void gps_action_record_param_min_sat(void)
{
    menu_gps_action_record_param_min_sat(&record_param_view_mode, &menu_full_redraw, menu_request_redraw);
}

static void pid_action_edit_kp(void)
{
    menu_pid_action_enter(&pid_display_mode, PID_VIEW_EDIT_KP,
                          &g_params, &menu_pid_param_cache,
                          &pid_preview_controller, &menu_full_redraw,
                          menu_request_redraw);
}

static void pid_action_edit_ki(void)
{
    menu_pid_action_enter(&pid_display_mode, PID_VIEW_EDIT_KI,
                          &g_params, &menu_pid_param_cache,
                          &pid_preview_controller, &menu_full_redraw,
                          menu_request_redraw);
}

static void pid_action_edit_kd(void)
{
    menu_pid_action_enter(&pid_display_mode, PID_VIEW_EDIT_KD,
                          &g_params, &menu_pid_param_cache,
                          &pid_preview_controller, &menu_full_redraw,
                          menu_request_redraw);
}

static void pid_action_preview(void)
{
    menu_pid_action_enter(&pid_display_mode, PID_VIEW_PREVIEW,
                          &g_params, &menu_pid_param_cache,
                          &pid_preview_controller, &menu_full_redraw,
                          menu_request_redraw);
}

static void pid_action_reset(void)
{
    menu_pid_action_reset(&g_params,
                          &pid_display_mode,
                          &menu_pid_param_cache,
                          &pid_preview_controller,
                          &pid_param_dirty,
                          &menu_full_redraw,
                          menu_set_status_message,
                          MENU_STATUS_SHOW_MS);
}

/* 打开 GPS 数据页，显示当前定位与轨迹统计信息 */
static void gps_action_display_data(void)
{
    menu_gps_action_display_data(&gps_display_mode,
                                 &menu_full_redraw,
                                 menu_drain_encoder_events,
                                 menu_request_redraw,
                                 menu_reset_dynamic_region);
}

/* 根据当前记录状态启动或停止轨迹记录，并更新状态提示 */
static void gps_action_toggle_record(void)
{
    menu_gps_action_toggle_record(&gps_display_mode,
                                  gps_status_hint,
                                  (uint32)sizeof(gps_status_hint),
                                  menu_set_status_message,
                                  MENU_STATUS_SHOW_MS,
                                  menu_request_redraw);
}

/* 初始化轨迹显示区域并进入 GPS 地图页面 */
static void gps_action_map(void)
{
    menu_gps_action_map(&gps_display_mode,
                        &menu_full_redraw,
                        menu_drain_encoder_events,
                        menu_request_redraw,
                        menu_reset_dynamic_region);
}

static void gps_action_save_track_flash(void)
{
    menu_gps_action_save_track_flash(&gps_display_mode,
                                     menu_set_status_message,
                                     MENU_STATUS_SHOW_MS,
                                     menu_request_redraw);
}

static void gps_action_load_track_flash(void)
{
    menu_gps_action_load_track_flash(&gps_display_mode,
                                     menu_set_status_message,
                                     MENU_STATUS_SHOW_MS,
                                     menu_request_redraw);
}

static void gps_action_raw_debug(void)
{
    menu_gps_action_raw_debug(&gps_display_mode,
                              &menu_full_redraw,
                              menu_drain_encoder_events,
                              menu_request_redraw,
                              menu_reset_dynamic_region);
}

static void icm_action_raw_data(void)          { menu_icm_action_enter(&icm_ctx, ICM_VIEW_RAW); }
static void icm_action_gyro_bias_calib(void)   { menu_icm_action_enter(&icm_ctx, ICM_VIEW_GYRO_BIAS_CALIB); }
static void icm_action_ins_track_map(void)     { menu_icm_action_enter(&icm_ctx, ICM_VIEW_INS_MAP); }
static void icm_action_encoder(void)           { menu_icm_action_enter(&icm_ctx, ICM_VIEW_ENCODER); }

static char icm_ins_rec_label[48] = "4. INS REC [IDLE]";

static void icm_action_ins_rec_toggle(void)
{
    if (ins_record_is_recording())
    {
        ins_record_stop();
        menu_set_status_message("INS Rec Stopped", MENU_STATUS_SHOW_MS);
    }
    else
    {
        ins_record_start();
        menu_set_status_message("INS Rec Started", MENU_STATUS_SHOW_MS);
    }
    menu_request_full_redraw();
}

static void icm_action_clear_track(void)
{
    if (ins_record_is_recording())
        ins_record_stop();
    ins_record_clear();
    ins_playback_stop();
    encoder_odom_reset();
    encoder_odom_right_reset();
    menu_set_status_message("Track Cleared", MENU_STATUS_SHOW_MS);
    menu_request_full_redraw();
}

static void icm_action_reset_ins(void)
{
    icm_ins_reset_position();
    icm_ins_reset_velocity();
    encoder_odom_reset();
    encoder_odom_right_reset();
    menu_set_status_message("INS Reset", MENU_STATUS_SHOW_MS);
    menu_request_full_redraw();
}

static char icm_stream_label[48] = "6. Stream [OFF]";

static void icm_action_stream_toggle(void)
{
    extern volatile uint8 track_stream_enabled;

    track_stream_enabled = track_stream_enabled ? 0U : 1U;
    snprintf(icm_stream_label, sizeof(icm_stream_label),
             "6. Stream [%s]", track_stream_enabled ? "ON" : "OFF");
    menu_set_status_message(track_stream_enabled ? "Stream Started" : "Stream Stopped",
                            MENU_STATUS_SHOW_MS);
    menu_request_full_redraw();
}

static void icm_action_upload_track(void)
{
    extern volatile uint8  track_dump_active;
    extern volatile uint16 track_dump_index;
    uint16 pts = ins_record_get_point_count();

    if (0U == pts)
    {
        menu_set_status_message("No points to upload", MENU_STATUS_SHOW_MS);
        menu_request_full_redraw();
        return;
    }
    track_dump_index = 0U;
    track_dump_active = 1U;
    {
        char msg[48];
        snprintf(msg, sizeof(msg), "Uploading %u pts...", (unsigned int)pts);
        menu_set_status_message(msg, MENU_STATUS_SHOW_MS);
    }
    menu_request_full_redraw();
}

static void menu_sync_ins_rec_item(void)
{
    char new_label[48];

    if (ins_record_is_recording())
    {
        sprintf(new_label, "4. INS REC [REC:%u]", (unsigned int)ins_record_get_point_count());
    }
    else
    {
        sprintf(new_label, "4. INS REC [%u pts]", (unsigned int)ins_record_get_point_count());
    }

    if (menu_ui_update_label(icm_ins_rec_label, sizeof(icm_ins_rec_label), new_label)
        && (current_page == &icm_menu)
        && (ICM_VIEW_NONE == icm_display_mode)
        && !menu_full_redraw)
    {
        menu_draw_item(3, (uint8)(current_selection == 3));
    }

    /* Stream label 同步 */
    {
        extern volatile uint8 track_stream_enabled;
        char stream_lbl[48];
        snprintf(stream_lbl, sizeof(stream_lbl),
                 "6. Stream [%s]", track_stream_enabled ? "ON" : "OFF");
        if (menu_ui_update_label(icm_stream_label, sizeof(icm_stream_label), stream_lbl)
            && (current_page == &icm_menu)
            && (ICM_VIEW_NONE == icm_display_mode)
            && !menu_full_redraw)
        {
            menu_draw_item(5, (uint8)(current_selection == 5));
        }
    }
}

/* ---- GPS+INS Fusion 菜单 ---- */

/* Fusion 菜单已移除，保留 ctx/display_mode 供 handle_view 链兼容 */
/* static void fusion_action_debug(void) { menu_fusion_action_enter(&fusion_ctx, FUSION_VIEW_DEBUG); } */

/* ---- Pedal ---- */

/* ---- TC264 Link ---- */

static void link_action_debug(void)     { menu_link_action_enter(&link_ctx, LINK_VIEW_DEBUG); }
static void link_action_hq_status(void) { menu_link_action_enter(&link_ctx, LINK_VIEW_HQ_STATUS); }

/* ---- Pedal ---- */

static void pedal_action_debug(void)      { menu_pedal_action_enter(&pedal_ctx, PEDAL_VIEW_DEBUG); }
static void pedal_action_drive_ctrl(void) { menu_pedal_action_enter(&pedal_ctx, PEDAL_VIEW_DRIVE_CTRL); }

/* ---- INS Replay 子菜单 ---- */

/* 动态标签：随状态每帧刷新 */
static char ins_replay_load_label[48]  = "1. Load Track [0 pts]";
static char ins_replay_rec_status[48]  = "5. REC:OFF 0pts";
static char ins_replay_play_status[64] = "6. PLAY:IDLE";

static void ins_replay_action_load(void)
{
    if (ins_playback_load())
    {
        menu_set_status_message("Load OK", MENU_STATUS_SHOW_MS);
    }
    else
    {
        menu_set_status_message("Load FAIL: no pts", MENU_STATUS_SHOW_MS);
    }
    menu_request_full_redraw();
}

static void ins_replay_action_start(void)
{
    if (INS_PLAY_READY != ins_playback_get_state())
    {
        menu_set_status_message("Load first!", MENU_STATUS_SHOW_MS);
        return;
    }
    ins_playback_start();
    menu_set_status_message("Replay Started", MENU_STATUS_SHOW_MS);
    menu_request_full_redraw();
}

static void ins_replay_action_stop(void)
{
    ins_playback_stop();
    menu_set_status_message("Replay Stopped", MENU_STATUS_SHOW_MS);
    menu_request_full_redraw();
}

static void ins_replay_action_clear(void)
{
    /* 先停止 playback，再清除轨迹，防止 playback 访问已清除内存 */
    ins_playback_stop();
    ins_record_clear();
    menu_set_status_message("Track Cleared", MENU_STATUS_SHOW_MS);
    menu_request_full_redraw();
}

static void menu_sync_ins_replay_labels(void)
{
    char buf[64];
    ins_play_state_t play_st = ins_playback_get_state();
    float dist;

    /* ---- 1. Load Track 标签：显示已加载点数或回放进度 ---- */
    if (INS_PLAY_RUNNING == play_st)
    {
        sprintf(buf, "1. Load [RUN:%u/%u]",
                (unsigned int)ins_playback_get_target_idx(),
                (unsigned int)ins_playback_get_total_points());
    }
    else if (INS_PLAY_READY == play_st)
    {
        sprintf(buf, "1. Load [RDY:%u pts]",
                (unsigned int)ins_playback_get_total_points());
    }
    else
    {
        sprintf(buf, "1. Load Track [%u pts]",
                (unsigned int)ins_record_get_point_count());
    }
    if (menu_ui_update_label(ins_replay_load_label, sizeof(ins_replay_load_label), buf)
        && (current_page == &ins_replay_menu) && !menu_full_redraw)
    {
        menu_draw_item(0, (uint8)(current_selection == 0));
    }

    /* ---- 5. REC 状态行 ---- */
    if (ins_record_is_recording())
    {
        sprintf(buf, "5. REC:ON  %u pts", (unsigned int)ins_record_get_point_count());
    }
    else
    {
        sprintf(buf, "5. REC:OFF %u pts", (unsigned int)ins_record_get_point_count());
    }
    if (menu_ui_update_label(ins_replay_rec_status, sizeof(ins_replay_rec_status), buf)
        && (current_page == &ins_replay_menu) && !menu_full_redraw)
    {
        menu_draw_item(4, (uint8)(current_selection == 4));
    }

    /* ---- 6. PLAY 状态行 ---- */
    if (INS_PLAY_RUNNING == play_st)
    {
        dist = ins_playback_get_dist_to_target();
        if (dist < 0.0f) { dist = 0.0f; }
        sprintf(buf, "6. PLAY:RUN %u/%u %.2fm",
                (unsigned int)ins_playback_get_target_idx(),
                (unsigned int)ins_playback_get_total_points(),
                dist);
    }
    else if (INS_PLAY_READY == play_st)
    {
        sprintf(buf, "6. PLAY:RDY %u pts",
                (unsigned int)ins_playback_get_total_points());
    }
    else
    {
        sprintf(buf, "6. PLAY:IDLE");
    }
    if (menu_ui_update_label(ins_replay_play_status, sizeof(ins_replay_play_status), buf)
        && (current_page == &ins_replay_menu) && !menu_full_redraw)
    {
        menu_draw_item(5, (uint8)(current_selection == 5));
    }
}

static MenuItem ins_replay_items[] = {
    {ins_replay_load_label,  ins_replay_action_load,  NULL},  /* 1. Load Track  */
    {"2. Start Replay",      ins_replay_action_start, NULL},  /* 2. Start       */
    {"3. Stop Replay",       ins_replay_action_stop,  NULL},  /* 3. Stop        */
    {"4. Clear Track",       ins_replay_action_clear, NULL},  /* 4. Clear       */
    {ins_replay_rec_status,  NULL,                    NULL},  /* 5. REC status  */
    {ins_replay_play_status, NULL,                    NULL},  /* 6. PLAY status */
};

static MenuPage ins_replay_menu = {
    "INS Replay",
    ins_replay_items,
    sizeof(ins_replay_items) / sizeof(MenuItem),
    NULL
};

static MenuItem icm_items[] = {
    {"1. IMU Data",       icm_action_raw_data,          NULL},
    {"2. Encoder",        icm_action_encoder,           NULL},
    {"3. Gyro Bias Cal",  icm_action_gyro_bias_calib,   NULL},
    {icm_ins_rec_label,   icm_action_ins_rec_toggle,    NULL},  /* 4. INS REC toggle */
    {"5. Track Map",      icm_action_ins_track_map,     NULL},
    {icm_stream_label,    icm_action_stream_toggle,     NULL},  /* 6. Stream ON/OFF */
    {"7. Upload Track",   icm_action_upload_track,      NULL},  /* WiFi dump to PC */
    {"8. Clear Track",    icm_action_clear_track,       NULL},
    {"9. Reset INS",      icm_action_reset_ins,         NULL},
};

static MenuPage icm_menu = {
    "INS",
    icm_items,
    sizeof(icm_items) / sizeof(MenuItem),
    NULL
};

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
    {"5. Save Track Flash", gps_action_save_track_flash, NULL},
    {"6. Load Track Flash", gps_action_load_track_flash, NULL},
    {"7. GPS RAW/DEBUG", gps_action_raw_debug, NULL},
};

static MenuPage gps_menu = {
    "GPS",
    gps_items,
    sizeof(gps_items) / sizeof(MenuItem),
    NULL
};

/* Camera 菜单已从主菜单移除（全部项为空实现），如需恢复取消注释即可 */
#if 0
static MenuItem threshold_algo_items[] = {
    {"1. Manual", NULL, NULL},
    {"2. Otsu", NULL, NULL},
    {"3. Kittler", NULL, NULL},
    {"4. Sauvola", NULL, NULL},
};
static MenuPage threshold_algo_menu = {
    "Threshold Mode", threshold_algo_items,
    sizeof(threshold_algo_items) / sizeof(MenuItem), NULL
};
static MenuItem camera_items[] = {
    {"1. Preview", NULL, NULL},
    {"2. Threshold", NULL, NULL},
    {"3. Color Preview", NULL, NULL},
    {"4. Threshold Mode", NULL, &threshold_algo_menu},
    {"5. Template Manage", NULL, NULL},
};
static MenuPage camera_menu = {
    "Camera", camera_items,
    sizeof(camera_items) / sizeof(MenuItem), NULL
};
#endif

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

/* Fusion 菜单已从主菜单移除（GPS+INS 融合调试），如需恢复取消注释即可 */
#if 0
static MenuItem fusion_items[] = {
    {"1. Fusion Debug", fusion_action_debug, NULL},
};
static MenuPage fusion_menu = {
    "GPS+INS Fusion", fusion_items,
    sizeof(fusion_items) / sizeof(MenuItem), NULL
};
#endif

static MenuItem pedal_items[] = {
    {"1. Pedal Debug",    pedal_action_debug,      NULL},
    {"2. Drive Control",  pedal_action_drive_ctrl,  NULL},
};

static MenuPage pedal_menu = {
    "Pedal",
    pedal_items,
    sizeof(pedal_items) / sizeof(MenuItem),
    NULL
};

static MenuItem link_items[] = {
    {"1. Link Debug", link_action_debug,     NULL},
    {"2. HQ Status",  link_action_hq_status, NULL},
};

static MenuPage link_menu = {
    "TC264 Link",
    link_items,
    sizeof(link_items) / sizeof(MenuItem),
    NULL
};

static MenuItem main_items[] = {
    {"1. GPS",          NULL, &gps_menu},
    {"2. INS",          NULL, &icm_menu},
    {"3. Pedal",        NULL, &pedal_menu},
    {"4. TC264 Link",   NULL, &link_menu},
    {"5. PID",          NULL, &pid_menu},
    {"6. WiFi",         NULL, &wifi_page},
    {"7. Tuning",       NULL, &tuning_menu},
    {"8. Turn",         NULL, &turn_menu},
};

static MenuPage main_menu = {
    "Jx116 Menu",
    main_items,
    sizeof(main_items) / sizeof(MenuItem),
    NULL
};



static void menu_set_status_message(const char *text, uint32 duration_ms)
{
    menu_status_message[0] = '\0';
    menu_status_expire_ms = 0U;

    if (NULL == text || '\0' == text[0])
    {
        menu_footer_needs_update = 1U;
        return;
    }

    menu_ui_copy_text(menu_status_message, (uint32)sizeof(menu_status_message), text);
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

    menu_ui_fill_rect(0U,
                     (uint16)(ips200_height_max - 20U),
                     (uint16)(ips200_width_max - 1U),
                     (uint16)(ips200_height_max - 1U),
                     RGB565_BLACK);
    ips200_set_color(text_color, RGB565_BLACK);
    menu_ui_show_pad(5U,
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

static uint8 menu_is_gps_root_idle(void)
{
    return ((current_page == &gps_menu) &&
            (GPS_VIEW_NONE == gps_display_mode) &&
            !menu_full_redraw) ? 1U : 0U;
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

    if (menu_ui_update_label(gps_record_menu_label, sizeof(gps_record_menu_label), new_label)
        && menu_is_gps_root_idle())
    {
        menu_draw_item(1, (uint8)(current_selection == 1));
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

    menu_ui_fill_rect(5U,
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
    menu_ui_show_fit_width(10U,
                          y_pos,
                          (uint16)(ips200_width_max - 20U),
                          current_page->items[index].name);
}

static void menu_draw_page(void)
{
    int i;

    ips200_full(RGB565_BLACK);

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    menu_ui_show_fit(10, 10, current_page->title);
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
    gps_display_mode = GPS_VIEW_NONE;
    record_param_view_mode = RECORD_PARAM_VIEW_NONE;
    icm_display_mode    = 0U;
    fusion_display_mode = 0U;
    pedal_display_mode  = 0U;
    link_display_mode   = 0U;
    gps_clear_status_hint();
    menu_reset_dynamic_region();
    menu_request_redraw(1U);
}

static uint8 menu_handle_gps_view(void)
{
    return menu_gps_handle_view(&gps_display_mode,
                                gps_status_hint,
                                (uint32)sizeof(gps_status_hint),
                                &menu_full_redraw,
                                menu_drain_encoder_events,
                                menu_request_redraw,
                                menu_reset_dynamic_region);
}

static uint8 menu_handle_record_param_view(void)
{
    return menu_gps_handle_record_param_view(&record_param_view_mode,
                                             &g_params,
                                             &menu_full_redraw,
                                             menu_request_redraw,
                                             menu_set_status_message,
                                             MENU_STATUS_SHOW_MS,
                                             menu_sync_gps_record_param_items);
}

static uint8 menu_handle_pid_view(void)
{
    return menu_pid_handle_view(&pid_display_mode,
                                &g_params,
                                &menu_pid_param_cache,
                                &pid_preview_controller,
                                &pid_param_dirty,
                                &menu_full_redraw,
                                menu_request_redraw,
                                menu_set_status_message,
                                MENU_STATUS_SHOW_MS);
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
            menu_handle_pid_view();
            return;
        }

        if (RECORD_PARAM_VIEW_NONE != record_param_view_mode)
        {
            menu_handle_record_param_view();
            return;
        }

        if (GPS_VIEW_NONE != gps_display_mode)
        {
            menu_handle_gps_view();
            return;
        }

        if (wifi_menu_is_active())
        {
            wifi_menu_handle_view();
            return;
        }

        if (ICM_VIEW_NONE != icm_display_mode)
        {
            menu_icm_handle_view(&icm_ctx);
            return;
        }

        if (FUSION_VIEW_NONE != fusion_display_mode)
        {
            menu_fusion_handle_view(&fusion_ctx);
            return;
        }

        if (PEDAL_VIEW_NONE != pedal_display_mode)
        {
            menu_pedal_handle_view(&pedal_ctx);
            return;
        }

        if (LINK_VIEW_NONE != link_display_mode)
        {
            menu_link_handle_view(&link_ctx);
            return;
        }

        if (menu_turn_is_active())
        {
            menu_turn_handle_view();
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

    /* 初始化 4 个视图上下文 */
    icm_ctx.mode                 = &icm_display_mode;
    icm_ctx.menu_full_redraw     = &menu_full_redraw;
    icm_ctx.drain_encoder_events = menu_drain_encoder_events;
    icm_ctx.request_redraw       = menu_request_redraw;
    icm_ctx.reset_dynamic_region = menu_reset_dynamic_region;

    fusion_ctx.mode                 = &fusion_display_mode;
    fusion_ctx.menu_full_redraw     = &menu_full_redraw;
    fusion_ctx.drain_encoder_events = menu_drain_encoder_events;
    fusion_ctx.request_redraw       = menu_request_redraw;
    fusion_ctx.reset_dynamic_region = menu_reset_dynamic_region;

    pedal_ctx.mode                 = &pedal_display_mode;
    pedal_ctx.menu_full_redraw     = &menu_full_redraw;
    pedal_ctx.drain_encoder_events = menu_drain_encoder_events;
    pedal_ctx.request_redraw       = menu_request_redraw;
    pedal_ctx.reset_dynamic_region = menu_reset_dynamic_region;

    link_ctx.mode                 = &link_display_mode;
    link_ctx.menu_full_redraw     = &menu_full_redraw;
    link_ctx.drain_encoder_events = menu_drain_encoder_events;
    link_ctx.request_redraw       = menu_request_redraw;
    link_ctx.reset_dynamic_region = menu_reset_dynamic_region;

    Init_Load_Params();
    menu_sync_gps_record_item();
    menu_sync_gps_record_param_items();
    menu_sync_ins_rec_item();
    menu_sync_ins_replay_labels();
    tuning_soft_init();
    Turn_Page_Init();

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

uint8 menu_set_pid_param(const pid_param_t *param, uint8 save_to_flash)
{
    if (NULL == param)
    {
        return 0U;
    }

    if ((param->kp < PARAM_PID_P_MIN) || (param->kp > PARAM_PID_P_MAX) ||
        (param->ki < PARAM_PID_I_MIN) || (param->ki > PARAM_PID_I_MAX) ||
        (param->kd < PARAM_PID_D_MIN) || (param->kd > PARAM_PID_D_MAX))
    {
        return 0U;
    }

    g_params.pid_p = param->kp;
    g_params.pid_i = param->ki;
    g_params.pid_d = param->kd;
    pid_param_dirty = 1U;
    menu_pid_sync_runtime_param(&g_params, &menu_pid_param_cache, &pid_preview_controller);

    if (save_to_flash)
    {
        if (!menu_params_save_to_flash(&g_params))
        {
            return 0U;
        }
        pid_param_dirty = 0U;
    }

    return 1U;
}

void menu_request_full_redraw(void)
{
    menu_request_redraw(1U);
}

/* 菜单周期任务：处理输入、切换页面并刷新当前显示内容
 * 非阻塞节拍：不到 10ms 直接返回，不阻塞主循环。
 * board_comm_task() 因此可以在间隙高速运行，及时处理 UART 接收。 */
void menu_task(void)
{
    static uint32 menu_last_tick_ms = 0U;
    uint32 now_ms = system_getval_ms();

    if ((now_ms - menu_last_tick_ms) < 10U)
        return;                         /* 不到 10ms，跳过本次，不阻塞 */
    menu_last_tick_ms = now_ms;
    my_key_scanner();
    Get_Switch_Num();
    menu_process_status_timeout(now_ms);
    menu_sync_gps_record_item();
    menu_sync_ins_rec_item();
    menu_sync_ins_replay_labels();
    menu_turn_sync_labels();
    tuning_soft_task();

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

    if (wifi_menu_handle_view())
    {
        return;
    }

    if (tuning_soft_handle_view())
    {
        return;
    }

    if (menu_icm_handle_view(&icm_ctx))
    {
        return;
    }

    if (menu_fusion_handle_view(&fusion_ctx))
    {
        return;
    }

    if (menu_pedal_handle_view(&pedal_ctx))
    {
        return;
    }

    if (menu_link_handle_view(&link_ctx))
    {
        return;
    }

    if (menu_turn_handle_view())
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

        if ((PID_VIEW_NONE != pid_display_mode) || (GPS_VIEW_NONE != gps_display_mode) || (RECORD_PARAM_VIEW_NONE != record_param_view_mode) || wifi_menu_is_active() || tuning_soft_is_active() || menu_turn_is_active() || (ICM_VIEW_NONE != icm_display_mode) || (FUSION_VIEW_NONE != fusion_display_mode) || (PEDAL_VIEW_NONE != pedal_display_mode) || (LINK_VIEW_NONE != link_display_mode))
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
            menu_ui_fill_rect(menu_dynamic_x, menu_dynamic_y, x_end, y_end, RGB565_BLACK);
        }
        menu_dynamic_draw_cb(menu_dynamic_x, menu_dynamic_y, menu_dynamic_w, menu_dynamic_h);
    }
}
