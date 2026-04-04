#ifndef __MENU_GPS_H__
#define __MENU_GPS_H__

#include "menu.h"
#include "menu_params.h"
#include "MyKey.h"
#include "MyEncoder.h"

#define MENU_GPS_STATUS_HINT_LEN 64U

typedef enum
{
    GPS_VIEW_NONE = 0U,
    GPS_VIEW_DATA,
    GPS_VIEW_MAP,
    GPS_VIEW_RAW_DEBUG,
} gps_view_mode_t;

typedef enum
{
    RECORD_PARAM_VIEW_NONE = 0U,
    RECORD_PARAM_VIEW_DISTANCE,
    RECORD_PARAM_VIEW_INTERVAL,
    RECORD_PARAM_VIEW_MIN_SAT,
} record_param_view_mode_t;

void menu_gps_init_state(char *status_hint, uint32 status_hint_size);
uint8 menu_gps_menu_gnss_ready(void);
void menu_gps_action_display_data(gps_view_mode_t *gps_mode,
                                  uint8 *menu_full_redraw,
                                  void (*drain_encoder_events)(void),
                                  void (*request_redraw)(uint8 full_redraw),
                                  void (*reset_dynamic_region)(void));
void menu_gps_action_toggle_record(gps_view_mode_t *gps_mode,
                                   char *status_hint,
                                   uint32 status_hint_size,
                                   void (*set_status_message)(const char *text, uint32 duration_ms),
                                   uint32 status_show_ms,
                                   void (*request_redraw)(uint8 full_redraw));
void menu_gps_action_map(gps_view_mode_t *gps_mode,
                         uint8 *menu_full_redraw,
                         void (*drain_encoder_events)(void),
                         void (*request_redraw)(uint8 full_redraw),
                         void (*reset_dynamic_region)(void));
void menu_gps_action_save_track_flash(gps_view_mode_t *gps_mode,
                                      void (*set_status_message)(const char *text, uint32 duration_ms),
                                      uint32 status_show_ms,
                                      void (*request_redraw)(uint8 full_redraw));
void menu_gps_action_load_track_flash(gps_view_mode_t *gps_mode,
                                      void (*set_status_message)(const char *text, uint32 duration_ms),
                                      uint32 status_show_ms,
                                      void (*request_redraw)(uint8 full_redraw));
void menu_gps_action_raw_debug(gps_view_mode_t *gps_mode,
                               uint8 *menu_full_redraw,
                               void (*drain_encoder_events)(void),
                               void (*request_redraw)(uint8 full_redraw),
                               void (*reset_dynamic_region)(void));

void menu_gps_action_record_param_distance(record_param_view_mode_t *record_mode, uint8 *menu_full_redraw, void (*request_redraw)(uint8 full_redraw));
void menu_gps_action_record_param_interval(record_param_view_mode_t *record_mode, uint8 *menu_full_redraw, void (*request_redraw)(uint8 full_redraw));
void menu_gps_action_record_param_min_sat(record_param_view_mode_t *record_mode, uint8 *menu_full_redraw, void (*request_redraw)(uint8 full_redraw));

uint8 menu_gps_handle_view(gps_view_mode_t *gps_mode,
                           char *status_hint,
                           uint32 status_hint_size,
                           uint8 *menu_full_redraw,
                           void (*drain_encoder_events)(void),
                           void (*request_redraw)(uint8 full_redraw),
                           void (*reset_dynamic_region)(void));

uint8 menu_gps_handle_record_param_view(record_param_view_mode_t *record_mode,
                                        MyParams_t *params,
                                        uint8 *menu_full_redraw,
                                        void (*request_redraw)(uint8 full_redraw),
                                        void (*set_status_message)(const char *text, uint32 duration_ms),
                                        uint32 status_show_ms,
                                        void (*sync_record_param_items)(void));

#endif
