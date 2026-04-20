#include <ICM42688/icm_attitude.h>
#include <ICM42688/icm_gps_fusion.h>
#include <ICM42688/icm_ins.h>
#include <ICM42688/ins_ctrl.h>
#include <ICM42688/ins_playback.h>
#include <ICM42688/ins_record.h>
#include "tuning_soft.h"

#include "zf_common_headfile.h"
#include "zf_device_ips200.h"
#include "zf_device_wifi_spi.h"
#include "wifi_menu.h"
#include "path_recorder.h"
#include "MyKey.h"
#include "MyEncoder.h"
#include "autotune.h"
#include "rear_left_encoder.h"
#include "encoder_odom.h"
#include "Turn.h"
#include "rear_right_encoder.h"
#include "encoder_odom_right.h"
#include "pedal_input.h"
#include <math.h>

#define TUNING_DEFAULT_PERIOD_MS 20U
#define TUNING_MIN_PERIOD_MS 10U
#define TUNING_MAX_PERIOD_MS 1000U
#define TUNING_STEP_MS 10U
#define TUNING_SEND_BUFFER_LEN 384U
#define TUNING_RECV_BUFFER_LEN 192U
#define TUNING_RESP_BUFFER_LEN 256U
#define TUNING_TITLE_H 30U
#define TUNING_LINE_H 20U
#define TUNING_LINE_START 38U
#define TUNING_PAD_X 8U
#define TUNING_FOOTER_H 20U
#define TUNING_DATA_WIDTH ((uint16)(ips200_width_max - TUNING_PAD_X - 4U))

typedef enum
{
    TUNING_VIEW_NONE = 0U,
    TUNING_VIEW_STATUS,
} tuning_view_mode_t;

static volatile uint8 tuning_enabled = 0U;
static uint16 tuning_period_ms = TUNING_DEFAULT_PERIOD_MS;
static uint32 tuning_last_send_ms = 0U;
static uint32 tuning_total_sent = 0U;
static uint32 tuning_total_fail = 0U;
static tuning_view_mode_t tuning_view_mode = TUNING_VIEW_NONE;
static uint8 tuning_full_redraw = 1U;
static uint32 tuning_last_view_ms = 0U;

/* ---- Track streaming / dump state ---- */
volatile uint8 track_stream_enabled = 0U;
static uint16 track_stream_interval_ms = 50U;
static uint32 track_stream_last_ms = 0U;
volatile uint16 track_dump_index = 0U;
volatile uint8  track_dump_active = 0U;

#define TRACK_STREAM_MIN_MS   20U
#define TRACK_STREAM_MAX_MS   500U
#define TRACK_STREAM_STEP_MS  10U
#define TRACK_DUMP_BATCH      5U

static char tuning_toggle_label[48] = "1. Telemetry [OFF]";
static char tuning_period_plus_label[48] = "3. Period + [100ms]";
static char tuning_period_minus_label[48] = "4. Period - [100ms]";
static char track_stream_label[48]  = "5. Track Stream [OFF]";
static char track_record_label[48]  = "6. Track Record [IDLE]";
static char track_rate_label[48]    = "7. Stream Rate [50ms]";

static void tuning_fill_rect(uint16 x0, uint16 y0, uint16 x1, uint16 y1, uint16 color);
static void tuning_show_padded(uint16 x, uint16 y, const char *s);
static void tuning_draw_header(const char *title);
static void tuning_draw_footer(const char *hint);
static void tuning_draw_status(void);
static void tuning_update_labels(void);
static void tuning_set_period(uint16 period_ms);
static void tuning_enter_view(tuning_view_mode_t mode);
static void tuning_exit_view(void);
static void tuning_action_toggle(void);
static void tuning_action_status(void);
static void tuning_action_period_plus(void);
static void tuning_action_period_minus(void);
static void tuning_action_track_stream_toggle(void);
static void tuning_action_track_record_toggle(void);
static void tuning_action_track_rate_cycle(void);
static uint8 tuning_send_once(void);
static uint8 tuning_send_line(const char *line);
static void tuning_send_track_point(void);
static void tuning_send_track_dump_batch(void);
static void tuning_send_response(const char *response);
static void tuning_reply_error(const char *cmd, const char *reason);
static void tuning_reply_ack_pid(const char *cmd);
static void tuning_process_rx_line(char *line);
static void tuning_receive_task(void);

static MenuItem tuning_items[] = {
    {tuning_toggle_label, tuning_action_toggle, NULL},
    {"2. Status", tuning_action_status, NULL},
    {tuning_period_plus_label, tuning_action_period_plus, NULL},
    {tuning_period_minus_label, tuning_action_period_minus, NULL},
    {track_stream_label, tuning_action_track_stream_toggle, NULL},
    {track_record_label, tuning_action_track_record_toggle, NULL},
    {track_rate_label, tuning_action_track_rate_cycle, NULL},
};

MenuPage tuning_menu = {
    "Tuning",
    tuning_items,
    sizeof(tuning_items) / sizeof(MenuItem),
    NULL
};

static void tuning_fill_rect(uint16 x0, uint16 y0, uint16 x1, uint16 y1, uint16 color)
{
    uint16 y;
    for (y = y0; y <= y1; y++)
    {
        ips200_draw_line(x0, y, x1, y, color);
    }
}

static void tuning_show_padded(uint16 x, uint16 y, const char *s)
{
    int max_chars;
    char buf[64];
    int i;

    max_chars = (int)(TUNING_DATA_WIDTH / 8U);
    if (max_chars <= 0 || NULL == s)
    {
        return;
    }
    if (max_chars > (int)(sizeof(buf) - 1))
    {
        max_chars = (int)(sizeof(buf) - 1);
    }

    for (i = 0; i < max_chars && s[i] != '\0'; i++)
    {
        buf[i] = s[i];
    }
    while (i < max_chars)
    {
        buf[i] = ' ';
        i++;
    }
    buf[i] = '\0';
    ips200_show_string(x, y, buf);
}

static void tuning_draw_header(const char *title)
{
    tuning_fill_rect(0, 0, (uint16)(ips200_width_max - 1U), (uint16)(TUNING_TITLE_H - 1U), RGB565_BLACK);
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(TUNING_PAD_X, 8U, title);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_draw_line(0, TUNING_TITLE_H, (uint16)(ips200_width_max - 1U), TUNING_TITLE_H, RGB565_GRAY);
}

static void tuning_draw_footer(const char *hint)
{
    uint16 y = (uint16)(ips200_height_max - TUNING_FOOTER_H);
    tuning_fill_rect(0, y, (uint16)(ips200_width_max - 1U), (uint16)(ips200_height_max - 1U), RGB565_BLACK);
    ips200_draw_line(0, y, (uint16)(ips200_width_max - 1U), y, RGB565_GRAY);
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    ips200_show_string(TUNING_PAD_X, (uint16)(y + 2U), hint);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

static void tuning_update_labels(void)
{
    snprintf(tuning_toggle_label, sizeof(tuning_toggle_label), "1. Telemetry [%s]", tuning_enabled ? "ON" : "OFF");
    snprintf(tuning_period_plus_label, sizeof(tuning_period_plus_label), "3. Period + [%ums]", (unsigned int)tuning_period_ms);
    snprintf(tuning_period_minus_label, sizeof(tuning_period_minus_label), "4. Period - [%ums]", (unsigned int)tuning_period_ms);
    snprintf(track_stream_label, sizeof(track_stream_label), "5. Track Stream [%s]", track_stream_enabled ? "ON" : "OFF");
    snprintf(track_record_label, sizeof(track_record_label), "6. Track Record [%s]", ins_record_is_recording() ? "REC" : "IDLE");
    snprintf(track_rate_label, sizeof(track_rate_label), "7. Stream Rate [%ums]", (unsigned int)track_stream_interval_ms);
}

static void tuning_set_period(uint16 period_ms)
{
    if (period_ms < TUNING_MIN_PERIOD_MS)
    {
        period_ms = TUNING_MIN_PERIOD_MS;
    }
    else if (period_ms > TUNING_MAX_PERIOD_MS)
    {
        period_ms = TUNING_MAX_PERIOD_MS;
    }

    tuning_period_ms = period_ms;
    tuning_update_labels();
}

void tuning_soft_init(void)
{
    tuning_enabled = 0U;    /* 默认关闭，WiFi 连上后由命令或菜单开启 */
    tuning_period_ms = TUNING_DEFAULT_PERIOD_MS;
    tuning_last_send_ms = system_getval_ms();
    tuning_total_sent = 0U;
    tuning_total_fail = 0U;
    tuning_view_mode = TUNING_VIEW_NONE;
    tuning_full_redraw = 1U;
    tuning_last_view_ms = 0U;
    track_stream_enabled = 0U;
    track_stream_interval_ms = 50U;
    track_stream_last_ms = system_getval_ms();
    track_dump_index = 0U;
    track_dump_active = 0U;
    autotune_init();
    tuning_update_labels();
}

static void tuning_enter_view(tuning_view_mode_t mode)
{
    tuning_view_mode = mode;
    tuning_full_redraw = 1U;
    my_key_clear_state(MY_KEY_1);
}

static void tuning_exit_view(void)
{
    tuning_view_mode = TUNING_VIEW_NONE;
    tuning_full_redraw = 1U;
    my_key_clear_state(MY_KEY_1);
    menu_request_full_redraw();
}

static void tuning_action_toggle(void)
{
    tuning_enabled = tuning_enabled ? 0U : 1U;
    tuning_last_send_ms = system_getval_ms();
    tuning_update_labels();
    menu_request_full_redraw();
}

static void tuning_action_status(void)
{
    tuning_enter_view(TUNING_VIEW_STATUS);
    tuning_draw_status();
}

static void tuning_action_period_plus(void)
{
    tuning_set_period((uint16)(tuning_period_ms + TUNING_STEP_MS));
    menu_request_full_redraw();
}

static void tuning_action_period_minus(void)
{
    uint16 next_period = tuning_period_ms;
    if (next_period > TUNING_STEP_MS)
    {
        next_period = (uint16)(next_period - TUNING_STEP_MS);
    }
    tuning_set_period(next_period);
    menu_request_full_redraw();
}

static void tuning_action_track_stream_toggle(void)
{
    track_stream_enabled = track_stream_enabled ? 0U : 1U;
    track_stream_last_ms = system_getval_ms();
    tuning_update_labels();
    menu_request_full_redraw();
}

static void tuning_action_track_record_toggle(void)
{
    if (ins_record_is_recording())
    {
        ins_record_stop();
    }
    else
    {
        ins_record_start();
    }
    tuning_update_labels();
    menu_request_full_redraw();
}

static void tuning_action_track_rate_cycle(void)
{
    if (track_stream_interval_ms >= TRACK_STREAM_MAX_MS)
    {
        track_stream_interval_ms = TRACK_STREAM_MIN_MS;
    }
    else
    {
        track_stream_interval_ms = (uint16)(track_stream_interval_ms + TRACK_STREAM_STEP_MS);
    }
    tuning_update_labels();
    menu_request_full_redraw();
}

static uint8 tuning_send_line(const char *line)
{
    uint32 remain;

    if (NULL == line)
    {
        return 0U;
    }

    remain = wifi_spi_send_buffer((const uint8 *)line, (uint32)strlen(line));
    if (0U == remain)
    {
        tuning_total_sent++;
        return 1U;
    }

    tuning_total_fail++;
    return 0U;
}

static uint8 tuning_send_once(void)
{
    char line[TUNING_SEND_BUFFER_LEN];
    uint32 now_ms = system_getval_ms();
    float gyro_x = icm42688_gyro_x;
    float gyro_y = icm42688_gyro_y;
    float gyro_z = icm42688_gyro_z;
    float acc_x = icm42688_acc_x;
    float acc_y = icm42688_acc_y;
    float acc_z = icm42688_acc_z;
    float gyro_bias_x = 0.0f;
    float gyro_bias_y = 0.0f;
    float gyro_bias_z = 0.0f;
    float gyro_corr_x = 0.0f;
    float gyro_corr_y = 0.0f;
    float gyro_corr_z = 0.0f;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    float acc_norm_g = 0.0f;
    float gyro_norm = 0.0f;
    float nx = 0.0f;
    float ny = 0.0f;
    float nz = 0.0f;
    uint32 attitude_update_count = 0U;
    uint32 bias_sample_count = 0U;
    uint32 bias_target_count = 0U;
    uint8 ok = 1U;

    icm_attitude_get_gyro_bias(&gyro_bias_x, &gyro_bias_y, &gyro_bias_z);
    icm_attitude_get_gyro_bias_calibration_progress(&bias_sample_count, &bias_target_count);
    icm_attitude_get_euler(&roll_deg, &pitch_deg, &yaw_deg);
    icm_attitude_get_quaternion(&q0, &q1, &q2, &q3);
    acc_norm_g = icm_attitude_get_acc_norm_g();
    attitude_update_count = icm_attitude_get_update_count();

    gyro_corr_x = gyro_x - gyro_bias_x;
    gyro_corr_y = gyro_y - gyro_bias_y;
    gyro_corr_z = gyro_z - gyro_bias_z;
    gyro_norm = sqrtf(gyro_corr_x * gyro_corr_x + gyro_corr_y * gyro_corr_y + gyro_corr_z * gyro_corr_z);

    nx = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * acc_x
       + 2.0f * (q1 * q2 - q0 * q3) * acc_y
       + 2.0f * (q1 * q3 + q0 * q2) * acc_z;
    ny = 2.0f * (q1 * q2 + q0 * q3) * acc_x
       + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * acc_y
       + 2.0f * (q2 * q3 - q0 * q1) * acc_z;
    nz = 2.0f * (q1 * q3 - q0 * q2) * acc_x
       + 2.0f * (q2 * q3 + q0 * q1) * acc_y
       + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * acc_z - 1.0f;

    snprintf(line,
             sizeof(line),
             "TELG,ms=%lu,fix=%d,sat=%d,spd=%.2f,enc=%d,gx=%.3f,gy=%.3f,gz=%.3f,gcx=%.3f,gcy=%.3f,gcz=%.3f,gxyz=%.3f,gbx=%.3f,gby=%.3f,gbz=%.3f,bias_ok=%u,bias_cal=%u,bias_n=%lu,bias_t=%lu,bias_flash=%u\r\n",
             (unsigned long)now_ms,
             gnss.state,
             gnss.satellite_used,
             gnss.speed,
             switch_encoder_num,
             gyro_x,
             gyro_y,
             gyro_z,
             gyro_corr_x,
             gyro_corr_y,
             gyro_corr_z,
             gyro_norm,
             gyro_bias_x,
             gyro_bias_y,
             gyro_bias_z,
             (unsigned int)icm_attitude_is_gyro_bias_valid(),
             (unsigned int)icm_attitude_is_gyro_bias_calibrating(),
             (unsigned long)bias_sample_count,
             (unsigned long)bias_target_count,
             (unsigned int)icm_attitude_is_gyro_bias_from_flash());
    if (!tuning_send_line(line))
    {
        ok = 0U;
    }

    snprintf(line,
             sizeof(line),
             "TELA,ms=%lu,roll=%.2f,pitch=%.2f,yaw=%.2f,q0=%.5f,q1=%.5f,q2=%.5f,q3=%.5f,ax=%.4f,ay=%.4f,az=%.4f,anorm=%.4f,nx=%.4f,ny=%.4f,nz=%.4f,att_upd=%lu,lat=%.6f,lon=%.6f\r\n",
             (unsigned long)now_ms,
             roll_deg,
             pitch_deg,
             yaw_deg,
             q0,
             q1,
             q2,
             q3,
             acc_x,
             acc_y,
             acc_z,
             acc_norm_g,
             nx,
             ny,
             nz,
             (unsigned long)attitude_update_count,
             gnss.latitude,
             gnss.longitude);
    if (!tuning_send_line(line))
    {
        ok = 0U;
    }

    {
        float ins_px = 0.0f;
        float ins_py = 0.0f;
        float ins_vx = 0.0f;
        float ins_vy = 0.0f;

        icm_ins_get_position(&ins_px, &ins_py);
        icm_ins_get_velocity(&ins_vx, &ins_vy, NULL);

        snprintf(line,
                 sizeof(line),
                 "TELINS,ms=%lu,ins_rec=%u,ins_pts=%u,px=%.3f,py=%.3f,vx=%.3f,vy=%.3f,yaw=%.2f,stat=%u\r\n",
                 (unsigned long)now_ms,
                 (unsigned int)ins_record_is_recording(),
                 (unsigned int)ins_record_get_point_count(),
                 ins_px,
                 ins_py,
                 ins_vx,
                 ins_vy,
                 yaw_deg,
                 (unsigned int)icm_ins_is_stationary());
        if (!tuning_send_line(line))
        {
            ok = 0U;
        }
    }

    {
        const ins_ctrl_output_t *ctrl = ins_ctrl_get_output();

        snprintf(line,
                 sizeof(line),
                 "TELPLAY,ms=%lu,play=%u,tgt_idx=%u,tgt_px=%.3f,tgt_py=%.3f"
                 ",tgt_dist=%.3f,tgt_yaw=%.2f,yaw_err=%.2f,spd=%.3f,drv=%u\r\n",
                 (unsigned long)now_ms,
                 (unsigned int)ins_playback_is_running(),
                 (unsigned int)ins_playback_get_target_idx(),
                 ctrl->tgt_px_m,
                 ctrl->tgt_py_m,
                 ctrl->tgt_dist_m,
                 ctrl->tgt_yaw_deg,
                 ctrl->yaw_err_deg,
                 ctrl->speed_cmd,
                 (unsigned int)ctrl->drive_enable);
        if (!tuning_send_line(line))
        {
            ok = 0U;
        }
    }

    /* ---- TELY：yaw 专用调参遥测包 ---- */
    snprintf(line,
             sizeof(line),
             "TELTURN,ms=%lu,target=%.3f,current=%.3f,encoder=%ld,enable=%u\r\n",
             (unsigned long)now_ms,
             turn_target_angle_deg,
             turn_current_angle_deg,
             (long)Turn_GetEncoderCount(),
             (unsigned int)Turn_GetMotorEnable());
    if (!tuning_send_line(line))
    {
        ok = 0U;
    }

    snprintf(line,
             sizeof(line),
             "TELENR,ms=%lu,count=%ld,dist_mm=%ld,spd_mm_s=%ld,"
             "odom_px=%.3f,odom_py=%.3f,odom_spd=%.3f,active=%u\r\n",
             (unsigned long)now_ms,
             (long)rear_right_get_count(),
             (long)rear_right_get_dist_mm(),
             (long)rear_right_get_spd_mm_s(),
             encoder_odom_right_get_px_m(),
             encoder_odom_right_get_py_m(),
             encoder_odom_right_get_speed_ms(),
             (unsigned int)encoder_odom_right_is_active());
    if (!tuning_send_line(line))
    {
        ok = 0U;
    }

    {
        float ins_px = 0.0f;
        float ins_py = 0.0f;

        icm_ins_get_position(&ins_px, &ins_py);
        snprintf(line,
                 sizeof(line),
                 "TELODOM,ms=%lu,"
                 "L_cnt=%ld,L_dst=%ld,L_spd=%ld,L_act=%u,"
                 "R_cnt=%ld,R_dst=%ld,R_spd=%ld,R_act=%u,"
                 "ins_px=%.3f,ins_py=%.3f,ins_spd=%.3f,"
                 "L_px=%.3f,L_py=%.3f,R_px=%.3f,R_py=%.3f\r\n",
                 (unsigned long)now_ms,
                 (long)rear_left_get_count(),
                 (long)rear_left_get_dist_mm(),
                 (long)rear_left_get_spd_mm_s(),
                 (unsigned int)encoder_odom_is_active(),
                 (long)rear_right_get_count(),
                 (long)rear_right_get_dist_mm(),
                 (long)rear_right_get_spd_mm_s(),
                 (unsigned int)encoder_odom_right_is_active(),
                 ins_px,
                 ins_py,
                 icm_ins_get_speed_ms(),
                 encoder_odom_get_px_m(),
                 encoder_odom_get_py_m(),
                 encoder_odom_right_get_px_m(),
                 encoder_odom_right_get_py_m());
        if (!tuning_send_line(line))
        {
            ok = 0U;
        }
    }

    snprintf(line,
             sizeof(line),
             "TELBRK,ms=%lu,a24=%u,filt=%u,pct=%u,pressed=%u,valid=%u,cmd=%u,"
             "thr_cmd=%u,thr_en=%u,brake_overrides=%u\r\n",
             (unsigned long)now_ms,
             (unsigned int)pedal_input_get_a24(),
             (unsigned int)pedal_input_get_brake_filtered(),
             (unsigned int)pedal_input_get_brake_percent(),
             (unsigned int)pedal_input_get_brake_pressed(),
             (unsigned int)pedal_input_get_brake_valid(),
             (unsigned int)pedal_input_get_brake_cmd(),
             (unsigned int)pedal_input_get_throttle_cmd(),
             (unsigned int)pedal_input_get_throttle_enable_request(),
             (unsigned int)(pedal_input_get_brake_pressed() && pedal_input_get_brake_valid()));
    if (!tuning_send_line(line))
    {
        ok = 0U;
    }

    {
        float gz_raw = 0.0f, gz_bias = 0.0f, gz_comp = 0.0f;
        float gz_scaled = 0.0f, yaw_integral = 0.0f;
        float yaw_final = 0.0f, yaw_corr = 0.0f, yaw_dt = 0.0f;
        uint8  zero_busy = 0U, zero_ok = 0U;
        uint32 zero_n = 0U;
        float  zero_gbz = 0.0f;

        icm_attitude_yaw_get_debug(&gz_raw, &gz_bias, &gz_comp,
                                    &gz_scaled, &yaw_integral,
                                    &yaw_final, &yaw_corr, &yaw_dt);
        icm_attitude_yaw_zero_get_state(&zero_busy, &zero_ok,
                                         &zero_n, &zero_gbz);

        snprintf(line,
                 sizeof(line),
                 "TELY,ms=%lu"
                 ",gzr=%.3f,gzb=%.3f,gzc=%.3f"
                 ",gzs=%.4f,gzd=%.3f"
                 ",yi=%.2f,yf=%.2f,yc=%.4f"
                 ",dt=%lu"
                 ",dbg=%u,ver=%lu"
                 ",ton=%u,tst=%.1f,tdl=%.1f"
                 ",zb=%u,zok=%u,zn=%lu,zgz=%.3f\r\n",
                 (unsigned long)now_ms,
                 gz_raw, gz_bias, gz_comp,
                 icm_attitude_yaw_get_scale(), gz_scaled,
                 yaw_integral, yaw_final, yaw_corr,
                 (unsigned long)(uint32)(yaw_dt * 1000000.0f),
                 (unsigned int)icm_attitude_yaw_get_debug_enable(),
                 (unsigned long)icm_attitude_yaw_get_param_version(),
                 (unsigned int)icm_attitude_yaw_test_is_on(),
                 icm_attitude_yaw_test_get_start(),
                 icm_attitude_yaw_test_get_last_delta(),
                 (unsigned int)zero_busy, (unsigned int)zero_ok,
                 (unsigned long)zero_n, zero_gbz);
        if (!tuning_send_line(line))
        {
            ok = 0U;
        }
    }

    return ok;
}

static void tuning_send_track_point(void)
{
    char line[TUNING_SEND_BUFFER_LEN];
    float px = 0.0f, py = 0.0f, vx = 0.0f, vy = 0.0f;
    float roll_tmp = 0.0f, pitch_tmp = 0.0f, yaw = 0.0f;
    const icm_gps_fusion_debug_t *fdbg;

    icm_gps_fusion_get_position(&px, &py);
    icm_gps_fusion_get_velocity(&vx, &vy);
    icm_attitude_get_euler(&roll_tmp, &pitch_tmp, &yaw);
    fdbg = icm_gps_fusion_get_debug();

    snprintf(line,
             sizeof(line),
             "TELPT,ms=%lu,px=%.3f,py=%.3f,vx=%.3f,vy=%.3f"
             ",yaw=%.2f,spd=%.2f,lat=%.6f,lon=%.6f"
             ",sat=%u,gv=%u,stat=%u,fpx=%.3f,fpy=%.3f"
             ",esm=%ld,edm=%ld\r\n",
             (unsigned long)system_getval_ms(),
             px, py, vx, vy,
             yaw, gnss.speed, gnss.latitude, gnss.longitude,
             (unsigned int)gnss.satellite_used,
             (unsigned int)(fdbg ? fdbg->gps_valid : 0U),
             (unsigned int)icm_ins_is_stationary(),
             fdbg ? fdbg->gps_x_m : 0.0f,
             fdbg ? fdbg->gps_y_m : 0.0f,
             (long)rear_left_get_spd_mm_s(),
             (long)rear_left_get_dist_mm());
    tuning_send_line(line);
}

static void tuning_send_track_dump_batch(void)
{
    char line[TUNING_SEND_BUFFER_LEN];
    uint16 total = ins_record_get_point_count();
    uint16 batch_end;
    uint16 i;

    if (!track_dump_active)
    {
        return;
    }

    if (track_dump_index >= total)
    {
        track_dump_active = 0U;
        return;
    }

    /* Send header on first batch */
    if (0U == track_dump_index)
    {
        snprintf(line, sizeof(line),
                 "TELPTDUMP,count=%u\r\n",
                 (unsigned int)total);
        tuning_send_line(line);
    }

    batch_end = track_dump_index + TRACK_DUMP_BATCH;
    if (batch_end > total)
    {
        batch_end = total;
    }

    for (i = track_dump_index; i < batch_end; i++)
    {
        ins_record_point_t pt;
        if (ins_record_get_point(i, &pt))
        {
            snprintf(line,
                     sizeof(line),
                     "TELPT,ms=%lu,px=%.3f,py=%.3f,vx=%.3f,vy=%.3f"
                     ",yaw=%.2f,spd=0.00,lat=0.000000,lon=0.000000"
                     ",sat=0,gv=0,stat=0,fpx=0.000,fpy=0.000"
                     ",esm=%ld,edm=%ld\r\n",
                     (unsigned long)pt.t_ms,
                     pt.px_m, pt.py_m, pt.vx_ms, pt.vy_ms,
                     pt.yaw_deg,
                     (long)pt.enc_spd_mm_s,
                     (long)pt.enc_dist_mm);
            tuning_send_line(line);
        }
    }

    track_dump_index = batch_end;
    if (track_dump_index >= total)
    {
        track_dump_active = 0U;
    }
}

void tuning_soft_task(void)
{
    uint32 now_ms;

    /* WiFi SPI 未初始化时不能调用任何 wifi_spi 函数，否则访问未初始化外设会硬错误。
     * wifi_tcp_connected 只在 wifi_spi_init + TCP 连接成功后才为 1，
     * 此时 SPI 一定可用。连接成功后由命令自动开启 tuning_enabled。 */
    if (!wifi_menu_get_tcp_status())
    {
        return;
    }

    tuning_receive_task();

    /* Track streaming runs independently of tuning_enabled */
    now_ms = system_getval_ms();
    if (track_stream_enabled)
    {
        if ((uint32)(now_ms - track_stream_last_ms) >= track_stream_interval_ms)
        {
            track_stream_last_ms = now_ms;
            tuning_send_track_point();
        }
    }

    if (track_dump_active)
    {
        tuning_send_track_dump_batch();
    }

    if (!tuning_enabled)
    {
        return;
    }

    now_ms = system_getval_ms();
    if ((uint32)(now_ms - tuning_last_send_ms) < tuning_period_ms)
    {
        return;
    }

    tuning_last_send_ms = now_ms;
    (void)tuning_send_once();
}

static void tuning_send_response(const char *response)
{
    if (NULL == response)
    {
        return;
    }

    if (0U != wifi_spi_send_buffer((const uint8 *)response, (uint32)strlen(response)))
    {
        tuning_total_fail++;
    }
}

static void tuning_reply_error(const char *cmd, const char *reason)
{
    char response[TUNING_RESP_BUFFER_LEN];

    snprintf(response,
             sizeof(response),
             "ERR,cmd=%s,reason=%s\r\n",
             (NULL != cmd) ? cmd : "UNKNOWN",
             (NULL != reason) ? reason : "UNKNOWN");
    tuning_send_response(response);
}

static void tuning_reply_ack_pid(const char *cmd)
{
    char response[TUNING_RESP_BUFFER_LEN];
    const pid_param_t *pid_param = menu_get_pid_param();

    snprintf(response,
             sizeof(response),
             "ACK,cmd=%s,ms=%lu,kp=%.3f,ki=%.3f,kd=%.3f\r\n",
             (NULL != cmd) ? cmd : "UNKNOWN",
             (unsigned long)system_getval_ms(),
             (NULL != pid_param) ? pid_param->kp : 0.0f,
             (NULL != pid_param) ? pid_param->ki : 0.0f,
             (NULL != pid_param) ? pid_param->kd : 0.0f);
    tuning_send_response(response);
}

static void tuning_reply_autotune_status(const char *cmd)
{
    char response[TUNING_RESP_BUFFER_LEN];
    const autotune_status_t *status = autotune_get_status();

    snprintf(response,
             sizeof(response),
             "ACK,cmd=%s,state=%u,idx=%u,total=%u,best=%u,last=%.3f,bestScore=%.3f,kp=%.3f,ki=%.3f,kd=%.3f\r\n",
             (NULL != cmd) ? cmd : "AUTO_STATUS",
             (unsigned int)((NULL != status) ? status->state : 0U),
             (unsigned int)((NULL != status) ? status->candidate_index : 0U),
             (unsigned int)((NULL != status) ? status->candidate_total : 0U),
             (unsigned int)((NULL != status) ? status->best_valid : 0U),
             (NULL != status) ? status->last_score : 0.0f,
             (NULL != status) ? status->best_score : 0.0f,
             (NULL != status) ? status->current.kp : 0.0f,
             (NULL != status) ? status->current.ki : 0.0f,
             (NULL != status) ? status->current.kd : 0.0f);
    tuning_send_response(response);
}

static void tuning_process_rx_line(char *line)
{
    char cmd[16] = {0};
    pid_param_t param;
    float value = 0.0f;

    if (NULL == line)
    {
        return;
    }

    while ((*line == ' ') || (*line == '\t'))
    {
        line++;
    }

    if ('\0' == *line)
    {
        return;
    }

    if ((0 == strcmp(line, "GET PID")) || (0 == strcmp(line, "GET PID RUNTIME")))
    {
        tuning_reply_ack_pid("GET_PID_RUNTIME");
        return;
    }

    if (3 == sscanf(line, "SET PID %f %f %f", &param.kp, &param.ki, &param.kd))
    {
        if (NULL != menu_get_pid_param())
        {
            param.integral_limit = menu_get_pid_param()->integral_limit;
            param.output_limit = menu_get_pid_param()->output_limit;
        }

        if (menu_set_pid_param(&param, 0U))
        {
            tuning_reply_ack_pid("SET_PID");
        }
        else
        {
            tuning_reply_error("SET_PID", "OUT_OF_RANGE");
        }
        return;
    }

    if (2 == sscanf(line, "SET %15s %f", cmd, &value))
    {
        const pid_param_t *current = menu_get_pid_param();

        if (NULL == current)
        {
            tuning_reply_error(cmd, "PID_UNAVAILABLE");
            return;
        }

        param = *current;
        if (0 == strcmp(cmd, "KP"))
        {
            param.kp = value;
            if (menu_set_pid_param(&param, 0U))
            {
                tuning_reply_ack_pid("SET_KP");
            }
            else
            {
                tuning_reply_error("SET_KP", "OUT_OF_RANGE");
            }
            return;
        }
        if (0 == strcmp(cmd, "KI"))
        {
            param.ki = value;
            if (menu_set_pid_param(&param, 0U))
            {
                tuning_reply_ack_pid("SET_KI");
            }
            else
            {
                tuning_reply_error("SET_KI", "OUT_OF_RANGE");
            }
            return;
        }
        if (0 == strcmp(cmd, "KD"))
        {
            param.kd = value;
            if (menu_set_pid_param(&param, 0U))
            {
                tuning_reply_ack_pid("SET_KD");
            }
            else
            {
                tuning_reply_error("SET_KD", "OUT_OF_RANGE");
            }
            return;
        }

        if (0 == strcmp(cmd, "YAW_BIAS"))
        {
            char resp[TUNING_RESP_BUFFER_LEN];
            icm_attitude_yaw_set_manual_bias(value);
            snprintf(resp, sizeof(resp),
                     "ACK,cmd=SET_YAW_BIAS,val=%.4f,ver=%lu\r\n",
                     icm_attitude_yaw_get_manual_bias(),
                     (unsigned long)icm_attitude_yaw_get_param_version());
            tuning_send_response(resp);
            return;
        }
        if (0 == strcmp(cmd, "YAW_SCALE"))
        {
            char resp[TUNING_RESP_BUFFER_LEN];
            icm_attitude_yaw_set_scale(value);
            snprintf(resp, sizeof(resp),
                     "ACK,cmd=SET_YAW_SCALE,val=%.4f,ver=%lu\r\n",
                     icm_attitude_yaw_get_scale(),
                     (unsigned long)icm_attitude_yaw_get_param_version());
            tuning_send_response(resp);
            return;
        }
        if (0 == strcmp(cmd, "YAW_DBG"))
        {
            char resp[TUNING_RESP_BUFFER_LEN];
            icm_attitude_yaw_set_debug_enable((uint8)((int)value != 0));
            snprintf(resp, sizeof(resp),
                     "ACK,cmd=SET_YAW_DBG,val=%u,ver=%lu\r\n",
                     (unsigned int)icm_attitude_yaw_get_debug_enable(),
                     (unsigned long)icm_attitude_yaw_get_param_version());
            tuning_send_response(resp);
            return;
        }

        tuning_reply_error(cmd, "UNKNOWN_SET_TARGET");
        return;
    }

    if (0 == strcmp(line, "SAVE PID"))
    {
        const pid_param_t *current = menu_get_pid_param();
        if ((NULL != current) && menu_set_pid_param(current, 1U))
        {
            tuning_reply_ack_pid("SAVE_PID");
        }
        else
        {
            tuning_reply_error("SAVE_PID", "FLASH_SAVE_FAILED");
        }
        return;
    }

    if (0 == strcmp(line, "AUTO STATUS"))
    {
        tuning_reply_autotune_status("AUTO_STATUS");
        return;
    }

    if (0 == strcmp(line, "AUTO STOP"))
    {
        autotune_stop();
        tuning_reply_autotune_status("AUTO_STOP");
        return;
    }

    if (0 == strcmp(line, "AUTO APPLY BEST"))
    {
        if (autotune_apply_best(1U))
        {
            tuning_reply_autotune_status("AUTO_APPLY_BEST");
        }
        else
        {
            tuning_reply_error("AUTO_APPLY_BEST", "BEST_UNAVAILABLE");
        }
        return;
    }

    if (0 == strcmp(line, "AUTO START"))
    {
        const pid_param_t *current = menu_get_pid_param();
        if ((NULL != current) && autotune_start(current))
        {
            tuning_reply_autotune_status("AUTO_START");
        }
        else
        {
            tuning_reply_error("AUTO_START", "START_FAILED");
        }
        return;
    }

    if (3 == sscanf(line, "AUTO STEP %f %f %f", &param.kp, &param.ki, &param.kd))
    {
        autotune_set_steps(param.kp, param.ki, param.kd);
        tuning_reply_autotune_status("AUTO_STEP");
        return;
    }

    if (1 == sscanf(line, "AUTO SCORE %f", &value))
    {
        if (autotune_submit_score(value))
        {
            tuning_reply_autotune_status("AUTO_SCORE");
        }
        else
        {
            tuning_reply_error("AUTO_SCORE", "STATE_INVALID");
        }
        return;
    }

    /* ================================================================
     *  YAW 调参命令
     * ================================================================ */

    if (0 == strcmp(line, "GET YAW"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        uint8  zero_busy = 0U, zero_ok = 0U;
        uint32 zero_n = 0U;
        float  zero_gbz = 0.0f;
        float  yaw_final = 0.0f;
        icm_attitude_yaw_zero_get_state(&zero_busy, &zero_ok, &zero_n, &zero_gbz);
        icm_attitude_yaw_get_debug(NULL, NULL, NULL, NULL, NULL, &yaw_final, NULL, NULL);
        snprintf(resp, sizeof(resp),
                 "ACK,cmd=GET_YAW,ms=%lu"
                 ",yaw_bias=%.4f,yaw_scale=%.4f"
                 ",yaw_dbg=%u,yaw_param_ver=%lu"
                 ",yaw_test_on=%u,yaw_test_start=%.2f,yaw_test_last_delta=%.2f"
                 ",yaw_zero_busy=%u,yaw_zero_ok=%u"
                 ",yaw_zero_n=%lu,yaw_zero_gbz=%.4f"
                 ",yaw_final=%.2f\r\n",
                 (unsigned long)system_getval_ms(),
                 icm_attitude_yaw_get_manual_bias(),
                 icm_attitude_yaw_get_scale(),
                 (unsigned int)icm_attitude_yaw_get_debug_enable(),
                 (unsigned long)icm_attitude_yaw_get_param_version(),
                 (unsigned int)icm_attitude_yaw_test_is_on(),
                 icm_attitude_yaw_test_get_start(),
                 icm_attitude_yaw_test_get_last_delta(),
                 (unsigned int)zero_busy, (unsigned int)zero_ok,
                 (unsigned long)zero_n, zero_gbz,
                 yaw_final);
        tuning_send_response(resp);
        return;
    }

    if (0 == strcmp(line, "YAW ZERO"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        icm_attitude_yaw_zero_start(1000U);  /* 采样 1000 个点，@1kHz 约 1s */
        snprintf(resp, sizeof(resp),
                 "ACK,cmd=YAW_ZERO,busy=1,target=1000\r\n");
        tuning_send_response(resp);
        return;
    }

    if (0 == strcmp(line, "YAW TEST START"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        icm_attitude_yaw_test_start();
        snprintf(resp, sizeof(resp),
                 "ACK,cmd=YAW_TEST_START,start=%.2f\r\n",
                 icm_attitude_yaw_test_get_start());
        tuning_send_response(resp);
        return;
    }

    if (0 == strcmp(line, "YAW TEST STOP"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        float start_deg = 0.0f, end_deg = 0.0f, delta = 0.0f, err90 = 0.0f;
        icm_attitude_yaw_test_stop(&start_deg, &end_deg, &delta, &err90);
        /* YAWTEST 结果包 */
        snprintf(resp, sizeof(resp),
                 "YAWTEST,start=%.2f,end=%.2f,delta=%.2f,err90=%.2f\r\n",
                 start_deg, end_deg, delta, err90);
        tuning_send_response(resp);
        return;
    }

    /* ---- yaw zero 完成时的异步通知由 TELY 携带，此处额外提供查询 ---- */
    if (0 == strcmp(line, "YAW ZERO STATUS"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        uint8  busy = 0U, ok = 0U;
        uint32 n = 0U;
        float  gbz = 0.0f;
        icm_attitude_yaw_zero_get_state(&busy, &ok, &n, &gbz);
        snprintf(resp, sizeof(resp),
                 "YAWCAL,busy=%u,ok=%u,n=%lu,gbz=%.4f,ver=%lu\r\n",
                 (unsigned int)busy, (unsigned int)ok,
                 (unsigned long)n, gbz,
                 (unsigned long)icm_attitude_yaw_get_param_version());
        tuning_send_response(resp);
        return;
    }

    /* ================================================================
     *  TRACK 轨迹流 / 记录 / 导出命令
     * ================================================================ */

    if (0 == strcmp(line, "TRACK STREAM ON"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        track_stream_enabled = 1U;
        track_stream_last_ms = system_getval_ms();
        tuning_update_labels();
        snprintf(resp, sizeof(resp),
                 "ACK,cmd=TRACK_STREAM_ON,ms=%lu,interval=%u\r\n",
                 (unsigned long)system_getval_ms(),
                 (unsigned int)track_stream_interval_ms);
        tuning_send_response(resp);
        return;
    }

    if (0 == strcmp(line, "TRACK STREAM OFF"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        track_stream_enabled = 0U;
        tuning_update_labels();
        snprintf(resp, sizeof(resp),
                 "ACK,cmd=TRACK_STREAM_OFF,ms=%lu\r\n",
                 (unsigned long)system_getval_ms());
        tuning_send_response(resp);
        return;
    }

    if (0 == strcmp(line, "TRACK RECORD START"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        ins_record_start();
        tuning_update_labels();
        snprintf(resp, sizeof(resp),
                 "ACK,cmd=TRACK_RECORD_START,ms=%lu,pts=%u\r\n",
                 (unsigned long)system_getval_ms(),
                 (unsigned int)ins_record_get_point_count());
        tuning_send_response(resp);
        return;
    }

    if (0 == strcmp(line, "TRACK RECORD STOP"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        ins_record_stop();
        tuning_update_labels();
        snprintf(resp, sizeof(resp),
                 "ACK,cmd=TRACK_RECORD_STOP,ms=%lu,pts=%u\r\n",
                 (unsigned long)system_getval_ms(),
                 (unsigned int)ins_record_get_point_count());
        tuning_send_response(resp);
        return;
    }

    if (0 == strcmp(line, "TRACK DUMP"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        uint16 pts = ins_record_get_point_count();
        if (0U == pts)
        {
            tuning_reply_error("TRACK_DUMP", "NO_POINTS");
            return;
        }
        track_dump_index = 0U;
        track_dump_active = 1U;
        snprintf(resp, sizeof(resp),
                 "ACK,cmd=TRACK_DUMP,ms=%lu,count=%u\r\n",
                 (unsigned long)system_getval_ms(),
                 (unsigned int)pts);
        tuning_send_response(resp);
        return;
    }

    if (0 == strcmp(line, "TRACK CLEAR"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        ins_record_clear();
        track_dump_active = 0U;
        track_dump_index = 0U;
        tuning_update_labels();
        snprintf(resp, sizeof(resp),
                 "ACK,cmd=TRACK_CLEAR,ms=%lu\r\n",
                 (unsigned long)system_getval_ms());
        tuning_send_response(resp);
        return;
    }

    tuning_reply_error("UNKNOWN", "UNKNOWN_COMMAND");
}

static void tuning_receive_task(void)
{
    uint8 raw[TUNING_RECV_BUFFER_LEN];
    uint32 raw_len = 0U;
    static char line_buf[TUNING_RECV_BUFFER_LEN];
    static uint32 line_len = 0U;
    uint32 i;

    raw_len = wifi_spi_read_buffer(raw, sizeof(raw));
    if (0U == raw_len)
    {
        return;
    }

    /* 首次收到数据 → 自动开启遥测（用户不需手动到 Tuning 菜单开） */
    if (!tuning_enabled)
    {
        tuning_enabled = 1U;
        tuning_update_labels();
    }

    for (i = 0U; i < raw_len; i++)
    {
        char ch = (char)raw[i];

        if ('\r' == ch)
        {
            continue;
        }

        if ('\n' == ch)
        {
            line_buf[line_len] = '\0';
            if (line_len > 0U)
            {
                tuning_process_rx_line(line_buf);
            }
            line_len = 0U;
            continue;
        }

        if (line_len < (sizeof(line_buf) - 1U))
        {
            line_buf[line_len++] = ch;
        }
        else
        {
            line_len = 0U;
            tuning_reply_error("UNKNOWN", "LINE_TOO_LONG");
        }
    }
}

static void tuning_draw_status(void)
{
    char line[64];
    uint32 now_ms = system_getval_ms();
    const pid_param_t *pid_param = menu_get_pid_param();

    if (!tuning_full_redraw && (uint32)(now_ms - tuning_last_view_ms) < 200U)
    {
        return;
    }
    tuning_last_view_ms = now_ms;

    if (tuning_full_redraw)
    {
        tuning_fill_rect(0, 0, (uint16)(ips200_width_max - 1U), (uint16)(ips200_height_max - 1U), RGB565_BLACK);
        tuning_draw_header("Tuning Status");
        tuning_draw_footer("LONG K1: Back");
        tuning_full_redraw = 0U;
    }

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    snprintf(line, sizeof(line), "Telemetry: %s", tuning_enabled ? "ON" : "OFF");
    tuning_show_padded(TUNING_PAD_X, TUNING_LINE_START, line);

    snprintf(line, sizeof(line), "Period: %ums TCP:%s", (unsigned int)tuning_period_ms, wifi_menu_get_tcp_status() ? "OK" : "DOWN");
    tuning_show_padded(TUNING_PAD_X, (uint16)(TUNING_LINE_START + TUNING_LINE_H), line);

    snprintf(line, sizeof(line), "Sent:%lu Fail:%lu", (unsigned long)tuning_total_sent, (unsigned long)tuning_total_fail);
    tuning_show_padded(TUNING_PAD_X, (uint16)(TUNING_LINE_START + TUNING_LINE_H * 2U), line);

    snprintf(line, sizeof(line), "Fix:%d Sat:%d Spd:%.2f", gnss.state, gnss.satellite_used, gnss.speed);
    tuning_show_padded(TUNING_PAD_X, (uint16)(TUNING_LINE_START + TUNING_LINE_H * 3U), line);

    snprintf(line, sizeof(line), "Enc:%d Step:%d", switch_encoder_num, switch_encoder_change_num);
    tuning_show_padded(TUNING_PAD_X, (uint16)(TUNING_LINE_START + TUNING_LINE_H * 4U), line);

    snprintf(line, sizeof(line), "PID: %.3f %.3f %.3f",
             (NULL != pid_param) ? pid_param->kp : 0.0f,
             (NULL != pid_param) ? pid_param->ki : 0.0f,
             (NULL != pid_param) ? pid_param->kd : 0.0f);
    tuning_show_padded(TUNING_PAD_X, (uint16)(TUNING_LINE_START + TUNING_LINE_H * 5U), line);
}

uint8 tuning_soft_is_active(void)
{
    return (TUNING_VIEW_NONE != tuning_view_mode) ? 1U : 0U;
}

uint8 tuning_soft_handle_view(void)
{
    if (TUNING_VIEW_NONE == tuning_view_mode)
    {
        return 0U;
    }

    if (MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1))
    {
        my_key_clear_state(MY_KEY_1);
        tuning_exit_view();
        return 1U;
    }

    if (TUNING_VIEW_STATUS == tuning_view_mode)
    {
        tuning_draw_status();
    }

    return 1U;
}
