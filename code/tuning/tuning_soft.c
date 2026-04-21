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
#include "ins_enc_tune.h"
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

/* tuning_period_ms: default streaming period applied when a stream is started
 *   without an explicit period, and used when the on-car menu toggle turns
 *   Phase-1 streams on. Each active stream tracks its own period in the
 *   registry below.
 *
 * Streams follow an on-demand subscription model driven by desktop commands:
 *   START STREAM <name> [period_ms] / STOP STREAM <name> / STOP ALL STREAMS /
 *   GET ONCE <name> / LIST STREAMS / GET STREAMS.
 * On TCP disconnect, all subscriptions are cleared automatically. */
static uint16 tuning_period_ms = TUNING_DEFAULT_PERIOD_MS;
static uint32 tuning_total_sent = 0U;
static uint32 tuning_total_fail = 0U;
static tuning_view_mode_t tuning_view_mode = TUNING_VIEW_NONE;
static uint8 tuning_full_redraw = 1U;
static uint32 tuning_last_view_ms = 0U;
static uint8 tuning_tcp_was_connected = 0U;   /* 1 once TCP saw first up-edge */

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
static uint8 tuning_send_line(const char *line);
static void tuning_send_track_point(void);
static void tuning_send_track_dump_batch(void);
static void tuning_send_response(const char *response);
static void tuning_reply_error(const char *cmd, const char *reason);
static void tuning_reply_ack_pid(const char *cmd);
static void tuning_process_rx_line(char *line);
static void tuning_receive_task(void);

/* ============================================================================
 * Stream registry — on-demand subscription model
 * ---------------------------------------------------------------------------
 *   Desktop tool drives all continuous telemetry via explicit subscriptions.
 *   Each registered stream has a send_fn that formats + sends one packet.
 *   tuning_soft_task() iterates the registry and calls send_fn whenever
 *   (now_ms - last_send_ms) >= period_ms for enabled streams.
 *
 *   Migrating a new stream in a future round:
 *     1. Write a static void send_telXYZ(void) that emits one line.
 *     2. Add { "XYZ", 0U, default_period, default_period, 0U, send_telXYZ }
 *        to g_streams[]. No other plumbing is required.
 * ========================================================================== */

typedef void (*tuning_stream_send_fn)(void);

typedef struct
{
    const char *name;                  /* stream short name, upper-case */
    volatile uint8  enabled;           /* 1 while subscribed            */
    volatile uint16 period_ms;         /* active period                 */
    uint16 default_period_ms;          /* default on START w/o period   */
    uint32 last_send_ms;               /* last time send_fn ran         */
    tuning_stream_send_fn send_fn;
} tuning_stream_t;

#define TUNING_STREAM_MIN_PERIOD_MS   5U
#define TUNING_STREAM_MAX_PERIOD_MS   5000U

/* Per-stream builders (defined below) */
static void tuning_send_telinsenc(void);
static void tuning_send_tely(void);
static void tuning_send_telins(void);

static tuning_stream_t g_streams[] = {
    { "INSENC", 0U, TUNING_DEFAULT_PERIOD_MS, TUNING_DEFAULT_PERIOD_MS, 0U, tuning_send_telinsenc },
    { "YAW",    0U, TUNING_DEFAULT_PERIOD_MS, TUNING_DEFAULT_PERIOD_MS, 0U, tuning_send_tely      },
    { "INS",    0U, TUNING_DEFAULT_PERIOD_MS, TUNING_DEFAULT_PERIOD_MS, 0U, tuning_send_telins    },
};
#define TUNING_STREAM_COUNT ((uint8)(sizeof(g_streams)/sizeof(g_streams[0])))

static tuning_stream_t *tuning_stream_find(const char *name);
static uint8 tuning_streams_any_enabled(void);
static void  tuning_streams_clear_all(void);
static void  tuning_streams_tick(uint32 now_ms);
static uint8 tuning_stream_start(tuning_stream_t *s, uint16 period_ms);
static void  tuning_stream_stop(tuning_stream_t *s);

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
    /* "Telemetry" reflects whether any Phase-1 stream is currently active. */
    snprintf(tuning_toggle_label, sizeof(tuning_toggle_label),
             "1. Telemetry [%s]", tuning_streams_any_enabled() ? "ON" : "OFF");
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
    /* All streams start disabled; desktop owns subscriptions. Menu toggle
     * below provides an on-car convenience to enable all Phase-1 streams. */
    tuning_streams_clear_all();
    tuning_period_ms = TUNING_DEFAULT_PERIOD_MS;
    tuning_total_sent = 0U;
    tuning_total_fail = 0U;
    tuning_view_mode = TUNING_VIEW_NONE;
    tuning_full_redraw = 1U;
    tuning_last_view_ms = 0U;
    tuning_tcp_was_connected = 0U;
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
    /* On-car convenience: ON enables every registered Phase-1 stream at the
     * current tuning_period_ms; OFF clears all subscriptions. */
    if (tuning_streams_any_enabled())
    {
        tuning_streams_clear_all();
    }
    else
    {
        uint8 i;
        for (i = 0U; i < TUNING_STREAM_COUNT; i++)
        {
            (void)tuning_stream_start(&g_streams[i], tuning_period_ms);
        }
    }
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

/* ---- INS stream: INS position/velocity/yaw/stationary (TELINS) ---- */
static void tuning_send_telins(void)
{
    char line[TUNING_SEND_BUFFER_LEN];
    float ins_px = 0.0f, ins_py = 0.0f;
    float ins_vx = 0.0f, ins_vy = 0.0f;
    float yaw_deg = 0.0f;

    icm_ins_get_position(&ins_px, &ins_py);
    icm_ins_get_velocity(&ins_vx, &ins_vy, NULL);
    icm_attitude_get_euler(NULL, NULL, &yaw_deg);

    snprintf(line,
             sizeof(line),
             "TELINS,ms=%lu,ins_rec=%u,ins_pts=%u,px=%.3f,py=%.3f,vx=%.3f,vy=%.3f,yaw=%.2f,stat=%u\r\n",
             (unsigned long)system_getval_ms(),
             (unsigned int)ins_record_is_recording(),
             (unsigned int)ins_record_get_point_count(),
             ins_px,
             ins_py,
             ins_vx,
             ins_vy,
             yaw_deg,
             (unsigned int)icm_ins_is_stationary());
    (void)tuning_send_line(line);
}

/* ---- YAW stream: yaw-axis tuning telemetry (TELY) ---- */
static void tuning_send_tely(void)
{
    char line[TUNING_SEND_BUFFER_LEN];
    float gz_raw = 0.0f, gz_bias = 0.0f, gz_comp = 0.0f;
    float gz_scaled = 0.0f, yaw_integral = 0.0f;
    float yaw_final = 0.0f, yaw_corr = 0.0f, yaw_dt = 0.0f;
    uint8  zero_busy = 0U, zero_ok = 0U;
    uint32 zero_n = 0U;
    float  zero_gbz = 0.0f;

    icm_attitude_yaw_get_debug(&gz_raw, &gz_bias, &gz_comp,
                               &gz_scaled, &yaw_integral,
                               &yaw_final, &yaw_corr, &yaw_dt);
    icm_attitude_yaw_zero_get_state(&zero_busy, &zero_ok, &zero_n, &zero_gbz);

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
             (unsigned long)system_getval_ms(),
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
    (void)tuning_send_line(line);
}

/* ---- INSENC stream: INS<->encoder fusion gains + live state (TELINSENC) ---- */
static void tuning_send_telinsenc(void)
{
    char line[TUNING_SEND_BUFFER_LEN];
    float ins_px = 0.0f, ins_py = 0.0f;
    float ins_vx = 0.0f, ins_vy = 0.0f;

    icm_ins_get_position(&ins_px, &ins_py);
    icm_ins_get_velocity(&ins_vx, &ins_vy, NULL);

    snprintf(line,
             sizeof(line),
             "TELINSENC,ms=%lu"
             ",L_vg=%.5f,L_pg=%.5f,L_sr=%.5f"
             ",R_vg=%.5f,R_pg=%.5f,R_sr=%.5f"
             ",ver=%lu,fls=%u,syn=%u"
             ",L_spd=%.3f,R_spd=%.3f,ins_spd=%.3f"
             ",L_px=%.3f,L_py=%.3f,R_px=%.3f,R_py=%.3f"
             ",ins_px=%.3f,ins_py=%.3f,ins_vx=%.3f,ins_vy=%.3f"
             ",L_act=%u,R_act=%u,stat=%u\r\n",
             (unsigned long)system_getval_ms(),
             ins_enc_tune_get_l_vel_gain(),
             ins_enc_tune_get_l_pos_gain(),
             ins_enc_tune_get_l_sync_rate(),
             ins_enc_tune_get_r_vel_gain(),
             ins_enc_tune_get_r_pos_gain(),
             ins_enc_tune_get_r_sync_rate(),
             (unsigned long)ins_enc_tune_get_param_version(),
             (unsigned int)ins_enc_tune_is_from_flash(),
             (unsigned int)ins_enc_tune_is_flash_synced(),
             encoder_odom_get_speed_ms(),
             encoder_odom_right_get_speed_ms(),
             icm_ins_get_speed_ms(),
             encoder_odom_get_px_m(),
             encoder_odom_get_py_m(),
             encoder_odom_right_get_px_m(),
             encoder_odom_right_get_py_m(),
             ins_px, ins_py, ins_vx, ins_vy,
             (unsigned int)encoder_odom_is_active(),
             (unsigned int)encoder_odom_right_is_active(),
             (unsigned int)icm_ins_is_stationary());
    (void)tuning_send_line(line);
}

/* ---- Stream registry helpers ---- */

static tuning_stream_t *tuning_stream_find(const char *name)
{
    uint8 i;
    if (NULL == name) return NULL;
    for (i = 0U; i < TUNING_STREAM_COUNT; i++)
    {
        if (0 == strcmp(name, g_streams[i].name))
        {
            return &g_streams[i];
        }
    }
    return NULL;
}

static uint8 tuning_streams_any_enabled(void)
{
    uint8 i;
    for (i = 0U; i < TUNING_STREAM_COUNT; i++)
    {
        if (g_streams[i].enabled) return 1U;
    }
    return 0U;
}

static void tuning_streams_clear_all(void)
{
    uint8 i;
    for (i = 0U; i < TUNING_STREAM_COUNT; i++)
    {
        g_streams[i].enabled = 0U;
    }
}

static void tuning_streams_tick(uint32 now_ms)
{
    uint8 i;
    for (i = 0U; i < TUNING_STREAM_COUNT; i++)
    {
        tuning_stream_t *s = &g_streams[i];
        uint16 period;
        if (!s->enabled) continue;
        period = s->period_ms;
        if ((uint32)(now_ms - s->last_send_ms) >= (uint32)period)
        {
            s->last_send_ms = now_ms;
            s->send_fn();
        }
    }
}

static uint8 tuning_stream_start(tuning_stream_t *s, uint16 period_ms)
{
    if (NULL == s) return 0U;
    if (0U == period_ms) period_ms = s->default_period_ms;
    if (period_ms < TUNING_STREAM_MIN_PERIOD_MS) period_ms = TUNING_STREAM_MIN_PERIOD_MS;
    if (period_ms > TUNING_STREAM_MAX_PERIOD_MS) period_ms = TUNING_STREAM_MAX_PERIOD_MS;
    s->period_ms   = period_ms;
    s->last_send_ms = system_getval_ms();
    s->enabled     = 1U;
    return 1U;
}

static void tuning_stream_stop(tuning_stream_t *s)
{
    if (NULL == s) return;
    s->enabled = 0U;
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
    uint8  tcp_up;

    /* WiFi SPI 未初始化时不能调用任何 wifi_spi 函数，否则访问未初始化外设会硬错误。
     * wifi_tcp_connected 只在 wifi_spi_init + TCP 连接成功后才为 1，
     * 此时 SPI 一定可用。 */
    tcp_up = wifi_menu_get_tcp_status() ? 1U : 0U;
    if (!tcp_up)
    {
        /* TCP just went down -> clear all subscriptions so the car does not
         * resume pushing to a stale peer on next reconnect. */
        if (tuning_tcp_was_connected)
        {
            tuning_streams_clear_all();
            track_stream_enabled = 0U;
            track_dump_active    = 0U;
            tuning_tcp_was_connected = 0U;
            tuning_update_labels();
        }
        return;
    }
    tuning_tcp_was_connected = 1U;

    tuning_receive_task();

    /* Track streaming remains on its own subscription (legacy TRACK STREAM
     * ON/OFF) — it will be folded into the stream registry in a later phase. */
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

    /* Dispatch subscribed streams. */
    tuning_streams_tick(now_ms);
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

static void tuning_reply_insenc(const char *cmd)
{
    char response[TUNING_RESP_BUFFER_LEN];
    snprintf(response, sizeof(response),
             "ACK,cmd=%s,ms=%lu"
             ",L_vg=%.5f,L_pg=%.5f,L_sr=%.5f"
             ",R_vg=%.5f,R_pg=%.5f,R_sr=%.5f"
             ",ver=%lu,fls=%u,syn=%u\r\n",
             (NULL != cmd) ? cmd : "INSENC",
             (unsigned long)system_getval_ms(),
             ins_enc_tune_get_l_vel_gain(),
             ins_enc_tune_get_l_pos_gain(),
             ins_enc_tune_get_l_sync_rate(),
             ins_enc_tune_get_r_vel_gain(),
             ins_enc_tune_get_r_pos_gain(),
             ins_enc_tune_get_r_sync_rate(),
             (unsigned long)ins_enc_tune_get_param_version(),
             (unsigned int)ins_enc_tune_is_from_flash(),
             (unsigned int)ins_enc_tune_is_flash_synced());
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

    /* ================================================================
     *  INSENC —— 惯导/编码器融合增益调参
     * ================================================================ */

    if (0 == strcmp(line, "GET INSENC"))
    {
        tuning_reply_insenc("GET_INSENC");
        return;
    }

    if (0 == strcmp(line, "SAVE INSENC"))
    {
        if (ins_enc_tune_save_to_flash())
        {
            tuning_reply_insenc("SAVE_INSENC");
        }
        else
        {
            tuning_reply_error("SAVE_INSENC", "FLASH_SAVE_FAILED");
        }
        return;
    }

    if (0 == strcmp(line, "LOAD INSENC"))
    {
        if (ins_enc_tune_load_from_flash())
        {
            tuning_reply_insenc("LOAD_INSENC");
        }
        else
        {
            tuning_reply_error("LOAD_INSENC", "FLASH_INVALID");
        }
        return;
    }

    if (0 == strcmp(line, "RESET INSENC"))
    {
        ins_enc_tune_reset_to_default();
        tuning_reply_insenc("RESET_INSENC");
        return;
    }

    /* SET INSENC_<KEY> <float>
     *   KEY ∈ { L_VG, L_PG, L_SR, R_VG, R_PG, R_SR } */
    {
        char key[16] = {0};
        float value = 0.0f;
        if (2 == sscanf(line, "SET INSENC_%15s %f", key, &value))
        {
            uint8 ok = 0U;
            if      (0 == strcmp(key, "L_VG")) ok = ins_enc_tune_set_l_vel_gain(value);
            else if (0 == strcmp(key, "L_PG")) ok = ins_enc_tune_set_l_pos_gain(value);
            else if (0 == strcmp(key, "L_SR")) ok = ins_enc_tune_set_l_sync_rate(value);
            else if (0 == strcmp(key, "R_VG")) ok = ins_enc_tune_set_r_vel_gain(value);
            else if (0 == strcmp(key, "R_PG")) ok = ins_enc_tune_set_r_pos_gain(value);
            else if (0 == strcmp(key, "R_SR")) ok = ins_enc_tune_set_r_sync_rate(value);
            else
            {
                tuning_reply_error("SET_INSENC", "UNKNOWN_KEY");
                return;
            }

            if (ok)
            {
                char ack_cmd[32];
                snprintf(ack_cmd, sizeof(ack_cmd), "SET_INSENC_%s", key);
                tuning_reply_insenc(ack_cmd);
            }
            else
            {
                tuning_reply_error("SET_INSENC", "OUT_OF_RANGE");
            }
            return;
        }
    }

    /* ================================================================
     *  STREAM control (on-demand telemetry subscription model)
     *
     *    LIST STREAMS                  -> names of all registered streams
     *    GET STREAMS                   -> name=enabled/period for each
     *    START STREAM <name> [period]  -> enable (optional explicit period)
     *    STOP STREAM <name>            -> disable a single stream
     *    STOP ALL STREAMS              -> disable everything
     *    GET ONCE <name>               -> emit one packet, no subscription
     * ================================================================ */
    if (0 == strcmp(line, "LIST STREAMS"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        char names[128];
        uint8 i;
        size_t off = 0;
        names[0] = '\0';
        for (i = 0U; i < TUNING_STREAM_COUNT; i++)
        {
            off += (size_t)snprintf(names + off,
                                    (off < sizeof(names)) ? (sizeof(names) - off) : 0U,
                                    "%s%s",
                                    (i == 0U) ? "" : ",",
                                    g_streams[i].name);
            if (off >= sizeof(names)) break;
        }
        snprintf(resp, sizeof(resp),
                 "ACK,cmd=LIST_STREAMS,count=%u,names=%s\r\n",
                 (unsigned int)TUNING_STREAM_COUNT, names);
        tuning_send_response(resp);
        return;
    }

    if (0 == strcmp(line, "GET STREAMS"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        char body[192];
        uint8 i;
        size_t off = 0;
        body[0] = '\0';
        for (i = 0U; i < TUNING_STREAM_COUNT; i++)
        {
            off += (size_t)snprintf(body + off,
                                    (off < sizeof(body)) ? (sizeof(body) - off) : 0U,
                                    ",%s=%u/%u",
                                    g_streams[i].name,
                                    (unsigned int)g_streams[i].enabled,
                                    (unsigned int)g_streams[i].period_ms);
            if (off >= sizeof(body)) break;
        }
        snprintf(resp, sizeof(resp),
                 "ACK,cmd=GET_STREAMS,count=%u%s\r\n",
                 (unsigned int)TUNING_STREAM_COUNT, body);
        tuning_send_response(resp);
        return;
    }

    if (0 == strcmp(line, "STOP ALL STREAMS"))
    {
        char resp[TUNING_RESP_BUFFER_LEN];
        tuning_streams_clear_all();
        tuning_update_labels();
        snprintf(resp, sizeof(resp),
                 "ACK,cmd=STOP_ALL_STREAMS,active=0\r\n");
        tuning_send_response(resp);
        return;
    }

    /* START STREAM <NAME> [period_ms] */
    {
        char name[16] = {0};
        unsigned int period_u = 0U;
        int n = sscanf(line, "START STREAM %15s %u", name, &period_u);
        if (n >= 1)
        {
            tuning_stream_t *s = tuning_stream_find(name);
            char resp[TUNING_RESP_BUFFER_LEN];
            if (NULL == s)
            {
                tuning_reply_error("START_STREAM", "UNKNOWN_STREAM");
                return;
            }
            (void)tuning_stream_start(s, (n >= 2) ? (uint16)period_u : 0U);
            tuning_update_labels();
            snprintf(resp, sizeof(resp),
                     "ACK,cmd=START_STREAM,name=%s,enabled=1,period=%u\r\n",
                     s->name, (unsigned int)s->period_ms);
            tuning_send_response(resp);
            return;
        }
    }

    /* STOP STREAM <NAME> */
    {
        char name[16] = {0};
        if (1 == sscanf(line, "STOP STREAM %15s", name))
        {
            tuning_stream_t *s = tuning_stream_find(name);
            char resp[TUNING_RESP_BUFFER_LEN];
            if (NULL == s)
            {
                tuning_reply_error("STOP_STREAM", "UNKNOWN_STREAM");
                return;
            }
            tuning_stream_stop(s);
            tuning_update_labels();
            snprintf(resp, sizeof(resp),
                     "ACK,cmd=STOP_STREAM,name=%s,enabled=0\r\n",
                     s->name);
            tuning_send_response(resp);
            return;
        }
    }

    /* GET ONCE <NAME> — fire one packet, leave subscription state untouched */
    {
        char name[16] = {0};
        if (1 == sscanf(line, "GET ONCE %15s", name))
        {
            tuning_stream_t *s = tuning_stream_find(name);
            char resp[TUNING_RESP_BUFFER_LEN];
            if (NULL == s)
            {
                tuning_reply_error("GET_ONCE", "UNKNOWN_STREAM");
                return;
            }
            s->send_fn();
            snprintf(resp, sizeof(resp),
                     "ACK,cmd=GET_ONCE,name=%s\r\n", s->name);
            tuning_send_response(resp);
            return;
        }
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

    /* Note: the old "auto-enable telemetry on first RX byte" behavior was
     * intentionally removed. Telemetry is now strictly on-demand, driven by
     * START STREAM / GET ONCE commands from the desktop. */

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
    {
        uint8 cnt = 0U;
        uint8 i;
        for (i = 0U; i < TUNING_STREAM_COUNT; i++)
        {
            if (g_streams[i].enabled) cnt++;
        }
        snprintf(line, sizeof(line), "Streams: %u/%u active", (unsigned int)cnt,
                 (unsigned int)TUNING_STREAM_COUNT);
    }
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
