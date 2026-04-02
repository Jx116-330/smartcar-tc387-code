/*********************************************************************************************************************
* File: wifi_menu.c
* Brief: WiFi 菜单模块实现
*        菜单结构：
*          WiFi
*          ├── 1. Net Info     — IP / MAC / 固件版本 / TCP 连接状态（实时刷新，无闪烁）
*          ├── 2. Send Test    — 向服务器发送测试报文，屏幕显示结果
*          ├── 3. Config       — 只读展示编译期配置（SSID / 目标IP / 端口）
*          └── 4. Reconnect    — 断开并重新建立 TCP 连接
*        操作：K1 短按确认/进入，K1 长按退出视图返回上级菜单
*********************************************************************************************************************/

#include "wifi_menu.h"
#include "zf_common_headfile.h"
#include "zf_device_ips200.h"
#include "zf_device_wifi_spi.h"
#include "MyKey.h"

/* ----------------------------- 宏 ----------------------------- */
#define WIFI_TCP_TARGET_IP      "192.168.137.1"
#define WIFI_TCP_TARGET_PORT    "8080"
#define WIFI_LOCAL_PORT         "6666"
#define WIFI_SSID               "cx330"
#define WIFI_PASSWORD           "12345678"

#define TITLE_H     30U
#define LINE_H      20U
#define LINE_START  38U
#define PAD_X       8U
#define FOOTER_H    20U

/* 数据区宽度：用于 padded 覆写，消除闪烁 */
#define DATA_WIDTH  ((uint16)(ips200_width_max - PAD_X - 4U))

/* ----------------------------- 类型 ----------------------------- */
typedef enum
{
    WIFI_VIEW_NONE = 0U,
    WIFI_VIEW_NET_INFO,
    WIFI_VIEW_CONNECT,
    WIFI_VIEW_SEND_TEST,
    WIFI_VIEW_CONFIG,
    WIFI_VIEW_RECONNECT,
} wifi_view_mode_t;

/* ----------------------------- 状态 ----------------------------- */
static wifi_view_mode_t wifi_view_mode  = WIFI_VIEW_NONE;
static uint8            wifi_tcp_connected = 0U;
static uint8            wifi_full_redraw   = 1U;
static uint8            wifi_operation_cancelled = 0U;
static uint32           wifi_last_key_scan_ms    = 0U;

/* ----------------------------- 前向声明 ----------------------------- */
static void wifi_fill_rect(uint16 x0, uint16 y0, uint16 x1, uint16 y1, uint16 color);
static void wifi_show_padded(uint16 x, uint16 y, const char *s);
static void wifi_draw_header(const char *title);
static void wifi_draw_footer(const char *hint);
static void wifi_enter_view(wifi_view_mode_t mode);
static void wifi_exit_view(void);
static void wifi_begin_blocking_operation(void);
static void wifi_end_blocking_operation(void);
static uint8 wifi_wait_cancel_hook(void);
static void wifi_action_net_info(void);
static void wifi_action_connect(void);
static void wifi_action_send_test(void);
static void wifi_action_config(void);
static void wifi_action_reconnect(void);
static void wifi_draw_net_info(void);
static void wifi_draw_connect(uint8 phase);
static void wifi_draw_send_test(uint8 result_valid, uint8 result_ok);
static void wifi_draw_config(void);
static void wifi_draw_reconnect(uint8 phase);

/* =========================================================
 * 工具函数
 * ========================================================= */

/* 填充矩形区域（用黑色清屏某块区域） */
static void wifi_fill_rect(uint16 x0, uint16 y0, uint16 x1, uint16 y1, uint16 color)
{
    uint16 y;
    for (y = y0; y <= y1; y++)
    {
        ips200_draw_line(x0, y, x1, y, color);
    }
}

/* 写字符串并用空格补齐到数据区宽度——覆写旧内容，无需清屏，消除闪烁 */
static void wifi_show_padded(uint16 x, uint16 y, const char *s)
{
    int    max_chars;
    char   buf[48];
    int    i;

    max_chars = (int)(DATA_WIDTH / 8U);
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

static void wifi_draw_header(const char *title)
{
    wifi_fill_rect(0, 0, (uint16)(ips200_width_max - 1U), (uint16)(TITLE_H - 1U), RGB565_BLACK);
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(PAD_X, 8U, title);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_draw_line(0, TITLE_H, (uint16)(ips200_width_max - 1U), TITLE_H, RGB565_GRAY);
}

static void wifi_draw_footer(const char *hint)
{
    uint16 y = (uint16)(ips200_height_max - FOOTER_H);
    wifi_fill_rect(0, y, (uint16)(ips200_width_max - 1U), (uint16)(ips200_height_max - 1U), RGB565_BLACK);
    ips200_draw_line(0, y, (uint16)(ips200_width_max - 1U), y, RGB565_GRAY);
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    ips200_show_string(PAD_X, (uint16)(y + 2U), hint);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

static void wifi_begin_blocking_operation(void)
{
    wifi_operation_cancelled = 0U;
    wifi_last_key_scan_ms    = system_getval_ms();
    my_key_clear_state(MY_KEY_1);
    wifi_spi_set_wait_hook(wifi_wait_cancel_hook);
}

static void wifi_end_blocking_operation(void)
{
    wifi_spi_set_wait_hook(NULL);
}

static uint8 wifi_wait_cancel_hook(void)
{
    uint32 now_ms = system_getval_ms();

    if ((uint32)(now_ms - wifi_last_key_scan_ms) >= 10U)
    {
        wifi_last_key_scan_ms = now_ms;
        my_key_scanner();
    }

    if (MY_KEY_LONG_PRESS == my_key_get_state(MY_KEY_1))
    {
        wifi_operation_cancelled = 1U;
        my_key_clear_state(MY_KEY_1);
        return 1U;
    }

    return 0U;
}

/* =========================================================
 * 视图进入 / 退出
 * ========================================================= */
static void wifi_enter_view(wifi_view_mode_t mode)
{
    wifi_view_mode   = mode;
    wifi_full_redraw = 1U;
    my_key_clear_state(MY_KEY_1);
}

static void wifi_exit_view(void)
{
    wifi_view_mode   = WIFI_VIEW_NONE;
    wifi_full_redraw = 1U;
    /* 通知菜单系统需要整页重绘，否则退出后屏幕不刷新 */
    wifi_spi_set_wait_hook(NULL);
    my_key_clear_state(MY_KEY_1);
    menu_request_full_redraw();
}

/* =========================================================
 * 视图绘制
 * ========================================================= */

/*
 * Net Info 刷新策略：
 *   full_redraw=1 时：清屏 + 绘制标题/页脚 + 绘制 Ver/MAC/IP 三条静态行
 *   此后每 500ms：仅覆写 TCP 状态行（padded 字符串，不清屏，无闪烁）
 */
static void wifi_draw_net_info(void)
{
    static uint32 last_ms = 0U;
    uint32  now_ms;
    char    line[48];

    now_ms = system_getval_ms();

    if (!wifi_full_redraw && (now_ms - last_ms < 500U))
    {
        return;
    }
    last_ms = now_ms;

    if (wifi_full_redraw)
    {
        wifi_fill_rect(0, 0, (uint16)(ips200_width_max - 1U), (uint16)(ips200_height_max - 1U), RGB565_BLACK);
        wifi_draw_header("Net Info");
        wifi_draw_footer("LONG K1: Back");

        /* 静态行：版本 / MAC / IP，只在 full_redraw 时绘制 */
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);

        snprintf(line, sizeof(line), "Ver: %s", wifi_spi_version[0] ? wifi_spi_version : "---");
        wifi_show_padded(PAD_X, (uint16)(LINE_START), line);

        snprintf(line, sizeof(line), "MAC: %s", wifi_spi_mac_addr[0] ? wifi_spi_mac_addr : "---");
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H), line);

        snprintf(line, sizeof(line), "IP : %s", wifi_spi_ip_addr_port[0] ? wifi_spi_ip_addr_port : "---");
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H * 2U), line);

        wifi_full_redraw = 0U;
    }

    /* TCP 状态行：每次用 padded 覆写，颜色变化但不清屏 */
    if (wifi_tcp_connected)
    {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H * 3U), "TCP: Connected");
    }
    else
    {
        ips200_set_color(RGB565_RED, RGB565_BLACK);
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H * 3U), "TCP: Disconnected");
    }
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

/* Send Test：一次性绘制，result_valid=0 时只画 "Sending..."，=1 后显示结果 */
static void wifi_draw_send_test(uint8 result_valid, uint8 result_ok)
{
    wifi_fill_rect(0, 0, (uint16)(ips200_width_max - 1U), (uint16)(ips200_height_max - 1U), RGB565_BLACK);
    wifi_draw_header("Send Test");
    wifi_draw_footer("LONG K1: Back");

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    wifi_show_padded(PAD_X, LINE_START, "Sending: WiFi TCP Test");

    if (result_valid)
    {
        if (result_ok)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
            wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H), "Result: OK");
        }
        else
        {
            ips200_set_color(RGB565_RED, RGB565_BLACK);
            wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H), "Result: FAIL");
            ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
            wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H * 2U), "Check TCP server on PC");
        }
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    }
}

static void wifi_draw_config(void)
{
    char line[48];

    wifi_fill_rect(0, 0, (uint16)(ips200_width_max - 1U), (uint16)(ips200_height_max - 1U), RGB565_BLACK);
    wifi_draw_header("Config (Read Only)");
    wifi_draw_footer("LONG K1: Back");

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    snprintf(line, sizeof(line), "SSID    : %s", WIFI_SSID);
    wifi_show_padded(PAD_X, LINE_START, line);

    snprintf(line, sizeof(line), "Svr IP  : %s", WIFI_TCP_TARGET_IP);
    wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H), line);

    snprintf(line, sizeof(line), "Svr Port: %s", WIFI_TCP_TARGET_PORT);
    wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H * 2U), line);

    snprintf(line, sizeof(line), "Loc Port: %s", WIFI_LOCAL_PORT);
    wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H * 3U), line);
}

/* phase: 0=连接WiFi中, 1=WiFi OK连TCP中, 2=WiFi失败, 3=TCP成功, 4=TCP失败 */
static void wifi_draw_connect(uint8 phase)
{
    if (0U == phase)
    {
        wifi_fill_rect(0, 0, (uint16)(ips200_width_max - 1U), (uint16)(ips200_height_max - 1U), RGB565_BLACK);
        wifi_draw_header("Connect");
        wifi_draw_footer("LONG K1: Back");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        wifi_show_padded(PAD_X, LINE_START, "Connecting to WiFi...");
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H), WIFI_SSID);
        return;
    }

    /* 清除状态区，覆写新内容 */
    wifi_fill_rect(0, LINE_START, (uint16)(ips200_width_max - 1U), (uint16)(LINE_START + LINE_H * 4U + 16U), RGB565_BLACK);

    if (1U == phase)
    {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        wifi_show_padded(PAD_X, LINE_START, "WiFi OK!");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H), "Connecting TCP...");
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H * 2U), WIFI_TCP_TARGET_IP);
    }
    else if (2U == phase)
    {
        ips200_set_color(RGB565_RED, RGB565_BLACK);
        wifi_show_padded(PAD_X, LINE_START, "WiFi Failed!");
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H), "Check SSID/Password");
    }
    else if (3U == phase)
    {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        wifi_show_padded(PAD_X, LINE_START, "TCP Connected!");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H), "Ready to send data.");
    }
    else
    {
        ips200_set_color(RGB565_RED, RGB565_BLACK);
        wifi_show_padded(PAD_X, LINE_START, "TCP Failed!");
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H), "Check TCP server on PC");
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H * 2U), "Use Reconnect to retry");
    }
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

/* phase: 0=连接中, 1=成功, 2=失败 */
static void wifi_draw_reconnect(uint8 phase)
{
    if (0U == phase)
    {
        wifi_fill_rect(0, 0, (uint16)(ips200_width_max - 1U), (uint16)(ips200_height_max - 1U), RGB565_BLACK);
        wifi_draw_header("Reconnect");
        wifi_draw_footer("LONG K1: Back");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        wifi_show_padded(PAD_X, LINE_START, "Disconnecting...");
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H), "Connecting TCP...");
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H * 2U), WIFI_TCP_TARGET_IP);
        return;
    }

    /* 清除状态行，覆写结果 */
    wifi_fill_rect(0, LINE_START, (uint16)(ips200_width_max - 1U), (uint16)(LINE_START + LINE_H * 3U + 16U), RGB565_BLACK);

    if (1U == phase)
    {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        wifi_show_padded(PAD_X, LINE_START, "TCP Connected!");
    }
    else
    {
        ips200_set_color(RGB565_RED, RGB565_BLACK);
        wifi_show_padded(PAD_X, LINE_START, "Connect Failed!");
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        wifi_show_padded(PAD_X, (uint16)(LINE_START + LINE_H), "Check TCP server on PC");
    }
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

static void wifi_action_connect(void)
{
    uint8 ret;

    wifi_enter_view(WIFI_VIEW_CONNECT);
    wifi_draw_connect(0U);

    wifi_begin_blocking_operation();
    ret = wifi_spi_init(WIFI_SSID, WIFI_PASSWORD);
    wifi_end_blocking_operation();

    if (wifi_operation_cancelled)
    {
        wifi_exit_view();
        return;
    }

    if (0U != ret)
    {
        wifi_tcp_connected = 0U;
        wifi_draw_connect(2U);
        return;
    }

    wifi_draw_connect(1U);

    wifi_begin_blocking_operation();
    ret = wifi_spi_socket_connect("TCP", WIFI_TCP_TARGET_IP, WIFI_TCP_TARGET_PORT, WIFI_LOCAL_PORT);
    wifi_end_blocking_operation();

    if (wifi_operation_cancelled)
    {
        wifi_exit_view();
        return;
    }

    if (0U == ret)
    {
        wifi_tcp_connected = 1U;
        wifi_draw_connect(3U);
    }
    else
    {
        wifi_tcp_connected = 0U;
        wifi_draw_connect(4U);
    }
}

/* =========================================================
 * 菜单动作（K1 短按触发）
 * ========================================================= */
static void wifi_action_net_info(void)
{
    wifi_enter_view(WIFI_VIEW_NET_INFO);
}

static void wifi_action_send_test(void)
{
    const char *test_msg = "WiFi TCP Test";
    uint32      remain;

    wifi_enter_view(WIFI_VIEW_SEND_TEST);
    wifi_draw_send_test(0U, 0U);

    wifi_begin_blocking_operation();
    remain = wifi_spi_send_buffer((const uint8 *)test_msg, (uint32)strlen(test_msg));
    wifi_end_blocking_operation();

    if (wifi_operation_cancelled)
    {
        wifi_exit_view();
        return;
    }

    wifi_draw_send_test(1U, (uint8)(remain == 0U));
}

static void wifi_action_config(void)
{
    wifi_enter_view(WIFI_VIEW_CONFIG);
    wifi_draw_config();
}

static void wifi_action_reconnect(void)
{
    uint8 ret = 1U;

    wifi_enter_view(WIFI_VIEW_RECONNECT);
    wifi_draw_reconnect(0U);

    wifi_begin_blocking_operation();
    wifi_spi_socket_disconnect();
    if (!wifi_operation_cancelled)
    {
        ret = wifi_spi_socket_connect("TCP", WIFI_TCP_TARGET_IP, WIFI_TCP_TARGET_PORT, WIFI_LOCAL_PORT);
    }
    wifi_end_blocking_operation();

    if (wifi_operation_cancelled)
    {
        wifi_exit_view();
        return;
    }

    if (0U == ret)
    {
        wifi_tcp_connected = 1U;
        wifi_draw_reconnect(1U);
    }
    else
    {
        wifi_tcp_connected = 0U;
        wifi_draw_reconnect(2U);
    }
}

/* =========================================================
 * 菜单页面定义
 * ========================================================= */
static MenuItem wifi_items[] = {
    {"1. Net Info",  wifi_action_net_info,   NULL},
    {"2. Connect",   wifi_action_connect,    NULL},
    {"3. Send Test", wifi_action_send_test,  NULL},
    {"4. Config",    wifi_action_config,     NULL},
    {"5. Reconnect", wifi_action_reconnect,  NULL},
};

MenuPage wifi_page = {
    "WiFi",
    wifi_items,
    sizeof(wifi_items) / sizeof(MenuItem),
    NULL
};

/* =========================================================
 * 公开接口
 * ========================================================= */
void wifi_menu_set_tcp_status(uint8 connected)
{
    wifi_tcp_connected = connected;
}

uint8 wifi_menu_is_active(void)
{
    return (WIFI_VIEW_NONE != wifi_view_mode) ? 1U : 0U;
}

uint8 wifi_menu_get_tcp_status(void)
{
    return wifi_tcp_connected;
}

/*
 * 由 menu_task() 和 menu_execute_current_item() 调用。
 * 返回 1：视图已接管本帧，调用方直接 return。
 * 返回 0：无活跃视图，调用方正常处理。
 */
uint8 wifi_menu_handle_view(void)
{
    if (WIFI_VIEW_NONE == wifi_view_mode)
    {
        return 0U;
    }

    /* 长按 K1：退出视图，返回 WiFi 菜单页 */
    if (my_key_get_state(MY_KEY_1) == MY_KEY_LONG_PRESS)
    {
        my_key_clear_state(MY_KEY_1);
        wifi_exit_view();   /* 内部调用 menu_request_full_redraw() */
        return 1U;
    }

    /* Net Info 需要周期性刷新 TCP 状态行 */
    if (WIFI_VIEW_NET_INFO == wifi_view_mode)
    {
        wifi_draw_net_info();
    }

    /* SEND_TEST / CONFIG / RECONNECT 由对应动作函数一次性绘制，无需周期刷新 */

    return 1U;
}
