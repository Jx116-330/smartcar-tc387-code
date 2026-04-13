/*********************************************************************************************************************
* File: menu_ui_utils.c
* Brief: 菜单系统共享 UI 工具函数实现
* Author: JX116
*********************************************************************************************************************/

#include "menu_ui_utils.h"
#include "zf_device_ips200.h"
#include "zf_driver_gpio.h"
#include "zf_driver_timer.h"
#include "MyKey.h"
#include "myhead.h"
#include <string.h>

/* ========== 显示工具 ========== */

void menu_ui_show_fit(uint16 x, uint16 y, const char *s)
{
    int max_chars = (int)((ips200_width_max - 1 - x) / 8);
    char buf[64];
    int i = 0;

    if (max_chars <= 0) return;

    while (i < max_chars && s[i] != '\0' && i < (int)sizeof(buf) - 1)
    {
        buf[i] = s[i];
        i++;
    }
    buf[i] = '\0';
    ips200_show_string(x, y, buf);
}

void menu_ui_show_fit_width(uint16 x, uint16 y, uint16 max_width, const char *s)
{
    int max_chars;
    char buf[64];
    int i = 0;

    if (0U == max_width) return;

    max_chars = (int)(max_width / 8U);
    if (max_chars <= 0) return;

    while (i < max_chars && s[i] != '\0' && i < (int)sizeof(buf) - 1)
    {
        buf[i] = s[i];
        i++;
    }
    buf[i] = '\0';
    ips200_show_string(x, y, buf);
}

void menu_ui_show_pad(uint16 x, uint16 y, uint16 max_width, const char *s)
{
    char buf[96];
    int max_chars = (int)(max_width / 8U);
    int i = 0;

    if (max_chars <= 0) return;
    if (max_chars > (int)(sizeof(buf) - 1)) max_chars = (int)(sizeof(buf) - 1);

    while ((i < max_chars) && (NULL != s) && (s[i] != '\0'))
    {
        buf[i] = s[i];
        i++;
    }
    while (i < max_chars)
    {
        buf[i++] = ' ';
    }
    buf[i] = '\0';
    ips200_show_string(x, y, buf);
}

void menu_ui_fill_rect(uint16 x0, uint16 y0, uint16 x1, uint16 y1, uint16 color)
{
    uint16 y;
    for (y = y0; y <= y1; y++)
    {
        ips200_draw_line(x0, y, x1, y, color);
    }
}

void menu_ui_draw_page_header(const char *title, uint16 title_y, uint16 sep_y)
{
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    menu_ui_show_fit(10, title_y, title);
    ips200_draw_line(0, sep_y, ips200_width_max - 1, sep_y, RGB565_GRAY);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

/* ========== 按键与退出检测 ========== */

void menu_ui_consume_key1(void)
{
    my_key_clear_state(MY_KEY_1);
    key_long_press_flag[MY_KEY_1] = close_status;
}

uint8 menu_ui_check_exit_hold(uint32 *key_press_start_ms, uint32 hold_ms)
{
    uint32 now_ms = system_getval_ms();

    if (MY_KEY_RELEASE_LEVEL != gpio_get_level(P20_2))
    {
        if (0U == *key_press_start_ms)
        {
            *key_press_start_ms = now_ms;
        }
        else if ((now_ms - *key_press_start_ms) >= hold_ms)
        {
            *key_press_start_ms = 0U;
            return 1U;
        }
    }
    else
    {
        *key_press_start_ms = 0U;
    }

    return 0U;
}

/* ========== 字符串工具 ========== */

void menu_ui_copy_text(char *dest, uint32 dest_size, const char *source)
{
    uint32 index = 0U;
    const char *text = (NULL != source) ? source : "";

    if ((NULL == dest) || (0U == dest_size))
    {
        return;
    }

    while ((index + 1U) < dest_size && text[index] != '\0')
    {
        dest[index] = text[index];
        index++;
    }
    dest[index] = '\0';
}

uint8 menu_ui_update_label(char *label_buf, uint32 buf_size, const char *new_text)
{
    if ((NULL == label_buf) || (NULL == new_text))
    {
        return 0U;
    }

    if (0 != strcmp(label_buf, new_text))
    {
        menu_ui_copy_text(label_buf, buf_size, new_text);
        return 1U;
    }

    return 0U;
}

/* ========== 显示视图上下文 ========== */

void menu_view_enter(menu_view_ctx_t *ctx, uint8 new_mode)
{
    if (NULL == ctx) return;

    if (NULL != ctx->mode) *(ctx->mode) = new_mode;

    menu_ui_consume_key1();

    if (NULL != ctx->drain_encoder_events) ctx->drain_encoder_events();
    if (NULL != ctx->menu_full_redraw) *(ctx->menu_full_redraw) = 1U;
    if (NULL != ctx->reset_dynamic_region) ctx->reset_dynamic_region();
    if (NULL != ctx->request_redraw) ctx->request_redraw(1U);
}

/* ========== 步进循环与参数调节 ========== */

void menu_step_cycle(menu_step_desc_t *desc)
{
    if (NULL == desc) return;
    desc->step_index = (uint8)((desc->step_index + 1U) % desc->step_count);
}

float menu_step_get_float(const menu_step_desc_t *desc)
{
    if ((NULL == desc) || (NULL == desc->steps)) return 0.0f;

    switch (desc->value_type)
    {
        case 0U: return ((const float *)desc->steps)[desc->step_index];
        case 1U: return (float)((const uint16 *)desc->steps)[desc->step_index];
        case 2U: return (float)((const uint8 *)desc->steps)[desc->step_index];
        default: return 0.0f;
    }
}

uint32 menu_step_get_uint(const menu_step_desc_t *desc)
{
    if ((NULL == desc) || (NULL == desc->steps)) return 0U;

    switch (desc->value_type)
    {
        case 0U: return (uint32)((const float *)desc->steps)[desc->step_index];
        case 1U: return (uint32)((const uint16 *)desc->steps)[desc->step_index];
        case 2U: return (uint32)((const uint8 *)desc->steps)[desc->step_index];
        default: return 0U;
    }
}

void menu_param_adjust_float(float *ptr, float step, float min_val, float max_val, uint8 is_add)
{
    if (NULL == ptr) return;

    if (is_add)
    {
        *ptr = (*ptr + step <= max_val) ? (*ptr + step) : max_val;
    }
    else
    {
        *ptr = (*ptr - step >= min_val) ? (*ptr - step) : min_val;
    }
}

void menu_param_adjust_uint32(uint32 *ptr, uint32 step, uint32 min_val, uint32 max_val, uint8 is_add)
{
    if (NULL == ptr) return;

    if (is_add)
    {
        *ptr = ((*ptr + step) <= max_val) ? (*ptr + step) : max_val;
    }
    else
    {
        *ptr = (*ptr >= (min_val + step)) ? (*ptr - step) : min_val;
    }
}
