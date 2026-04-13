/*********************************************************************************************************************
* File: menu_ui_utils.h
* Brief: 菜单系统共享 UI 工具函数
* Author: JX116
*********************************************************************************************************************/

#ifndef __MENU_UI_UTILS_H__
#define __MENU_UI_UTILS_H__

#include "zf_common_typedef.h"

/* ---------- 显示工具 ---------- */

/* 在 (x,y) 处显示字符串，截断以不超出屏幕宽度 */
void menu_ui_show_fit(uint16 x, uint16 y, const char *s);

/* 在 (x,y) 处显示字符串，截断到 max_width 像素宽度 */
void menu_ui_show_fit_width(uint16 x, uint16 y, uint16 max_width, const char *s);

/* 在 (x,y) 处显示字符串，用空格填充到 max_width 像素宽度 */
void menu_ui_show_pad(uint16 x, uint16 y, uint16 max_width, const char *s);

/* 矩形填充（水平扫描线） */
void menu_ui_fill_rect(uint16 x0, uint16 y0, uint16 x1, uint16 y1, uint16 color);

/* 页面标题栏：黄色文字 + 灰色分隔线 */
void menu_ui_draw_page_header(const char *title, uint16 title_y, uint16 sep_y);

/* ---------- 按键与退出检测 ---------- */

/* 消费 KEY_1 的按键状态和长按标志 */
void menu_ui_consume_key1(void);

/* GPIO 长按退出检测。调用者传入自己的计时器变量指针。返回 1 表示触发退出 */
uint8 menu_ui_check_exit_hold(uint32 *key_press_start_ms, uint32 hold_ms);

/* ---------- 字符串工具 ---------- */

/* 安全字符串拷贝（不超出 dest_size） */
void menu_ui_copy_text(char *dest, uint32 dest_size, const char *source);

/* 标签更新：比较新旧文本，若不同则拷贝并返回 1，否则返回 0 */
uint8 menu_ui_update_label(char *label_buf, uint32 buf_size, const char *new_text);

/* ---------- 显示视图上下文 ---------- */

typedef struct {
    uint8  *mode;
    uint8  *menu_full_redraw;
    void   (*drain_encoder_events)(void);
    void   (*request_redraw)(uint8 full_redraw);
    void   (*reset_dynamic_region)(void);
} menu_view_ctx_t;

/* 通用进入视图：设置 mode、清按键、排空编码器、请求全量重绘 */
void menu_view_enter(menu_view_ctx_t *ctx, uint8 new_mode);

/* ---------- 步进循环与参数调节 ---------- */

typedef struct {
    const void *steps;          /* 指向 const 步进值数组 */
    uint8       step_count;     /* 数组元素个数 */
    uint8       step_index;     /* 当前索引（可变） */
    uint8       value_type;     /* 0 = float, 1 = uint16, 2 = uint8 */
} menu_step_desc_t;

/* 切换到下一个步进值（循环） */
void menu_step_cycle(menu_step_desc_t *desc);

/* 获取当前步进值（转为 float） */
float menu_step_get_float(const menu_step_desc_t *desc);

/* 获取当前步进值（转为 uint32） */
uint32 menu_step_get_uint(const menu_step_desc_t *desc);

/* 调节 float 参数，带上下限钳位 */
void menu_param_adjust_float(float *ptr, float step, float min_val, float max_val, uint8 is_add);

/* 调节 uint32 参数，带上下限钳位 */
void menu_param_adjust_uint32(uint32 *ptr, uint32 step, uint32 min_val, uint32 max_val, uint8 is_add);

#endif /* __MENU_UI_UTILS_H__ */
