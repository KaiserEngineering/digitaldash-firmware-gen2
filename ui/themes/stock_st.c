/*
 * stock_st.c
 *
 *  Created on: Feb 19, 2025
 *      Author: Matthew Kaiser
 */

#include "themes.h"
#include "lib_pid.h"

LV_IMG_DECLARE(ui_img_gauge200_png);    // assets/gauge200.png
LV_IMG_DECLARE(ui_img_needle200_png);    // assets/needle200.png

static float tmpVal = 45;

static void event_cb(lv_event_t * e)
{
	PID_DATA * data = lv_event_get_user_data(e);
    lv_obj_t * needle = lv_event_get_target(e);
    lv_obj_t * value = lv_obj_get_child(needle, 0);
    lv_label_set_text_fmt(value, "%.1f", data->pid_value);
}

lv_obj_t * add_stock_st_gauge( int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid)
{
	lv_obj_t * gauge;
	gauge = lv_image_create(parent);
    lv_image_set_src(gauge, &ui_img_gauge200_png);
    lv_obj_set_width(gauge, LV_SIZE_CONTENT);   /// 125
    lv_obj_set_height(gauge, LV_SIZE_CONTENT);    /// 125
    lv_obj_set_x(gauge, x);
    lv_obj_set_y(gauge, y);
    lv_obj_set_align(gauge, LV_ALIGN_CENTER);
    lv_obj_remove_flag(gauge, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                       LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                       LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags

    lv_obj_t * needle;
    needle = lv_image_create(gauge);
    lv_image_set_src(needle, &ui_img_needle200_png);
    lv_obj_set_width(needle, LV_SIZE_CONTENT);   /// 125
    lv_obj_set_height(needle, LV_SIZE_CONTENT);    /// 125
    lv_obj_set_x(needle, 0);
    lv_obj_set_y(needle, 0);
    lv_obj_set_align(needle, LV_ALIGN_CENTER);
    lv_obj_set_user_data(needle, pid);
    lv_obj_add_event_cb(needle, event_cb, LV_EVENT_REFRESH, pid);
    lv_obj_remove_flag(needle, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                       LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                       LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags

    lv_obj_t * value;
    value = lv_label_create(needle);
    lv_obj_set_width(value, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(value, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(value, 0);
    lv_obj_set_y(value, -55);
    lv_obj_set_align(value, LV_ALIGN_CENTER);
    lv_label_set_text(value, "15°F");
    lv_obj_remove_flag(value, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                       LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                       LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_obj_set_style_text_font(value, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);

    return needle;
}
