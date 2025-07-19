/*
 * stock_st.c
 *
 *  Created on: Feb 19, 2025
 *      Author: Matthew Kaiser
 */

#include "ui.h"
#include "lib_pid.h"

LV_IMG_DECLARE(ui_img_grumpy_png);

#define GRUMPY_START_POS 92
#define GRUMPY_END_POS 0

static void event_cb(lv_event_t * e)
{
	// Get the PID data
	PID_DATA * data = (PID_DATA *)lv_event_get_param(e);
    lv_obj_t * needle = lv_event_get_target(e);
    lv_obj_t * value = lv_obj_get_child(needle, 0);

    // Calculate the needle angle
    if( data->pid_value >= data->upper_limit )
    	lv_obj_set_y(needle, GRUMPY_END_POS);
    else if ( data->pid_value <= data->lower_limit )
    	lv_obj_set_y(needle, GRUMPY_START_POS);
    else {
    	lv_obj_set_y(needle, GRUMPY_START_POS + ((data->pid_value - data->lower_limit) / (data->upper_limit - data->lower_limit)) * (GRUMPY_END_POS - GRUMPY_START_POS));
    }

    // Update the numbers
    switch( data->precision )
    {
		case 2:
			lv_label_set_text_fmt(value, "%.2f%s", data->pid_value, data->unit_label);
			break;

		case 1:
			lv_label_set_text_fmt(value, "%.1f%s", data->pid_value, data->unit_label);
			break;

		case 0:
		default:
			lv_label_set_text_fmt(value, "%.0f%s", data->pid_value, data->unit_label);
			break;
    }
}

lv_obj_t * add_grumpy_cat_gauge( int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid)
{
	lv_obj_t * needle;
    needle = lv_image_create(parent);
    lv_image_set_src(needle, &ui_img_grumpy_png);
    lv_obj_set_width(needle, LV_SIZE_CONTENT);   /// 130
    lv_obj_set_height(needle, LV_SIZE_CONTENT);    /// 130
    lv_obj_set_x(needle, x);
    lv_obj_set_y(needle, y);
    lv_obj_set_align(needle, LV_ALIGN_TOP_MID);
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
    lv_label_set_text(value, "");
    lv_obj_remove_flag(value, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                       LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                       LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_obj_set_style_text_font(value, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);

    return needle;
}
