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
#define CONTAINER_H_EXTEND 50
#define Y_ADJUST CONTAINER_H_EXTEND/2

static void event_cb(lv_event_t * e)
{
	// Get the PID data
	GAUGE_DATA * data = (GAUGE_DATA *)lv_event_get_param(e);

    if( pid_value_changed(data) )
    {
        lv_obj_t * guage = lv_event_get_target(e);
        lv_obj_t * span_group = lv_obj_get_child(guage, 1);
        lv_span_t * span_val = lv_spangroup_get_child(span_group, 0);

		// Calculate the needle angle
		int32_t target_y;
		if( data->pid->pid_value >= data->pid->upper_limit )
			target_y = GRUMPY_END_POS;
		else if ( data->pid->pid_value <= data->pid->lower_limit )
			target_y = GRUMPY_START_POS;
		else {
			// Scale all values
			float value = (float)scale_float(data->pid->pid_value, data->pid->precision);
			float lower = (float)scale_float(data->pid->lower_limit, data->pid->precision);
			float upper = (float)scale_float(data->pid->upper_limit, data->pid->precision);

			// Normalize the value between 0.0 and 1.0
			float normalized = (value - lower) / (upper - lower);

			// Map normalized range [0,1] → gauge position
			target_y = GRUMPY_START_POS + (int32_t)(normalized * (GRUMPY_END_POS - GRUMPY_START_POS));
		}

		// Smooth toward target_y (EMA filter)
		if (data->smoothed_y == -10000) {
			data->smoothed_y = target_y;  // snap to first value
		} else {
			// adjust divisor for smoothness: bigger = slower/smoother
			data->smoothed_y += (target_y - data->smoothed_y) / 6;
		}

		// Apply smoothed value
		lv_obj_set_y(guage, data->smoothed_y);

		if( pid_value_label_changed(data) )
		{
			// Update value text
			char value_buf[16];
			snprintf(value_buf, sizeof(value_buf), float_only[data->pid->precision], data->pid->pid_value);
			lv_span_set_text(span_val, value_buf);
			lv_spangroup_refresh(span_group);
		}
    }
}

lv_obj_t * add_grumpy_cat_gauge( int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, GAUGE_DATA* data)
{
	lv_obj_t * gauge = lv_obj_create(parent);
    lv_obj_remove_style_all(gauge);
    lv_obj_set_width(gauge, w);
    lv_obj_set_height(gauge, h+CONTAINER_H_EXTEND);
    lv_obj_align(gauge, LV_ALIGN_TOP_MID, x, y);
    lv_obj_remove_flag(gauge, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_user_data(gauge, data);
    lv_obj_add_event_cb(gauge, event_cb, LV_EVENT_REFRESH, data);
	#if UI_CONTAINER_DEBUG
	lv_obj_set_style_border_width(gauge, 2, 0);                    // Thickness of the outline
	lv_obj_set_style_border_color(gauge, lv_color_white(), 0);    // White outline
	lv_obj_set_style_border_opa(gauge, LV_OPA_COVER, 0);          // Fully opaque
	#endif

	lv_obj_t * needle = lv_image_create(gauge);
    lv_image_set_src(needle, &ui_img_grumpy_png);
    lv_obj_set_width(needle, LV_SIZE_CONTENT);
    lv_obj_set_height(needle, LV_SIZE_CONTENT);
    lv_obj_set_x(needle, 0);
    lv_obj_set_y(needle, 0);
    lv_obj_set_align(needle, LV_ALIGN_TOP_MID);
    lv_obj_remove_flag(needle, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                       LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                       LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags

    // Create span group for value and unit
    lv_obj_t * span_group = lv_spangroup_create(gauge);
    lv_obj_set_size(span_group, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_align(span_group, LV_ALIGN_CENTER);
    lv_obj_set_y(span_group, -45-Y_ADJUST);
    lv_spangroup_set_align(span_group, LV_TEXT_ALIGN_CENTER);

    // Create spans
    lv_span_t * span_val = lv_spangroup_new_span(span_group);
    lv_style_set_text_font(lv_span_get_style(span_val), &lv_font_montserrat_38);
    lv_style_set_text_color(lv_span_get_style(span_val), lv_color_white());

    lv_span_t * span_unit = lv_spangroup_new_span(span_group);
    lv_style_set_text_font(lv_span_get_style(span_unit), &lv_font_montserrat_24);
    lv_style_set_text_color(lv_span_get_style(span_unit), lv_color_hex(0xBBBBBB));

    // Split value and unit (assuming value is number and unit is already stored)
    char value_buf[16];
    snprintf(value_buf, sizeof(value_buf), float_only[data->pid->precision], data->pid->pid_value);
    lv_span_set_text(span_val, value_buf);
    lv_span_set_text(span_unit, data->pid->unit_label);
    lv_spangroup_refresh(span_group);

    return gauge;
}
