/*
 * radial.c
 *
 *  Created on: Apr 3, 2025
 *      Author: Matth
 */

#include "themes.h"

#define ARC_THICKNESS 15

static void event_cb(lv_event_t * e)
{
	// Get the PID data
	PID_DATA * data = (PID_DATA *)lv_event_get_param(e);
    lv_obj_t * needle = lv_event_get_target(e);
    lv_obj_t * value = lv_obj_get_child(needle, 0);
    lv_obj_t * minmax = lv_obj_get_child(needle, 1);

    // Check if the value needs to be refreshed
    //if( lv_arc_get_value(needle) == (int32_t)data->pid_value )
    //	return;

    lv_arc_set_value(needle, data->pid_value);

    // Update the numbers
    switch( data->precision )
    {
		case 2:
			lv_label_set_text_fmt(value, "%.2f%s", data->pid_value, data->unit_label);
			lv_label_set_text_fmt(minmax, "%.2f/%.2f", data->pid_min, data->pid_max);
			break;

		case 1:
			lv_label_set_text_fmt(value, "%.1f%s", data->pid_value, data->unit_label);
			lv_label_set_text_fmt(minmax, "%.1f/%.1f", data->pid_min, data->pid_max);
			break;

		case 0:
		default:
			lv_label_set_text_fmt(value, "%.0f%s", data->pid_value, data->unit_label);
			lv_label_set_text_fmt(minmax, "%.0f/%.0f", data->pid_min, data->pid_max);
			break;
    }
}

lv_obj_t * add_radial_gauge( int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid)
{
	lv_obj_t * needle = lv_arc_create(parent);
    lv_obj_set_width(needle, 200);
    lv_obj_set_height(needle, 200);
    lv_obj_set_x(needle, x);
    lv_obj_set_y(needle, y+5);
    lv_obj_set_align(needle, LV_ALIGN_TOP_MID);
    lv_obj_remove_flag(needle, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE |
                       LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC |
                       LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_arc_set_range(needle, pid->lower_limit, pid->upper_limit);
    lv_arc_set_value(needle, pid->pid_value);
    lv_arc_set_bg_angles(needle, 175, 5);
    lv_obj_set_style_arc_color(needle, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(needle, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(needle, ARC_THICKNESS, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_rounded(needle, false, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_color(needle, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(needle, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(needle, ARC_THICKNESS, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_rounded(needle, false, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(needle, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(needle, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_user_data(needle, pid);
    lv_obj_add_event_cb(needle, event_cb, LV_EVENT_REFRESH, pid);

    lv_obj_t * value = lv_label_create(needle);
    lv_obj_set_width(value, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(value, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(value, 0);
    lv_obj_set_y(value, -48);
    lv_obj_set_align(value, LV_ALIGN_CENTER);
    lv_label_set_text(value, "value");
    lv_obj_remove_flag(value, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                       LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                       LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_obj_set_style_text_font(value, &lv_font_montserrat_30, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t * minmax = lv_label_create(needle);
    lv_obj_set_width(minmax, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(minmax, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(minmax, 0);
    lv_obj_set_y(minmax, 0);
    lv_obj_set_align(minmax, LV_ALIGN_CENTER);
    lv_label_set_text(minmax, "min/max");
    lv_obj_set_style_text_font(minmax, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t * label = lv_label_create(needle);
    lv_obj_set_width(label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(label, 0);
    lv_obj_set_y(label, -21);
    lv_obj_set_align(label, LV_ALIGN_CENTER);
    lv_label_set_text(label, pid->label);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    return needle;
}
