/*
 * radial.c
 *
 *  Created on: Apr 3, 2025
 *      Author: Matth
 */

#include "ui.h"

#define ARC_THICKNESS 20
#define RADIAL_START_ANGLE 180
#define RADIAL_END_ANGLE 0
#define RADIAL_SIZE 230

static void event_cb(lv_event_t * e)
{
	// Get the PID data
	PID_DATA * data = (PID_DATA *)lv_event_get_param(e);
    lv_obj_t * needle = lv_event_get_target(e);
    lv_obj_t * span_group = lv_obj_get_child(needle, 0);
    lv_span_t * span_val = lv_spangroup_get_child(span_group, 0);
    lv_obj_t * minmax = lv_obj_get_child(needle, 1);

    lv_arc_set_value(needle, scale_float(data->pid_value, data->precision));

    char value_buf[16];
    snprintf(value_buf, sizeof(value_buf), float_only[data->precision], data->pid_value);
    lv_span_set_text(span_val, value_buf);
    lv_spangroup_refresh(span_group);

    lv_label_set_text_fmt(minmax, two_float_with_slash[data->precision], data->pid_min, data->pid_max);
}

lv_obj_t * add_radial_gauge( int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid)
{
	lv_obj_t * needle = lv_arc_create(parent);
    lv_obj_set_width(needle, RADIAL_SIZE);
    lv_obj_set_height(needle, RADIAL_SIZE);
    lv_obj_set_x(needle, x);
    lv_obj_set_y(needle, y+5);
    lv_obj_set_align(needle, LV_ALIGN_TOP_MID);
    lv_obj_remove_flag(needle, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE |
                       LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC |
                       LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_arc_set_range(needle, scale_float(pid->lower_limit, pid->precision), scale_float(pid->upper_limit, pid->precision));
    lv_arc_set_value(needle, pid->pid_value);
    lv_arc_set_bg_angles(needle, RADIAL_START_ANGLE, RADIAL_END_ANGLE);
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

    // Create span group for value and unit
    lv_obj_t * span_group = lv_spangroup_create(needle);
    lv_obj_set_size(span_group, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_align(span_group, LV_ALIGN_CENTER);
    lv_obj_set_y(span_group, -50);
    lv_spangroup_set_align(span_group, LV_TEXT_ALIGN_CENTER);

    // Create spans
    lv_span_t * span_val = lv_spangroup_new_span(span_group);
    lv_style_set_text_font(lv_span_get_style(span_val), &lv_font_montserrat_30);
    lv_style_set_text_color(lv_span_get_style(span_val), lv_color_white());

    lv_span_t * span_unit = lv_spangroup_new_span(span_group);
    lv_style_set_text_font(lv_span_get_style(span_unit), &lv_font_montserrat_20);
    lv_style_set_text_color(lv_span_get_style(span_unit), lv_color_hex(0xBBBBBBB));

    // Split value and unit (assuming value is number and unit is already stored)
    char value_buf[16];
    snprintf(value_buf, sizeof(value_buf), float_only[pid->precision], pid->pid_value);
    lv_span_set_text(span_val, value_buf);
    lv_span_set_text(span_unit, pid->unit_label);
    lv_spangroup_refresh(span_group);

    /*
    lv_obj_t * value = lv_label_create(needle);
    lv_obj_set_width(value, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(value, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(value, 0);
    lv_obj_set_y(value, -50);
    lv_obj_set_align(value, LV_ALIGN_CENTER);
    lv_label_set_text_fmt(value, float_with_units[pid->precision], pid->pid_value, pid->unit_label);
    lv_obj_remove_flag(value, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                       LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                       LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_obj_set_style_text_font(value, &lv_font_montserrat_30, LV_PART_MAIN | LV_STATE_DEFAULT);
    */

    lv_obj_t * minmax = lv_label_create(needle);
    lv_obj_set_width(minmax, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(minmax, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(minmax, 0);
    lv_obj_set_y(minmax, 4);
    lv_obj_set_align(minmax, LV_ALIGN_CENTER);
    lv_label_set_text(minmax, "min/max");
    lv_obj_set_style_text_font(minmax, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t * label = lv_label_create(needle);
    lv_obj_set_width(label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(label, 0);
    lv_obj_set_y(label, -22);
    lv_obj_set_align(label, LV_ALIGN_CENTER);
    lv_label_set_text(label, pid->label);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    return needle;
}
