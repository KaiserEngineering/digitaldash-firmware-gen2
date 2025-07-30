/*
 * digital.c
 *
 *  Created on: Jul 29, 2025
 *      Author: Matthew Kaiser
 */

#include "ui.h"
#include "lib_pid.h"


static void event_cb(lv_event_t * e)
{
    PID_DATA * data = (PID_DATA *)lv_event_get_param(e);
    lv_obj_t * guage = lv_event_get_target(e);
    lv_obj_t * span_group = lv_obj_get_child(guage, 0);
    lv_span_t * span_val = lv_spangroup_get_child(span_group, 0);
    lv_obj_t * minmax = lv_obj_get_child(guage, 1);

    // Update text value immediately
    char value_buf[16];
    snprintf(value_buf, sizeof(value_buf), float_only[data->precision], data->pid_value);
    lv_span_set_text(span_val, value_buf);
    lv_spangroup_refresh(span_group);

    // Update min/max label
    if( minmax != NULL )
    	lv_label_set_text_fmt(minmax, two_float_with_slash[data->precision], data->pid_min, data->pid_max);
}

lv_obj_t * add_digital_gauge( int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, PID_DATA * pid)
{
	lv_obj_t * gauge = lv_obj_create(parent);
    lv_obj_remove_style_all(gauge);
    lv_obj_set_width(gauge, w);
    lv_obj_set_height(gauge, h);
    lv_obj_set_x(gauge, x);
    lv_obj_set_y(gauge, y);
    lv_obj_set_align(gauge, LV_ALIGN_TOP_MID);
    lv_obj_remove_flag(gauge, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_user_data(gauge, pid);
    lv_obj_add_event_cb(gauge, event_cb, LV_EVENT_REFRESH, pid);
	#if UI_CONTAINER_DEBUG
    lv_obj_set_style_border_width(gauge, 2, 0);                    // Thickness of the outline
    lv_obj_set_style_border_color(gauge, lv_color_white(), 0);    // White outline
    lv_obj_set_style_border_opa(gauge, LV_OPA_COVER, 0);          // Fully opaque
	#endif

    // Create span group for value and unit
    lv_obj_t * span_group = lv_spangroup_create(gauge);
    lv_obj_set_size(span_group, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(span_group, LV_ALIGN_CENTER, 0, 0);
    lv_spangroup_set_align(span_group, LV_TEXT_ALIGN_CENTER);

    // Create spans
    lv_span_t * span_val = lv_spangroup_new_span(span_group);
    lv_style_set_text_font(lv_span_get_style(span_val), &lv_font_montserrat_48);
    lv_style_set_text_color(lv_span_get_style(span_val), lv_color_white());

    lv_span_t * span_unit = lv_spangroup_new_span(span_group);
    lv_style_set_text_font(lv_span_get_style(span_unit), &lv_font_montserrat_32);
    lv_style_set_text_color(lv_span_get_style(span_unit), lv_color_hex(0xBBBBBB));

    // Split value and unit (assuming value is number and unit is already stored)
    char value_buf[16];
    snprintf(value_buf, sizeof(value_buf), float_only[pid->precision], pid->pid_value);
    lv_span_set_text(span_val, value_buf);
    lv_span_set_text(span_unit, pid->unit_label);
    lv_spangroup_refresh(span_group);

    lv_obj_t * minmax = lv_label_create(gauge);
    lv_obj_set_width(minmax, LV_SIZE_CONTENT);
    lv_obj_set_height(minmax, LV_SIZE_CONTENT);
    lv_obj_align(minmax, LV_ALIGN_CENTER, 0, 40);
    lv_label_set_text(minmax, "min/max");
    lv_obj_set_style_text_font(minmax, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t * label = lv_label_create(gauge);
    lv_obj_set_width(label, LV_SIZE_CONTENT);
    lv_obj_set_height(label, LV_SIZE_CONTENT);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -40);
    lv_label_set_text(label, pid->label);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label, lv_color_hex(0xBBBBBB), LV_PART_MAIN);

    return gauge;
}
