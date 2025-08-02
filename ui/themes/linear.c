/*
 * linear.c
 *
 *  Created on: Apr 2, 2025
 *      Author: Matth
 */


#include "ui.h"
#include "lib_pid.h"

#if UI_HOR_RES == 800
#define BAR_WIDTH 600
#define BAR_HEIGHT 50
#define BAR_LABEL_Y 30
#define BAR_FONT &lv_font_montserrat_22
#elif UI_HOR_RES == 1024
#define BAR_PADDING 100
#define BAR_Y_OFFSET 20
#define BAR_HEIGHT 50
#define BAR_FONT &lv_font_montserrat_38
#define BAR_LABEL_Y BAR_Y_OFFSET + BAR_HEIGHT + 5
#endif

static void event_cb(lv_event_t * e)
{
    PID_DATA * data = (PID_DATA *)lv_event_get_param(e);
    lv_obj_t * gauge = lv_event_get_target(e);
    lv_obj_t * needle = lv_obj_get_child(gauge, 1);
    lv_obj_t * span_group = lv_obj_get_child(gauge, 2);
    lv_span_t * span_val = lv_spangroup_get_child(span_group, 0);
    lv_obj_t * min = lv_obj_get_child(gauge, 3);
    lv_obj_t * max = lv_obj_get_child(gauge, 4);

    // New target value
    int32_t new_value = scale_float(data->pid_value, data->precision);
    int32_t current_value = lv_bar_get_value(needle);

    if (current_value != new_value) {
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, needle);
        lv_anim_set_values(&a, current_value, new_value);
        lv_anim_set_duration(&a, ANIM_SPEED); // Animation duration
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_bar_set_value);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out); // <- smooth animation
        lv_anim_start(&a);
    }

    // Update needle color immediately
    lv_color_t needle_color = get_needle_color_from_value(data->pid_value, data->lower_limit, data->upper_limit);
    lv_obj_set_style_bg_color(needle, needle_color, LV_PART_INDICATOR);

    // Update text display
    char value_buf[16];
    snprintf(value_buf, sizeof(value_buf), float_only[data->precision], data->pid_value);
    lv_span_set_text(span_val, value_buf);
    lv_spangroup_refresh(span_group);

    if( min != NULL )
        lv_label_set_text_fmt(min, float_only[data->precision], data->pid_min);
    if( max != NULL )
        lv_label_set_text_fmt(max, float_only[data->precision], data->pid_max);
}

lv_obj_t * add_linear_gauge( int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, PID_DATA * pid)
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

    lv_obj_t * pid_label = lv_label_create(gauge);
    lv_obj_set_width(pid_label, LV_SIZE_CONTENT);
    lv_obj_set_height(pid_label, LV_SIZE_CONTENT);    /// 1
    lv_obj_align(pid_label, LV_ALIGN_TOP_MID, 0, BAR_LABEL_Y);
    lv_label_set_text(pid_label, pid->label);
    lv_obj_set_style_text_font(pid_label, BAR_FONT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_flag(pid_label, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t * needle = lv_bar_create(gauge);
    lv_bar_set_range(needle, scale_float(pid->lower_limit, pid->precision), scale_float(pid->upper_limit, pid->precision));
    lv_bar_set_mode(needle, LV_BAR_MODE_SYMMETRICAL);
    lv_obj_set_size(needle, w - BAR_PADDING, BAR_HEIGHT);
    lv_obj_set_align(needle, LV_ALIGN_TOP_MID);
    lv_obj_set_y(needle, BAR_Y_OFFSET);

    lv_obj_set_style_radius(needle, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(needle, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(needle, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(needle, lv_color_hex(0x00FF00), LV_PART_INDICATOR);

    lv_obj_set_style_outline_width(needle, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(needle, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(needle, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(needle, 0, LV_PART_INDICATOR);

    // Create span group for value and unit
    lv_obj_t * span_group = lv_spangroup_create(gauge);
    lv_obj_set_size(span_group, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_align(span_group, LV_ALIGN_CENTER);
    lv_obj_align(span_group, LV_ALIGN_TOP_MID, 0, BAR_LABEL_Y);

    // Create spans
    lv_span_t * span_val = lv_spangroup_new_span(span_group);
    lv_style_set_text_font(lv_span_get_style(span_val), &lv_font_montserrat_38);
    lv_style_set_text_color(lv_span_get_style(span_val), lv_color_white());

    lv_span_t * span_unit = lv_spangroup_new_span(span_group);
    lv_style_set_text_font(lv_span_get_style(span_unit), &lv_font_montserrat_24);
    lv_style_set_text_color(lv_span_get_style(span_unit), lv_color_hex(0xBBBBBB));

    lv_span_t * span_pid = lv_spangroup_new_span(span_group);
    lv_style_set_text_font(lv_span_get_style(span_pid), &lv_font_montserrat_24);
    lv_style_set_text_color(lv_span_get_style(span_pid), lv_color_white());

    // Split value and unit (assuming value is number and unit is already stored)
    char buf[24];
    snprintf(buf, sizeof(buf), float_only[pid->precision], pid->pid_value);
    lv_span_set_text(span_val, buf);
    lv_span_set_text(span_unit, pid->unit_label);
    snprintf(buf, sizeof(buf), " %s", pid->label);
    lv_span_set_text(span_pid, buf);
    lv_spangroup_refresh(span_group);

    if( w >= 500 )
    {
		lv_obj_t * min = lv_label_create(gauge);
		lv_obj_set_width(min, LV_SIZE_CONTENT);
		lv_obj_set_height(min, LV_SIZE_CONTENT);    /// 1
		lv_obj_align(min, LV_ALIGN_TOP_LEFT, BAR_PADDING/2, BAR_LABEL_Y);
		lv_label_set_text(min, "min");
		lv_obj_set_style_text_color(min, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
		lv_obj_set_style_text_opa(min, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
		lv_obj_set_style_text_font(min, BAR_FONT, LV_PART_MAIN | LV_STATE_DEFAULT);

		lv_obj_t * max = lv_label_create(gauge);
		lv_obj_set_width(max, LV_SIZE_CONTENT);
		lv_obj_set_height(max, LV_SIZE_CONTENT);    /// 1
		lv_obj_align(max, LV_ALIGN_TOP_RIGHT, -1*BAR_PADDING/2, BAR_LABEL_Y);
		lv_label_set_text(max, "max");
		lv_obj_set_style_text_color(max, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
		lv_obj_set_style_text_opa(max, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
		lv_obj_set_style_text_font(max, BAR_FONT, LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    return gauge;
}
