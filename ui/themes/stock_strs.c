/*
 * stock_st.c
 *
 *  Created on: Feb 19, 2025
 *      Author: Matthew Kaiser
 */

#include "ui.h"
#include "lib_pid.h"

LV_IMG_DECLARE(ui_img_oem_st_gauge_png);
LV_IMG_DECLARE(ui_img_oem_rs_gauge_png);
LV_IMG_DECLARE(ui_img_oem_strs_needle_png);

#define STOCK_ST_START_ANGLE -600
#define STOCK_ST_END_ANGLE 600
#define CONTAINER_H_EXTEND 50
#define Y_ADJUST CONTAINER_H_EXTEND/2

typedef enum
{
    STOCK_ST,
	STOCK_RS,
} STOCK_GAUGE;

static lv_obj_t * add_stock_gauge( STOCK_GAUGE type, int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, PID_DATA * pid);

static void event_cb(lv_event_t * e)
{
    // Get the PID data
    PID_DATA * data = (PID_DATA *)lv_event_get_param(e);
    lv_obj_t * gauge = lv_event_get_target(e);
    lv_obj_t * span_group = lv_obj_get_child(gauge, 2);
    lv_span_t * span_val = lv_spangroup_get_child(span_group, 0);
    lv_obj_t * needle = lv_obj_get_child(gauge, 3);
    lv_obj_t * min = lv_obj_get_child(gauge, 4);
    lv_obj_t * max = lv_obj_get_child(gauge, 5);

    // Calculate target angle based on value
    int32_t angle;
    if( data->pid_value >= data->upper_limit )
        angle = STOCK_ST_END_ANGLE;
    else if ( data->pid_value <= data->lower_limit )
        angle = STOCK_ST_START_ANGLE;
    else {
        float normalized = (float)(data->pid_value - data->lower_limit) / (data->upper_limit - data->lower_limit);
        angle = STOCK_ST_START_ANGLE + (int32_t)(normalized * (STOCK_ST_END_ANGLE - STOCK_ST_START_ANGLE));
    }

    // Get current needle angle
    int32_t current_angle = lv_image_get_rotation(needle);

    // Animate if changed
    if(current_angle != angle) {
        // Handle wraparound to choose shortest path
        int32_t diff = angle - current_angle;
        if(diff > 1800)      angle -= 3600;
        else if(diff < -1800) angle += 3600;

        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, needle);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_image_set_rotation);
        lv_anim_set_values(&a, current_angle, angle);
        lv_anim_set_duration(&a, ANIM_SPEED+50);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
        lv_anim_start(&a);
    }

    // Update value text
    char value_buf[16];
    snprintf(value_buf, sizeof(value_buf), float_only[data->precision], data->pid_value);
    lv_span_set_text(span_val, value_buf);
    lv_spangroup_refresh(span_group);

    if( min != NULL )
        lv_label_set_text_fmt(min, float_only[data->precision], data->pid_min);
    if( max != NULL )
        lv_label_set_text_fmt(max, float_only[data->precision], data->pid_max);
}

lv_obj_t * add_stock_st_gauge( int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, PID_DATA * pid)
{
	return add_stock_gauge( STOCK_ST, x, y, w, h, parent, pid);
}

lv_obj_t * add_stock_rs_gauge( int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, PID_DATA * pid)
{
	return add_stock_gauge( STOCK_RS, x, y, w, h, parent, pid);
}


static lv_obj_t * add_stock_gauge( STOCK_GAUGE type, int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, PID_DATA * pid)
{
	lv_obj_t * gauge = lv_obj_create(parent);
    lv_obj_remove_style_all(gauge);
    lv_obj_set_width(gauge, w);
    lv_obj_set_height(gauge, h+CONTAINER_H_EXTEND);
    lv_obj_align(gauge, LV_ALIGN_TOP_MID, x, y);
    lv_obj_remove_flag(gauge, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_user_data(gauge, pid);
    lv_obj_add_event_cb(gauge, event_cb, LV_EVENT_REFRESH, pid);
	#if UI_CONTAINER_DEBUG
	lv_obj_set_style_border_width(gauge, 2, 0);                    // Thickness of the outline
	lv_obj_set_style_border_color(gauge, lv_color_white(), 0);    // White outline
	lv_obj_set_style_border_opa(gauge, LV_OPA_COVER, 0);          // Fully opaque
	#endif

	lv_obj_t * scale = lv_image_create(gauge);
	if( type == STOCK_RS )
		lv_image_set_src(scale, &ui_img_oem_rs_gauge_png);
	else
		lv_image_set_src(scale, &ui_img_oem_st_gauge_png);
    lv_obj_set_width(scale, LV_SIZE_CONTENT);   /// 125
    lv_obj_set_height(scale, LV_SIZE_CONTENT);    /// 125
    lv_obj_align(scale, LV_ALIGN_CENTER, 0, 70-Y_ADJUST);

    lv_obj_t * pid_label = lv_label_create(gauge);
    lv_obj_set_width(pid_label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(pid_label, LV_SIZE_CONTENT);    /// 1
    lv_obj_align(pid_label, LV_ALIGN_CENTER, 0, 35-Y_ADJUST);
    lv_label_set_text(pid_label, pid->label);
    lv_obj_set_style_text_font(pid_label, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    /*
    lv_obj_t * outter_arc = lv_arc_create(gauge);
    lv_obj_set_width(outter_arc, 250);
    lv_obj_set_height(outter_arc, 250);
    lv_obj_align(outter_arc, LV_ALIGN_TOP_MID, 0, 5);
    lv_arc_set_value(outter_arc, 90);
    lv_arc_set_bg_angles(outter_arc, 210, 330);
    lv_obj_set_style_arc_color(outter_arc, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(outter_arc, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(outter_arc, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_rounded(outter_arc, false, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_color(outter_arc, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(outter_arc, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(outter_arc, 5, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_rounded(outter_arc, false, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(outter_arc, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(outter_arc, 0, LV_PART_KNOB | LV_STATE_DEFAULT);

    lv_obj_t * inner_arc = lv_arc_create(gauge);
    lv_obj_set_width(inner_arc, 230);
    lv_obj_set_height(inner_arc, 230);
    lv_obj_align(inner_arc, LV_ALIGN_TOP_MID, 0, 14);
    lv_arc_set_range(inner_arc, 0, 8000);
    lv_arc_set_value(inner_arc, 6500);
    lv_arc_set_bg_angles(inner_arc, 210, 330);
    lv_obj_set_style_arc_color(inner_arc, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(inner_arc, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(inner_arc, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_rounded(inner_arc, false, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_color(inner_arc, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(inner_arc, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(inner_arc, 12, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_rounded(inner_arc, false, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(inner_arc, lv_color_hex(0x000000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(inner_arc, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    */

    // Create span group for value and unit
    lv_obj_t * span_group = lv_spangroup_create(gauge);
    lv_obj_set_size(span_group, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_align(span_group, LV_ALIGN_CENTER);
    lv_obj_set_y(span_group, 6-Y_ADJUST);
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
    snprintf(value_buf, sizeof(value_buf), float_only[pid->precision], pid->pid_value);
    lv_span_set_text(span_val, value_buf);
    lv_span_set_text(span_unit, pid->unit_label);
    lv_spangroup_refresh(span_group);

    lv_obj_t * needle;
    needle = lv_image_create(gauge);
    lv_image_set_src(needle, &ui_img_oem_strs_needle_png);
    lv_obj_set_width(needle, LV_SIZE_CONTENT);   /// 125
    lv_obj_set_height(needle, LV_SIZE_CONTENT);    /// 125
    lv_obj_align(needle, LV_ALIGN_CENTER, 0, 45);
    lv_image_set_rotation(needle, STOCK_ST_START_ANGLE);

    lv_obj_t * min = lv_label_create(gauge);
    lv_obj_set_width(min, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(min, LV_SIZE_CONTENT);    /// 1
    lv_obj_align(min, LV_ALIGN_CENTER, -100, 40-Y_ADJUST);
    lv_label_set_text(min, "min");
	lv_obj_set_style_text_color(min, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(min, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(min, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t * max = lv_label_create(gauge);
    lv_obj_set_width(max, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(max, LV_SIZE_CONTENT);    /// 1
    lv_obj_align(max, LV_ALIGN_CENTER, 100, 40-Y_ADJUST);
    lv_label_set_text(max, "max");
	lv_obj_set_style_text_color(max, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(max, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(max, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    return gauge;
}
