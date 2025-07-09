/*
 * alert.c
 *
 *  Created on: May 12, 2025
 *      Author: Matth
 */

#include "ui.h"

lv_obj_t * ui_alert_container = NULL;
lv_obj_t * ui_alert = NULL;

lv_obj_t * add_alert( lv_obj_t * parent ){
	  ui_alert_container = lv_obj_create(parent);
	  lv_obj_remove_style_all(ui_alert_container);
	  lv_obj_set_width(ui_alert_container, 670);
	  lv_obj_set_height(ui_alert_container, 75);
	  lv_obj_set_x(ui_alert_container, X_OFFSET);
	  lv_obj_set_y(ui_alert_container, 5);
	  lv_obj_set_align(ui_alert_container, LV_ALIGN_TOP_MID);
	  //lv_obj_add_flag(ui_alert_container, LV_OBJ_FLAG_HIDDEN);     /// Flags
	  lv_obj_remove_flag(ui_alert_container,
	                     LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
	                     LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
	                     LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
	  lv_obj_set_style_radius(ui_alert_container, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	  lv_obj_set_style_bg_color(ui_alert_container, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
	  lv_obj_set_style_bg_opa(ui_alert_container, 225, LV_PART_MAIN | LV_STATE_DEFAULT);
	  lv_obj_set_style_border_color(ui_alert_container, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
	  lv_obj_set_style_border_opa(ui_alert_container, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	  lv_obj_set_style_border_width(ui_alert_container, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
	  lv_obj_set_style_shadow_color(ui_alert_container, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
	  lv_obj_set_style_shadow_opa(ui_alert_container, 225, LV_PART_MAIN | LV_STATE_DEFAULT);
	  lv_obj_set_style_shadow_width(ui_alert_container, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
	  lv_obj_set_style_shadow_spread(ui_alert_container, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	  lv_obj_set_style_shadow_offset_x(ui_alert_container, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
	  lv_obj_set_style_shadow_offset_y(ui_alert_container, 12, LV_PART_MAIN | LV_STATE_DEFAULT);

	  ui_alert = lv_label_create(ui_alert_container);
	  lv_obj_set_width(ui_alert, LV_SIZE_CONTENT);   /// 1
	  lv_obj_set_height(ui_alert, LV_SIZE_CONTENT);    /// 1
	  lv_obj_set_align(ui_alert, LV_ALIGN_CENTER);
	  lv_label_set_text(ui_alert, "No error set");
	  lv_obj_set_style_text_font(ui_alert, &lv_font_montserrat_30, LV_PART_MAIN | LV_STATE_DEFAULT);
	  lv_obj_add_flag(ui_alert_container, LV_OBJ_FLAG_HIDDEN);

	  // Hide the alert by default
	  //lv_obj_add_flag(ui_alert_container, LV_OBJ_FLAG_HIDDEN);
	  return ui_alert_container;
}

bool get_alert(void)
{
	return lv_obj_has_flag(ui_alert_container, LV_OBJ_FLAG_HIDDEN);
}

void set_alert( char *msg )
{
	lv_label_set_text(ui_alert, msg);
	lv_obj_remove_flag(ui_alert_container, LV_OBJ_FLAG_HIDDEN);
}

void clear_alert(void)
{
	lv_label_set_text(ui_alert, "No error set");
	lv_obj_add_flag(ui_alert_container, LV_OBJ_FLAG_HIDDEN);
}
