/*
 * ui.h
 *
 *  Created on: Mar 30, 2025
 *      Author: Matth
 */

#ifndef UI_H_
#define UI_H_

#include "lvgl.h"
#include "lib_pid.h"
#include "lvgl_port_display.h"
#include "ke_config.h"

extern const char *float_with_units[3];
extern const char *float_only[3];
extern const char *two_float_with_slash[3];

lv_obj_t * add_grumpy_cat_gauge( int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid);
lv_obj_t * add_linear_gauge( int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid);
lv_obj_t * add_radial_gauge( int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid);
lv_obj_t * add_stock_st_gauge( int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid);

lv_obj_t * add_gauge( GAUGE_THEME theme, int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid);
int32_t scale_float( float val, uint8_t precision );
void label_set_text_fmt_with_check(lv_obj_t * obj, const char * fmt, ...);

lv_obj_t * add_alert( lv_obj_t * parent );
bool get_alert(void);
void set_alert(char *msg );
void clear_alert(void);

#endif /* UI_H_ */
