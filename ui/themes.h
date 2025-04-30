/*
 * themes.h
 *
 *  Created on: Feb 19, 2025
 *      Author: Matth
 */

#ifndef THEMES_H_
#define THEMES_H_

#include "ui.h"

lv_obj_t * add_grumpy_cat_gauge( int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid);
lv_obj_t * add_linear_gauge( int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid);
lv_obj_t * add_radial_gauge( int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid);
lv_obj_t * add_stock_st_gauge( int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid);

lv_obj_t * add_gauge( GAUGE_THEME theme, int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid);

#endif /* THEMES_H_ */
