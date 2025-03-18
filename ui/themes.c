/*
 * themes.c
 *
 *  Created on: Mar 14, 2025
 *      Author: Matth
 */


#include "themes.h"

lv_obj_t * add_gauge( GAUGE_THEME theme, int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid)
{
	switch(theme)
	{
		case GRUMPY_CAT:
			return add_grumpy_cat_gauge( x, y, parent, pid);
		case STOCK_ST:
		default:
			return add_stock_st_gauge( x, y, parent, pid);
	}
}
