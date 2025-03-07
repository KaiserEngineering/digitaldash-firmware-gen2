/*
 * ke_digitaldash.h
 *
 *  Created on: Feb 21, 2025
 *      Author: Matth
 */

#ifndef INC_KE_DIGITALDASH_H_
#define INC_KE_DIGITALDASH_H_

#include "lib_pid.h"
#include "lvgl.h"
#include "stdio.h"

#define MAX_ALERT_LENGTH 64
#define MAX_GAUGES 3
#define MAX_ALERTS 5
#define MAX_VIEWS 3

typedef enum {
    DD_LESS_THAN,
    DD_LESS_THAN_OR_EQUAL_TO,
    DD_GREATER_THAN,
    DD_GREATER_THAN_OR_EQUAL_TO,
    DD_EQUAL,
    DD_NOT_EQUAL
}digitaldash_compare;

typedef enum {
    DD_LOW_PRIORITY,
    DD_MEDIUM_PRIORITY,
    DD_HIGH_PRIORITY
}digitaldash_priority;

typedef enum {
    THEME_STOCK_ST,
    THEME_STOCK_RS
}digitaldash_theme;

typedef enum {
    BACKGROUND_FLARE,
	BACKGROUND_USER1,
	BACKGROUND_RED,
	BACKGROUND_GREEN,
	BACKGROUND_BLUE,
    BACKGROUND_BLACK
}digitaldash_background;

typedef struct {
	PID_DATA * pid;              // PID to monitor
    digitaldash_compare compare; // Comparison type
    float thresh;                // Value to compare against
}digitaldash_trigger;

typedef struct {
    digitaldash_trigger trigger; // Trigger for when an alert should appear
    char msg[MAX_ALERT_LENGTH];  // Alert message
}digitaldash_alert;

typedef struct {
    digitaldash_theme theme;   // Style of the gauge
    PID_DATA * pid; // Parameter that is being streamed
    lv_obj_t * obj;
}digitaldash_gauge;

typedef struct {
    uint8_t enabled; // Is the view enabled or not
    uint8_t view_index; // Screen number 0 - MAX_VIEWS
    uint8_t num_gauges;
    digitaldash_background background; // background of the specific view
    digitaldash_gauge gauge[MAX_GAUGES]; // Gauge objects
}digitaldash_view;

typedef struct {
    uint8_t enabled;
    uint8_t view_index;
    digitaldash_trigger trigger;   // Trigger for when a view should dynamically change
    digitaldash_priority priority; // Priority level for when multiple trigger events are true
}digitaldash_dynamic;

typedef struct {
    digitaldash_view view[MAX_VIEWS];
    uint8_t num_views;
    digitaldash_alert alert[MAX_ALERTS];
    digitaldash_dynamic dynamic[MAX_VIEWS];
}digitaldash;

#endif /* INC_KE_DIGITALDASH_H_ */
