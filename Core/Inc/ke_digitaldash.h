/*
 * ke_digitaldash.h
 *
 *  Created on: Feb 21, 2025
 *      Author: Matth
 */

 #ifndef INC_KE_DIGITALDASH_H_
 #define INC_KE_DIGITALDASH_H_
 
 #include "lib_pid.h"
 #include <stdio.h>
 #include "ke_config.h"
 #include "lvgl.h"
 
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
 
 typedef struct {
     PID_DATA * pid;              // PID to monitor
     digitaldash_compare compare; // Comparison type
     float thresh;                // Value to compare against
 }digitaldash_trigger;
 
 typedef struct {
     uint8_t enabled;
     PID_DATA * pid;              // PID to monitor
     digitaldash_compare compare; // Comparison type
     float thresh;                // Value to compare against
     char msg[ALERT_MESSAGE_LEN];  // Alert message
 }digitaldash_alert;
 
 typedef struct {
     GAUGE_THEME theme;   // Style of the gauge
     PID_DATA * pid; // Parameter that is being streamed
     float min;
     float max;
     float value;
     float position;
     lv_obj_t * obj;
 }digitaldash_gauge;
 
 typedef struct {
     uint8_t enabled; // Is the view enabled or not
     uint8_t num_gauges;
     VIEW_BACKGROUND background; // background of the specific view
     digitaldash_gauge gauge[MAX_GAUGES_PER_VIEW]; // Gauge objects
 }digitaldash_view;
 
 typedef struct {
     uint8_t enabled;
     uint8_t view_index;
     PID_DATA * pid;              // PID to monitor
     digitaldash_compare compare; // Comparison type
     float thresh;                // Value to compare against
     digitaldash_priority priority; // Priority level for when multiple trigger events are true
 }digitaldash_dynamic;
 
 typedef struct {
     digitaldash_view view[MAX_VIEWS];
     uint8_t num_views;
     digitaldash_alert alert[MAX_ALERTS];
     digitaldash_dynamic dynamic[MAX_DYNAMICS];
 }digitaldash;
 
 bool digitaldash_to_json(const digitaldash *dash, char *buffer, size_t buffer_size);
 bool json_to_digitaldash(const char* json_str, digitaldash* dash_out);

 #endif /* INC_KE_DIGITALDASH_H_ */
 
