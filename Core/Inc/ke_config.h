/**
 ******************************************************************************
 *
 * Copyright (c) 2025 KaiserEngineering, LLC
 * Author Matthew Kaiser
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************
 */

#ifndef KE_CONFIG_H
#define KE_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "lvgl.h"
#include "stdbool.h"
#include "string.h"
#include "lib_pid.h"

typedef void(settings_write)(uint16_t bAdd, uint8_t bData);
typedef uint8_t(settings_read)(uint16_t bAdd);

void settings_setWriteHandler(settings_write *writeHandler);
void settings_setReadHandler(settings_read *readHandler);

#define GAUGES_PER_VIEW 3
#define MAX_ALERTS 5
#define ALERT_MESSAGE_LEN 64
#define MAX_VIEWS 3
#define NUM_DYNAMIC 2

/********************************************************************************
*                                  View enable                                  
*
* @param idx_view    index of the view
* @param enable    Enable or disable view by index
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
typedef enum
{
    VIEW_STATE_DISABLED,
    VIEW_STATE_ENABLED,
    VIEW_STATE_RESERVED
} VIEW_STATE;

bool verify_view_enable(VIEW_STATE enable);
VIEW_STATE get_view_enable(uint8_t idx_view);
bool set_view_enable(uint8_t idx_view, VIEW_STATE enable, bool save);


/********************************************************************************
*                                Number of gauges                               
*
* @param idx_view    index of the view
* @param num_gauges    Define the number of gauges for view by index
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
bool verify_view_num_gauges(uint8_t num_gauges);
uint8_t get_view_num_gauges(uint8_t idx_view);
bool set_view_num_gauges(uint8_t idx_view, uint8_t num_gauges, bool save);


/********************************************************************************
*                                   Background                                  
*
* @param idx_view    index of the view
* @param background    Set the background color or image of a view by index
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
typedef enum
{
    VIEW_BACKGROUND_BLACK,
    VIEW_BACKGROUND_FLARE,
    VIEW_BACKGROUND_RESERVED
} VIEW_BACKGROUND;

bool verify_view_background(VIEW_BACKGROUND background);
VIEW_BACKGROUND get_view_background(uint8_t idx_view);
bool set_view_background(uint8_t idx_view, VIEW_BACKGROUND background, bool save);


/********************************************************************************
*                          Theme assigned to the gauge                          
*
* @param idx_view    index of the view
* @param idx_gauge    index of the gauge
* @param theme    Set the gauge theme by view and gauge index
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
typedef enum
{
    GAUGE_THEME_STOCK_ST,
    GAUGE_THEME_GRUMPY_CAT,
    GAUGE_THEME_RESERVED
} GAUGE_THEME;

bool verify_view_gauge_theme(GAUGE_THEME theme);
GAUGE_THEME get_view_gauge_theme(uint8_t idx_view, uint8_t idx_gauge);
bool set_view_gauge_theme(uint8_t idx_view, uint8_t idx_gauge,  GAUGE_THEME theme, bool save);


/********************************************************************************
*                           PID assigned to the gauge                           
*
* @param idx_view    index of the view
* @param idx_gauge    index of the gauge
* @param pid    Set the gauge PID by view and gauge index
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
bool verify_view_gauge_pid(uint32_t pid);
uint32_t get_view_gauge_pid(uint8_t idx_view, uint8_t idx_gauge);
bool set_view_gauge_pid(uint8_t idx_view, uint8_t idx_gauge,  uint32_t pid, bool save);


/********************************************************************************
*                        PID units assigned to the gauge                        
*
* @param idx_view    index of the view
* @param idx_gauge    index of the gauge
* @param units    Set the PID units by view and gauge index
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
bool verify_view_gauge_units(PID_UNITS units);
PID_UNITS get_view_gauge_units(uint8_t idx_view, uint8_t idx_gauge);
bool set_view_gauge_units(uint8_t idx_view, uint8_t idx_gauge,  PID_UNITS units, bool save);


/********************************************************************************
*                                  Alert enable                                 
*
* @param idx_alert    index of the alert
* @param enable    Enable or disable view by index
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
typedef enum
{
    ALERT_STATE_DISABLED,
    ALERT_STATE_ENABLED,
    ALERT_STATE_RESERVED
} ALERT_STATE;

bool verify_alert_enable(ALERT_STATE enable);
ALERT_STATE get_alert_enable(uint8_t idx_alert);
bool set_alert_enable(uint8_t idx_alert, ALERT_STATE enable, bool save);


/********************************************************************************
*                                 Alert message                                 
*
* @param idx_alert    index of the alert
* @param message    Set the message that appears when alert occurs
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
bool verify_alert_message(char* message);
void get_alert_message(uint8_t idx, char* alert_message);
bool set_alert_message(uint8_t idx_alert, char* message, bool save);


/********************************************************************************
*                                Comparison type                                
*
* @param idx_alert    index of the alert
* @param compare    Configure the comparison used for the realtime value and alert threshold
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
typedef enum
{
    ALERT_COMPARISON_LESS_THAN,
    ALERT_COMPARISON_LESS_THAN_OR_EQUAL_TO,
    ALERT_COMPARISON_GREATER_THAN,
    ALERT_COMPARISON_GREATER_THAN_OR_EQUAL_TO,
    ALERT_COMPARISON_EQUAL,
    ALERT_COMPARISON_NOT_EQUAL,
    ALERT_COMPARISON_RESERVED
} ALERT_COMPARISON;

bool verify_alert_compare(ALERT_COMPARISON compare);
ALERT_COMPARISON get_alert_compare(uint8_t idx_alert);
bool set_alert_compare(uint8_t idx_alert, ALERT_COMPARISON compare, bool save);


/********************************************************************************
*                            Dynamic gauge threshold                            
*
* @param idx_alert    index of the alert
* @param threshold    Comparison value of the dynamic gauge
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
bool verify_alert_threshold(float threshold);
float get_alert_threshold(uint8_t idx_alert);
bool set_alert_threshold(uint8_t idx_alert, float threshold, bool save);


/********************************************************************************
*                                 Dynamic enable                                
*
* @param idx_dynamic    index of the dynamic
* @param enable    Enable or disable view by index
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
typedef enum
{
    DYNAMIC_STATE_DISABLED,
    DYNAMIC_STATE_ENABLED,
    DYNAMIC_STATE_RESERVED
} DYNAMIC_STATE;

bool verify_dynamic_enable(DYNAMIC_STATE enable);
DYNAMIC_STATE get_dynamic_enable(uint8_t idx_dynamic);
bool set_dynamic_enable(uint8_t idx_dynamic, DYNAMIC_STATE enable, bool save);


/********************************************************************************
*                                    Priority                                   
*
* @param idx_dynamic    index of the dynamic
* @param priority    Priority of the dynamic gauges, if both gauges comparisons are met then highest priority wins
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
typedef enum
{
    DYNAMIC_PRIORITY_LOW,
    DYNAMIC_PRIORITY_MEDIUM,
    DYNAMIC_PRIORITY_HIGH,
    DYNAMIC_PRIORITY_RESERVED
} DYNAMIC_PRIORITY;

bool verify_dynamic_priority(DYNAMIC_PRIORITY priority);
DYNAMIC_PRIORITY get_dynamic_priority(uint8_t idx_dynamic);
bool set_dynamic_priority(uint8_t idx_dynamic, DYNAMIC_PRIORITY priority, bool save);


/********************************************************************************
*                                Comparison type                                
*
* @param idx_dynamic    index of the dynamic
* @param compare    Configure the comparison used for the realtime value and alert threshold
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
typedef enum
{
    DYNAMIC_COMPARISON_LESS_THAN,
    DYNAMIC_COMPARISON_LESS_THAN_OR_EQUAL_TO,
    DYNAMIC_COMPARISON_GREATER_THAN,
    DYNAMIC_COMPARISON_GREATER_THAN_OR_EQUAL_TO,
    DYNAMIC_COMPARISON_EQUAL,
    DYNAMIC_COMPARISON_NOT_EQUAL,
    DYNAMIC_COMPARISON_RESERVED
} DYNAMIC_COMPARISON;

bool verify_dynamic_compare(DYNAMIC_COMPARISON compare);
DYNAMIC_COMPARISON get_dynamic_compare(uint8_t idx_dynamic);
bool set_dynamic_compare(uint8_t idx_dynamic, DYNAMIC_COMPARISON compare, bool save);


/********************************************************************************
*                            Dynamic gauge threshold                            
*
* @param idx_dynamic    index of the dynamic
* @param Threshold    Comparison value of the dynamic gauge
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
bool verify_dynamic_threshold(float threshold);
float get_dynamic_threshold(uint8_t idx_dynamic);
bool set_dynamic_threshold(uint8_t idx_dynamic, float threshold, bool save);


/********************************************************************************
*                                   View index                                  
*
* @param idx_dynamic    index of the dynamic
* @param Index    Set which view should be enabled if the dynamic event is true
* @param save    Set true to save to the EEPROM, otherwise value is non-volatile
*
********************************************************************************/
bool verify_dynamic_index(uint8_t index);
uint8_t get_dynamic_index(uint8_t idx_dynamic);
bool set_dynamic_index(uint8_t idx_dynamic, uint8_t index, bool save);

#ifdef __cplusplus
}
#endif

#endif /* KE_CONFIG_H */