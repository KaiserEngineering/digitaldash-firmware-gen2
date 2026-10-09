/*
 * lib_digital_dash.h (simulator stub)
 *
 * Shadows the firmware header so the UI links against the simulator's PID
 * stream (sim/src/sim_stream.c) instead of the data acquisition stack.
 */
#ifndef LIB_DIGITAL_DASH_H
#define LIB_DIGITAL_DASH_H

#include <stddef.h>
#include "ke_conf.h"
#include "lvgl.h"
#include "ui.h"
#include "ke_config.h"
#include "lib_pid.h"

uint32_t DigitalDash_Get_PID_Refresh_Period_ms( void );
uint8_t DigitalDash_Get_PID_Refresh_Count( void );
int DigitalDash_Remove_PID_From_Stream( PTR_PID_DATA pid, uint8_t device );
PTR_PID_DATA DigitalDash_Add_PID_To_Stream( PTR_PID_DATA pid, uint8_t device );
uint8_t DigitalDash_Pause_PID_In_Stream( PTR_PID_DATA pid, uint8_t device );
uint8_t DigitalDash_Resume_PID_In_Stream( PTR_PID_DATA pid, uint8_t device );

/* Simulator only: set every streamed copy of a PID, value in its base unit */
void sim_stream_set_value( uint32_t pid_uuid, float value, uint32_t timestamp );

#endif /* LIB_DIGITAL_DASH_H */
