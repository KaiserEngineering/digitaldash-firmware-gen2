/*
 * sim_stream.c (simulator)
 *
 * Stand-in for the PID stream in lib_digital_dash.c. Slots are shared by
 * PID UUID exactly like the firmware, but values come from JavaScript via
 * sim_stream_set_value() instead of OBDII / CAN / vehicle data.
 */
#include <string.h>
#include "lib_digital_dash.h"

#define BITSET(word,nbit)   ((word) |=  (1<<(nbit)))
#define BITCLEAR(word,nbit) ((word) &= ~(1<<(nbit)))

static PID_DATA stream[DD_MAX_PIDS];

uint32_t DigitalDash_Get_PID_Refresh_Period_ms( void ) { return 0U; }
uint8_t DigitalDash_Get_PID_Refresh_Count( void ) { return 0U; }

PTR_PID_DATA DigitalDash_Add_PID_To_Stream( PTR_PID_DATA pid, uint8_t device )
{
	for( uint8_t i = 0; i < DD_MAX_PIDS; i++ )
	{
		if( (stream[i].pid_uuid != PID_UNASSIGNED) && (stream[i].pid_uuid == pid->pid_uuid) )
		{
			BITSET(stream[i].devices, device);
			BITSET(stream[i].num_activated, device);
			return &stream[i];
		}
	}

	uint8_t slot;
	for( slot = 0; slot < DD_MAX_PIDS; slot++ )
	{
		if( stream[slot].pid_uuid == PID_UNASSIGNED )
			break;
	}
	if( slot >= DD_MAX_PIDS )
		return NULL;

	pid->acquisition_type = PID_UNASSIGNED;
	pid->pid_value        = 0;
	pid->timestamp        = 0;
	pid->pid_min          = INIT_MIN;
	pid->pid_max          = INIT_MAX;
	pid->devices          = 0;
	pid->num_activated    = 0;

	stream[slot] = *pid;
	BITSET(stream[slot].devices, device);
	BITSET(stream[slot].num_activated, device);
	load_pid_data( &stream[slot] );

	return &stream[slot];
}

int DigitalDash_Remove_PID_From_Stream( PTR_PID_DATA pid, uint8_t device )
{
	for( uint8_t i = 0; i < DD_MAX_PIDS; i++ )
	{
		if( (stream[i].pid_uuid != PID_UNASSIGNED) && (&stream[i] == pid) )
		{
			BITCLEAR(stream[i].devices, device);
			BITCLEAR(stream[i].num_activated, device);
			if( stream[i].devices == 0 )
				lib_pid_clear_PID( &stream[i] );
			return 1;
		}
	}
	return 0;
}

uint8_t DigitalDash_Pause_PID_In_Stream( PTR_PID_DATA pid, uint8_t device )
{
	(void)pid; (void)device;
	return 1U;
}

uint8_t DigitalDash_Resume_PID_In_Stream( PTR_PID_DATA pid, uint8_t device )
{
	(void)pid; (void)device;
	return 1U;
}

void sim_stream_set_value( uint32_t pid_uuid, float value, uint32_t timestamp )
{
	for( uint8_t i = 0; i < DD_MAX_PIDS; i++ )
	{
		if( (stream[i].pid_uuid != PID_UNASSIGNED) && (stream[i].pid_uuid == pid_uuid) )
			update_pid_data( &stream[i], value, timestamp );
	}
}
