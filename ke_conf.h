/**
 * @file ke_conf.h
 */

/*
 * Copy this file as `ke_conf.h`
 */

#ifndef KE_CONF_H
#define KE_CONF_H

#define FIRMWARE_VERSION_MAJOR  1
#define FIRMWARE_VERSION_MINOR  0
#define FIRMWARE_VERSION_HOTFIX 8

#define SPOOF_DATA 0
#define ENABLE_WHEN_ENGINE_ON 0
#define SAFE_SHUTDOWN 0 // This is only needed for the Pi
#define LCD_ALWAYS_ON 1

#define USE_KE_PROTOCOL 0
#define USE_UNIT_CONVERSION 1
#define USE_LIB_OBDII 1
#define USE_LIB_CAN_BUS_SNIFFER 1
#define USE_LIB_VEHICLE_DATA 1
#define USE_LIB_PID 1

#define VEHICLE_GENERIC 1
#define VEHICLE_FORD_FOCUS_STRS_2013_2018 2

#define VEHICLE FORD_FOCUS_STRS_2013_2018


/********************************************************************
* Hardware configuration                                            *
********************************************************************/
#define SD_CARD_ACTIVE 0
#define KE_ACTIVE 0
#define ECU_ACTIVE 1
#define BKLT_CTRL_ACTIVE 1
#define FAN_CTRL_ACTIVE 0
#define HOST_CTRL_ACTIVE 1
#define USB_PWR_CTRL 0
#define HW_CAN_FILTERS 1

#define POWER_CYCLE_TIME 1000
#define FORD_MAX_BRIGHTNESS 31
#define FORD_MIN_BRIGHTNESS 1
#define LCD_MAX_BRIGHTNESS 255
#define LCD_MIN_BRIGHTNESS 20
#define LCD_BKLT_TIMEOUT 1000

/* How long to wait in ms until resuming CAN communication */
#define TESTER_PRESENT_DELAY 10000

/* How long to wait in ms to shutdown if the engine is OFF */
#define ENGINE_OFF_SHUTDOWN_TIME 100000

#define FORCE_USB_ON     0

#define DD_MAX_PIDS 25

/********************************************************************
* CAN Bus Configuration                                             *
********************************************************************/
typedef enum _ecu_comm {
    ECU_COMM_NOT_AVAILABLE,
    ECU_COMM_AVAILABLE
} ECU_COMM, *PECU_COMM;


#define ECU_COMM_AVAL        ECU_COMM_AVAILABLE
#define ECU_TX_ID            (uint16_t)0x7E0
#define ECU_RX_ID            (uint16_t)0x7E8
#define ECU_DLC              (uint8_t)0x08
#define ECU_TIMEOUT          (uint32_t)1000
#define MAX_CAN_FILTERS       28

/********************************************************************
* UART Configuration                                                *
********************************************************************/
#define KE_MAX_PKT_SIZE      (uint8_t)0x28

/********************************************************************
* OS Configuration                                                  *
********************************************************************/
#define OS_BOOT_TIME_MAX 1000 // Reboot after 10 seconds
#define OS_FRAME_TIMEOUT 60000

/********************************************************************
* UI Configuration                                                  *
********************************************************************/
#define UI_ENABLED 1
#if UI_ENABLED
	#define SCREEN_FADE_INIT_T 1000 // Fading for when the screen first turns on
	#define SCREEN_FADE_T 250  // Fading for when the screen switches screens

	#define UI_HOR_RES    1024 // UI horizontal Resolution (This can be different from screen resolution)
	#define UI_VER_RES    200  // UI vertical Resolution (This can be different from screen resolution)
	#define UI_BYTES_PER_PIXEL 4 // Number of bytes per pixel (RGBA = 4)
	#define ACT_HOR_RES    1024 // Screen horizontal resolution
	#define ACT_VER_RES    600  // Screen vertical resolution
	#define DISP_PARTIAL 0 // Enable/disable partial rendering
#endif

#endif /*End of "Content enable"*/
