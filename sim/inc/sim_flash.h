/*
 * sim_flash.h (simulator)
 *
 * Force-included into every translation unit. On the device the user
 * backgrounds are memory mapped from OSPI flash; here they live in a static
 * buffer so ui.c's background descriptors point at real wasm memory.
 */
#ifndef SIM_FLASH_H
#define SIM_FLASH_H

#include <stdint.h>

/* The firmware build gets the UI resolution macros onto every UI file. */
#include "ke_conf.h"

extern uint8_t sim_ospi_flash[];

/* ui.h places the first background BACKGROUND_OFFSET past the flash base */
#define OSPI_BASE_ADDRESS ((uint32_t)(uintptr_t)sim_ospi_flash - 0x00400000u)

#endif /* SIM_FLASH_H */
