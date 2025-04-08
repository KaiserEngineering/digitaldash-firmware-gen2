/*
 * eeprom_24cw.h
 *
 *  Created on: Apr 7, 2025
 *      Author: Matth
 */

#ifndef INC_EEPROM_24CW_H_
#define INC_EEPROM_24CW_H_

#include "stdint.h"
#include "stm32u5xx_hal.h"

uint8_t eeprom_24cw_read(I2C_HandleTypeDef *eeprom_handle, uint16_t bAdd);
void eeprom_24cw_write(I2C_HandleTypeDef *eeprom_handle, uint16_t bAdd, uint8_t bData);

#endif /* INC_EEPROM_24CW_H_ */
