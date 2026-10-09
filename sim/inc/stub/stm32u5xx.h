/*
 * stm32u5xx.h (simulator stub)
 *
 * The simulator is single threaded, so interrupt masking is a no-op.
 */
#ifndef SIM_STM32U5XX_H
#define SIM_STM32U5XX_H

#include <stdint.h>

static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}

#endif /* SIM_STM32U5XX_H */
