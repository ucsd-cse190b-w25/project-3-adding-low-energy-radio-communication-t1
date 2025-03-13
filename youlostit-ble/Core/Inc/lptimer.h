/*
 * lptimer.h
 *
 *  Created on: Mar 13, 2025
 *      Author: phillip
 */

#ifndef INC_LPTIMER_H_
#define INC_LPTIMER_H_

/* Include the type definitions for the timer peripheral */
#include <stm32l475xx.h>

void lptim_init(LPTIM_TypeDef* lptim);
void lptim_reset(LPTIM_TypeDef* lptim);
void lptim_set_sec(LPTIM_TypeDef* lptim, uint16_t period_sec);

#endif /* INC_LPTIMER_H_ */
