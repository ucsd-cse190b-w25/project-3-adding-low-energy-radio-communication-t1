/*
 * timer.h
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#ifndef TIMER_H_
#define TIMER_H_

/* Include the type definitions for the timer peripheral */
#include <stm32l475xx.h>

void timer_init(TIM_TypeDef* timer);
void timer_reset(TIM_TypeDef* timer);
void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms);
void timer_set_presc(TIM_TypeDef* timer, uint16_t presc);
void timer_set_arr(TIM_TypeDef* timer, uint16_t arr);

#endif /* TIMER_H_ */
