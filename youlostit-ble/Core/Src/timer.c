/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


void timer_init(TIM_TypeDef* timer)
{
	// Give power to the timer
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

	// Enable interrupts
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 1);

	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 1);

	// Reset the timer
	timer->CNT &= 0;

	// Stop the timer
	timer->CR1 &= (~TIM_CR1_CEN);

	// Reset the timer
	timer->CNT &= 0;

	// Set the prescalar to 3999 since 1/(3999+1) = 4000 for 1 ms
//	timer->PSC = 3999; // adjust for 4mhz clock
	timer->PSC = 7999; // adjust it for an 8mhz clock
//	timer->PSC = 99; // adjust for 100 khz clock
//	timer->PSC = 199; // adjust for 200khz clock
//	timer->PSC = 399; // adjust for 400khz clock
//	timer->PSC = 799; // adjust for 800khz clock (minimum working)
//	timer->PSC = 999; // adjust for 1mhz clock
//	timer->PSC = 1999; // adjust for 2mhz clock
//	timer->PSC = 19999; //

	// reset the status of the timer
	timer->SR &= ~TIM_SR_UIF;


	// Enable interrupts on timer
	timer->DIER |= TIM_DIER_UIE;

	// Turn on the timer again
	timer->CR1 |= TIM_CR1_CEN;

}

void timer_reset(TIM_TypeDef* timer)
{
	// Set the counter in the timer back to 0
	timer->CNT &= 0;
}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
	timer->CNT &= 0;
  	// Set the reload time of the timer
	timer->ARR = period_ms - 1;
}

void timer_set_presc(TIM_TypeDef* timer, uint16_t presc) {
	timer->PSC = presc;
}

void timer_set_arr(TIM_TypeDef* timer, uint16_t arr) {
	timer->CNT &= 0;
	timer->ARR = arr;
}

