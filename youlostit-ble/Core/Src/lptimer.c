/*
 * lptimer.c
 *
 *  Created on: Mar 13, 2025
 *      Author: phillip
 */

#include "lptimer.h"

void lptim_init(LPTIM_TypeDef* lptim) {
	// Set up the LSI clock
	RCC->CIER |= RCC_CIER_LSIRDYIE;
	RCC->CSR |= RCC_CSR_LSION;
	while((RCC->CSR & RCC_CSR_LSIRDY) == 0) {}

	// Select the LSI clock for the LPTIM1
	RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;
	RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL_0;

	// Set up the Low Power Timer
	NVIC_EnableIRQ(LPTIM1_IRQn);
	NVIC_SetPriority(LPTIM1_IRQn,1);

	RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
	RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_LPTIM1RST;
	RCC->APB1SMENR1 |= RCC_APB1SMENR1_LPTIM1SMEN;

	lptim->CR &= ~LPTIM_CR_ENABLE;
	for (volatile int i = 0;i < 10; i++);

	// enable interrupts from auto reload match
	lptim->IER |= LPTIM_IER_ARRMIE;
	lptim->CFGR &= ~LPTIM_CFGR_CKSEL;
	lptim->CFGR &= ~LPTIM_CFGR_COUNTMODE;

	// Set the Prescaler
	lptim->CFGR &= ~LPTIM_CFGR_PRESC;
	lptim->CFGR |= LPTIM_CFGR_PRESC_0 | LPTIM_CFGR_PRESC_1 | LPTIM_CFGR_PRESC_2;

	lptim->CFGR &= ~LPTIM_CFGR_TRIGEN;

	// enable after doing ier and cfgr and need to wait 2 clock cycles
	lptim->CR |= LPTIM_CR_ENABLE;
	for (volatile int i = 0; i < 30; i++);
	lptim->CR |= LPTIM_CR_CNTSTRT;
	lptim->CR &= ~LPTIM_CR_SNGSTRT;
}

void lptim_reset(LPTIM_TypeDef* lptim) {
//	lptim->CNT &= ~LPTIM_CNT_CNT;
}

void lptim_set_sec(LPTIM_TypeDef* lptim, uint16_t period_sec) {
//	lptim->CNT &= ~LPTIM_CNT_CNT;
//	lptim->CMP = 30000;
	lptim->ARR = (period_sec * 256) - 1;
}
