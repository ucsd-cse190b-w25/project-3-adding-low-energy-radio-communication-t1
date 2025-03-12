/*
 * i2c.c
 *
 *  Created on: Jan 26, 2025
 *      Author: skyler
 */

#include "i2c.h"
/* Links
 * I2C Register Map
 * https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf#page=1337&zoom=100,165,106
 * RCC APB1:
 * https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf#page=81
 * https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf#page=253
 * I2C Master Mode:
 * https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf#page=1292
 * I2C Timing details
 * https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf#page=1292&zoom=100,165,150
 * I2C Registers
 * https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf#page=1323
 * I2C Configure Guide
 * https://controllerstech.com/stm32-i2c-configuration-using-registers/
 * I2C Initialization
 * https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf#page=1275
 * I2C Transfer/Receiver
 * https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf#page=1337&zoom=100,165,106
 * I2C Slave
 * https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf#page=1332
 * I2C Master
 * https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf#page=1327
 * Slave transmission and receiver flowchart
 * https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf#page=1287
 * Master Initialization
 * https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf#page=1294&zoom=100,165,150
 *
*/

unsigned int in = 0;

void i2c_init() {
	// Turn on Clock for I2C
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// Configure GPIO
	// Set GPIO to Alternative function mode
	GPIOB->MODER &= ~GPIO_MODER_MODE10;
	GPIOB->MODER |= GPIO_MODER_MODE10_1;
	GPIOB->MODER &= ~GPIO_MODER_MODE11;
	GPIOB->MODER |= GPIO_MODER_MODE11_1;

	/* Configure the GPIO output as push pull (transistor for high and low) */
	GPIOB->OTYPER |= GPIO_OTYPER_OT11;
	GPIOB->OTYPER |= GPIO_OTYPER_OT10;

	/* Disable the internal pull-up and pull-down resistors */
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD11;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD11_1;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD10;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_1;

	/* Configure the GPIO to use high speed mode */
	GPIOB->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED10_Pos);
	GPIOB->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED11_Pos);

	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11;
	GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL11_Pos);//GPIO_AFRH_AFSEL11_2;
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL10;
	GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL10_Pos);//GPIO_AFRH_AFSEL10_2;

	// Configure I2C
	I2C2->CR1 &= ~I2C_CR1_PE;


	// Turn on Master Mode timers
	I2C2->TIMINGR |= I2C_TIMINGR_SCLH;
	I2C2->TIMINGR |= I2C_TIMINGR_SCLL;

	// Set BAUD rate to 400khz
	// prescaler
	I2C2->TIMINGR |= (1 << I2C_TIMINGR_PRESC_Pos);//(0 << I2C_TIMINGR_PRESC_Pos);
	// low cycles
	I2C2->TIMINGR &= ~I2C_TIMINGR_SCLL;
	I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);
	// high cycles
	I2C2->TIMINGR &= ~I2C_TIMINGR_SCLH;
	I2C2->TIMINGR |= (0xF << I2C_TIMINGR_SCLH_Pos);
	// data hold cycles
	I2C2->TIMINGR &= ~I2C_TIMINGR_SDADEL;
	I2C2->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL_Pos);
	// data setup cycles
	I2C2->TIMINGR &= ~I2C_TIMINGR_SCLDEL;
	I2C2->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL_Pos);

//	// works for 800 khz
//	I2C2->TIMINGR |= (0 << I2C_TIMINGR_PRESC_Pos);//(0 << I2C_TIMINGR_PRESC_Pos);
//	// low cycles
//	I2C2->TIMINGR &= ~I2C_TIMINGR_SCLL;
//	I2C2->TIMINGR |= (0x6 << I2C_TIMINGR_SCLL_Pos);
//	// high cycles
//	I2C2->TIMINGR &= ~I2C_TIMINGR_SCLH;
//	I2C2->TIMINGR |= (0x3 << I2C_TIMINGR_SCLH_Pos);
//	// data hold cycles
//	I2C2->TIMINGR &= ~I2C_TIMINGR_SDADEL;
//	I2C2->TIMINGR |= (0x0 << I2C_TIMINGR_SDADEL_Pos);
//	// data setup cycles
//	I2C2->TIMINGR &= ~I2C_TIMINGR_SCLDEL;
//	I2C2->TIMINGR |= (0x1 << I2C_TIMINGR_SCLDEL_Pos);



	// Set slave byte control
	//I2C2->CR1 |= I2C_CR1_SBC;

	// Enable Reload
	I2C2->CR2 |= I2C_CR2_AUTOEND;

	// Turn it to 7 bit addressing mode
	I2C2->CR2 &= ~I2C_CR2_ADD10;

	// Enable interrupts
//	I2C2->CR1 |= I2C_CR1_TCIE; // transfer complete interrupt sets TC flag when NBYTES transferred
//	I2C2->CR1 |= I2C_CR1_STOPIE;
//	I2C2->CR1 |= I2C_CR1_NACKIE;
//	I2C2->CR1 |= I2C_CR1_ADDRIE;
//	I2C2->CR1 |= I2C_CR1_RXIE;
//	I2C2->CR1 |= I2C_CR1_TXIE;

	// Enable peripheral
	I2C2->CR1 |= I2C_CR1_PE;

}

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len) {
	while ((I2C2->ISR & I2C_ISR_BUSY)) {}
	I2C2->CR2 |= I2C_CR2_AUTOEND;
	I2C2->CR2 &= ~I2C_CR2_ADD10;

	int count = 0;

	// set amount of expecting bytes
	I2C2->CR2 &= ~I2C_CR2_NBYTES;
	I2C2->CR2 |= len << I2C_CR2_NBYTES_Pos;

	// set device addr
	I2C2->CR2 &= ~I2C_CR2_SADD;
	I2C2->CR2 |= (address << 1);

	// if dir == 0 then slave in receiver, else it is transmitter
	if (dir == 0) {
		I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
		I2C2->CR2 |= I2C_CR2_START;
		while (count < len) {
			// if TXIS = 1, then it it looking for something to be written the TXDR register
			if (I2C2->ISR & I2C_ISR_TXIS) {
				I2C2->TXDR = *(data+count) & 0xFF;
				count++;
			}
		}
	}
	else if (dir == 1) {
		// write register addr
		I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
		I2C2->CR2 |= I2C_CR2_START;
		while (!(I2C2->ISR & I2C_ISR_TXIS)) {}
		I2C2->TXDR = data[0] & 0xFF;

		// repeated start
		I2C2->CR2 |= (0x1 << I2C_CR2_RD_WRN_Pos);
		I2C2->CR2 |= I2C_CR2_START;
		while(count < len) {
			if (I2C2->ISR & I2C_ISR_RXNE) {
				//data[count] = 0;
				data[count] = I2C2->RXDR & 0xFF;

				count++;
			}
		}
	}

	//while (!(I2C2->ISR & I2C_ISR_STOPF)) {}

	// Clear the stop flag
//	I2C2->ICR |= I2C_ICR_STOPCF;
	// clear out address by setting ADDRCF bit
//	I2C2->ISR |= I2C_ICR_ADDRCF;
	//I2C2->CR2 &= ~I2C2->CR2;
	return 0;
}
