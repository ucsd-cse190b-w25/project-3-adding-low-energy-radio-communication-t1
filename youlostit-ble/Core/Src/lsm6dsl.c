/*
 * lsm6dsl.c
 *
 *  Created on: Feb 2, 2025
 *      Author: skyle
 */
#include "lsm6dsl.h"
#include "i2c.h"

#define LSM6DSL_ADDR 0x6A
#define LSM6DSL_READ 0xD5
#define LSM6DSL_WRITE 0xD4
#define CTRL1_XL 0x10
#define INT1_CTRL 0x0D
#define CTRL2_G 0x11
#define CTRL6_C 0x15
#define CTRL8_XL 0x17
#define STATUS_REG 0x1E
#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D
#define WAKE_UP_DUR 0x5C
#define WAKE_UP_THS 0x5B
#define TAP_CFG 0x58
#define MD1_CFG 0x5E

void lsm6dsl_init() {
	i2c_init();
//	uint8_t data[2] = {CTRL1_XL, 0x60};
//	i2c_transaction(LSM6DSL_ADDR, 0, data, 2);
//	data[0] = INT1_CTRL;
//	data[1] = 0x01;
//	i2c_transaction(LSM6DSL_ADDR, 0, data, 2);
	uint8_t data[2] = {CTRL1_XL, 0x60};
	i2c_transaction(LSM6DSL_ADDR, 0, data, 2);
	data[0] = INT1_CTRL;
	data[1] = 0x41;
	i2c_transaction(LSM6DSL_ADDR, 0, data, 2);

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Clear mode bits (and also set to input mode 00)
	GPIOD->MODER &= ~(GPIO_MODER_MODE11_Msk);
	GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD11_Msk);
	GPIOD->PUPDR |= (GPIO_PUPDR_PUPD11_1);

	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PD;
	EXTI->IMR1 |= EXTI_IMR1_IM11;
	EXTI->RTSR1 |= EXTI_RTSR1_RT11;
	EXTI->FTSR1 |= EXTI_FTSR1_FT11;
	EXTI->PR1 |= EXTI_PR1_PIF11;

	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn, 0); // Adjust priority as needed

	data[0] = TAP_CFG;
	data[1] = 0x90;
	i2c_transaction(LSM6DSL_ADDR, 0, data, 2);

	data[0] = WAKE_UP_DUR;
	data[1] = 0x00;
	i2c_transaction(LSM6DSL_ADDR, 0, data, 2);
	data[0] = WAKE_UP_THS;
	data[1] = 0x02;
	i2c_transaction(LSM6DSL_ADDR, 0, data, 2);

	data[0] = MD1_CFG;
	data[1] = 0x20;
	i2c_transaction(LSM6DSL_ADDR, 0, data, 2);

//	data[0] = CTRL8_XL;
//	data[1] = 0x11 << 5;
//	i2c_transaction(LSM6DSL_ADDR, 0, data, 2);
//	printf("lsm6dsl init done\n");
}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z) {
	uint8_t data = STATUS_REG;
	while (data & 0x1) {
		printf("checking status reg %d \n", data);
		data = STATUS_REG;
		i2c_transaction(LSM6DSL_ADDR, 1, &data, 1);
	}
	uint8_t xl = OUTX_L_XL;
	i2c_transaction(LSM6DSL_ADDR, 1, &xl, 1);
	//printf("xl: %d ", xl);
	uint8_t xh = OUTX_H_XL;
	i2c_transaction(LSM6DSL_ADDR, 1, &xh, 1);
	//printf("xh: %d \n", xh);
	uint8_t yl = OUTY_L_XL;
	i2c_transaction(LSM6DSL_ADDR, 1, &yl, 1);
	//printf("yl: %d \n", yl);
	uint8_t yh = OUTY_H_XL;
	i2c_transaction(LSM6DSL_ADDR, 1, &yh, 1);
	//printf("yh: %d \n", yh);
	uint8_t zl = OUTZ_L_XL;
	i2c_transaction(LSM6DSL_ADDR, 1, &zl, 1);
	//printf("zl: %d \n", zl);
	uint8_t zh = OUTZ_H_XL;
	i2c_transaction(LSM6DSL_ADDR, 1, &zh, 1);
	//printf("zh: %d \n", zh);
	*x = xh << 8 | xl;
	*y = yh << 8 | yl;
	*z = zh << 8 | zl;
}
