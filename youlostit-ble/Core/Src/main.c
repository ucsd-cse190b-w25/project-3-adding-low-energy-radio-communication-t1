/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "ble_commands.h"
/*
 * 172, 165, 166, 405, 264 LPTIM1SMEN,
 * 261 for disabling gpio while asleep
 * 1195 for lptim enable
 * https://community.st.com/t5/mems-sensors/lsm6dsl-interrupt-generation-interrupts-constantly-fire/td-p/616753
 * https://medium.com/@yashodhalakshana/stm32l4-low-power-modes-with-different-wake-up-sources-260b56c48933
 * https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf#page=1198
 *
 *
 */
#include "ble.h"

#include <stdlib.h>

/* Include LED driver */
#include "leds.h"
#include "timer.h"
#include "lsm6dsl.h"
#include "lptimer.h"

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

// Redefine the libc _write() function so you can use printf in your code
int _write(int file, char *ptr, int len) {
    int i = 0;
    for (i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
void you_lost_it(int16_t* xyz);
void new_lost_it(void);
void LPTIM1_init(void);

#define OFFSET_THRESH 4000

// Patterns for each LED
static int led2[] = {2,0,2,0, 0,2,0,0,0,0,2,2, 0,0,0,0};
static int led1[] = {0,1,0,1, 0,0,1,1,0,0,0,0, 0,0,0,0};

static volatile int interruptProcs = 0;
static volatile int TIM2start = 0;
static volatile int TIM3start = 0;
static volatile int led_interrupt = 0;
static volatile int on_off = 0;
static volatile int minsLost = 0;
static volatile int lights = 0;
static volatile uint8_t highbit = 0x2;
static volatile uint8_t lowbit = 0x1;
static volatile int sendMessage = 0;
static volatile int checkAccel = 0;
static volatile int readAccel = 0;
static volatile int timeTillLost = 10;


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();

  // Our peripheral configurables
//  leds_init();
  lptim_init(LPTIM1);
  lptim_set_sec(LPTIM1, 5);

  lsm6dsl_init();

  //RESET BLE MODULE
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);

  ble_init();

  HAL_Delay(10);
  __enable_irq();

  uint8_t nonDiscoverable = 0;

  int16_t prev_xyz[3] = {0,0,0};


  while (1)
  {
//	  printf("minsLost: %d, CNT: %d\n",minsLost, LPTIM1->CNT);
	  if(!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
		  catchBLE();
	  }else{
		  if (checkAccel) {
			  you_lost_it(prev_xyz);
			  checkAccel = 0;
		  }
	  }

	  HAL_SuspendTick();
	  HAL_PWREx_EnterSTOP2Mode(PWR_SLEEPENTRY_WFI);
	  HAL_ResumeTick();
  }
}

void you_lost_it(int16_t* xyz){
	int16_t prev_x = xyz[0];
	int16_t prev_y = xyz[1];
	int16_t prev_z = xyz[2];
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
	lsm6dsl_read_xyz(&x,&y, &z);
	int16_t diff_x = abs(x) - abs(prev_x);
	int16_t diff_y = abs(y) - abs(prev_y);
	int16_t diff_z = abs(z) - abs(prev_z);

    //leds_set(lights);
	// keep track of how many times that it moved
	if (diff_x + diff_y + diff_z >= OFFSET_THRESH) { // This is checking for when it moves
		sendMessage = 0;
		led_interrupt = 0;
		minsLost = 0;
		disconnectBLE();
		setDiscoverability(0);
//		standbyBle();
	}
	if (led_interrupt && minsLost >= 10) { // This is when it is lost for 60s (10 seconds)
		setDiscoverability(1);
		unsigned char message[20] = ""; //21 characters seems like the max
		if (sendMessage) {
			snprintf((char*)message, 20, "Secs lost %d", minsLost-timeTillLost);
			updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, sizeof(message)-1, message);
			sendMessage = 0;
		}
	}
	xyz[0] = x;
	xyz[1] = y;
	xyz[2] = z;
}
//void new_lost_it() {
//	if (readAccel) {
//		timer_reset(TIM2);
//		timer_reset(TIM3);
//		sendMessage = 0;
//		led_interrupt = 0;
//		minsLost = 0;
////		leds_set(0);
//		PWR->CR1 &= ~PWR_CR1_LPR;
//		while ((PWR->SR2 & PWR_SR2_REGLPF) != 0) {}
//		RCC->CR &= ~RCC_CR_MSIRANGE;
//		RCC->CR |= RCC_CR_MSIRANGE_7;
//		timer_set_presc(TIM2, 7999);
//		timer_set_presc(TIM3, 7999);
//		disconnectBLE();
//		setDiscoverability(0);
//		readAccel = 0;
//	}
//
//	if (led_interrupt && minsLost >= 10) { // This is when it is lost for 60s (10 seconds)
//		//HAL_Delay(10);
//		PWR->CR1 &= ~PWR_CR1_LPR;
//		while ((PWR->SR2 & PWR_SR2_REGLPF) != 0) {}
//		RCC->CR &= ~RCC_CR_MSIRANGE;
//		RCC->CR |= RCC_CR_MSIRANGE_7;
//		timer_set_presc(TIM2, 7999);
//		timer_set_presc(TIM3, 7999);
//		setDiscoverability(1);
////		leds_set(lights);
//		unsigned char message[20] = ""; //21 characters seems like the max
//		if (sendMessage) {
//			snprintf((char*)message, 20, "Secs lost %d", minsLost-60);
//			updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, sizeof(message)-1, message);
//			sendMessage = 0;
//		}
//	}
//}
// Timer to keep track of how long it has been lost and set the blinking
//void TIM2_IRQHandler(void)
//{
//	if (TIM2start == 0) {
//		TIM2start++;
//		TIM2->SR &= ~TIM_SR_UIF;
//		return;
//	}
//	minsLost+= 7;
////	for (int i = 0; i < 3; i++) {
////		led1[15-i] = (minsLost & (lowbit << 2*i)) ? 1 : 0;
////		led2[15-i] = (minsLost & (highbit << 2*i)) ? 2 : 0;
////	}
////	lights = led1[on_off] + led2[on_off];
////	on_off = (on_off + 1) % 16;
////	leds_set(1);
//	led_interrupt = 1;
//	checkAccel = 1;
////	if (minsLost >= timeTillLost && (minsLost % 10) == 0) {
////		sendMessage = 1;
//////		leds_set(3);
////	}
//	// Reset the interrupt bit
//	TIM2->SR &= ~TIM_SR_UIF;
//}

// set the leds blinking pattern
//void TIM3_IRQHandler(void) {
//	if (!TIM3start) {
//		TIM3start = 1;
//		TIM3->SR &= ~TIM_SR_UIF;
//		return;
//	}
////	lights = led1[on_off] + led2[on_off];
////	on_off = (on_off + 1) % 16;
//	leds_set(2);
//	sendMessage = 1;
//	TIM3->SR &= ~TIM_SR_UIF;
//}

void LPTIM1_IRQHandler(void) {
//	leds_set(1);
	minsLost+= 5;
	led_interrupt = 1;
	checkAccel = 1;
	if (minsLost >= timeTillLost && ((minsLost % 10) == 0)) {
//		leds_set(3);
		sendMessage = 1;
	}
	LPTIM1->ICR |= LPTIM_ICR_ARRMCF;
}

//void EXTI15_10_IRQHandler(void) {
////    leds_set(3);
//
//    if (EXTI->PR1 & EXTI_PR1_PIF11) {
//    	readAccel = 1;
////    	leds_set(3);
////    	printf("accel interrupt");
//    	EXTI->PR1 |= EXTI_PR1_PIF11;
//    }
//}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	if (GPIO_Pin == GPIO_PIN_11) {
//		readAccel = 1;
////		leds_set(3);
//	}
//}


//void check_clocks() {
//    uint32_t sysclk = HAL_RCC_GetSysClockFreq();  // Get SYSCLK frequency
//    uint32_t hclk = HAL_RCC_GetHCLKFreq();        // Get AHB bus frequency
//    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();      // Get APB1 peripheral frequency
//    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();      // Get APB2 peripheral frequency
//
//    printf("System Clock (SYSCLK): %lu Hz\n", sysclk);
//    printf("AHB Clock (HCLK): %lu Hz\n", hclk);
//    printf("APB1 Clock (PCLK1): %lu Hz\n", pclk1);
//    printf("APB2 Clock (PCLK2): %lu Hz\n", pclk2);
//
//    if (__HAL_RCC_GET_FLAG(RCC_FLAG_MSIRDY)) {
//        printf("MSI is enabled\n");
//    }
//    if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY)) {
//        printf("HSI16 is enabled\n");
//    }
//    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)) {
//        printf("LSI is enabled\n");
//    }
//    if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY)) {
//        printf("HSE is enabled\n");
//    }
//    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY)) {
//        printf("LSE is enabled\n");
//    }
//
//}



/**
  * @brief System Clock Configuration
  * @attention This changes the System clock frequency, make sure you reflect that change in your timer
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  // This lines changes system clock frequency
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7; // MSIRANGE_7 is 8 mhz
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  __HAL_RCC_HSI_DISABLE();
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin|BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
