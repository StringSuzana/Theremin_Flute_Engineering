/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

const uint32_t POS_PLAY_3 = 55;
const uint32_t POS_PLAY_2 = 45;
const uint32_t POS_PLAY = 35;
const uint32_t POS_SILENCE = 25;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMCLOCK   72000000 //This should be valid for all timers. But better check it.
#define PRESCALAR  1 // Input is 0 in Prescalar field in CubeMx

#define BOOLEAN int
#define true 1
#define false 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint32_t vol_IC_val1 = 0;
uint32_t vol_IC_val2 = 0;
uint32_t vol_period_ticks = 0;
int vol_is_first_captured = 0;

float vol_frequency = 0;
uint8_t vol_finished_one_measurement = 0;

const char *vol_timers[3] = { "HTIM_1", "HTIM_2", "WRONG" };
uint8_t vol_current_timer = 0;

const uint32_t VOL_LOWEST_FREQUENCY = 2000; // Hand CLOSE to antenna
uint32_t VOL_VALUES[3] = { VOL_LOWEST_FREQUENCY, VOL_LOWEST_FREQUENCY + 150,
		VOL_LOWEST_FREQUENCY + 300 };
uint32_t vol_value = { 0 };
uint8_t first_ever = 1;
uint32_t current_pos = POS_SILENCE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void vol_calibrate_antenna();
BOOLEAN vol_is_in_range(int lower_limit, int upper_limit, int number);
void vol_play();
void vol_play_hw(uint32_t vol_position);
void test_volume_servo();
void print_to_com_port();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_USB_DEVICE_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	//Frequency measurement on TIM2_CHANNEL_1
	TIM2->CCMR1 = 50;
	//HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // INPUT CAPTURE
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1); // INPUT CAPTURE

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //PWM SERVO

	//Initialize servo-motor positions to SILENCE
	htim3.Instance->CCR1 = POS_SILENCE;
	HAL_GPIO_WritePin(GPIOB, B14_GREEN_VOL_LED_Pin, GPIO_PIN_SET);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		//If switch is in calibrating position, then calibrate antenna.
		if (HAL_GPIO_ReadPin(B12_SWITCH_GPIO_Port, B12_SWITCH_Pin)
				== GPIO_PIN_SET) {
			HAL_GPIO_WritePin(GPIOB, B13_CALIBRATE_LED_Pin, GPIO_PIN_RESET);
			HAL_Delay(10);
			vol_calibrate_antenna();
			print_to_com_port();
		}
		//Play instrument
		else {
			HAL_GPIO_WritePin(GPIOB, B13_CALIBRATE_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, B14_GREEN_VOL_LED_Pin, GPIO_PIN_SET);
			//test_volume_servo();
			if (vol_finished_one_measurement == 1) {
				vol_play();
			}
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1440 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, B13_CALIBRATE_LED_Pin | B14_GREEN_VOL_LED_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B12_SWITCH_Pin */
	GPIO_InitStruct.Pin = B12_SWITCH_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(B12_SWITCH_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : B13_CALIBRATE_LED_Pin B14_GREEN_VOL_LED_Pin */
	GPIO_InitStruct.Pin = B13_CALIBRATE_LED_Pin | B14_GREEN_VOL_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void print_to_com_port() {
	char buffer[100];
	sprintf(buffer, "%f;%lu; \r\n", vol_frequency, vol_period_ticks);
	CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));

}
BOOLEAN vol_is_in_range(int lower_limit, int upper_limit, int number) {
	return (lower_limit <= number && number <= upper_limit);
}
void vol_calibrate_antenna() {
	const uint32_t TOLERANCE = 60;
	if (vol_is_in_range(VOL_LOWEST_FREQUENCY, VOL_LOWEST_FREQUENCY + TOLERANCE,
			vol_frequency) == true) {
		current_pos = POS_SILENCE;
		HAL_GPIO_WritePin(GPIOB, B14_GREEN_VOL_LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);
	} else {
		//Turn OFF GREEN LED
		HAL_GPIO_WritePin(GPIOB, B14_GREEN_VOL_LED_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
	}

}
void vol_play_hw(uint32_t vol_position) {
	htim3.Instance->CCR1 = vol_position;
	HAL_Delay(200);
	//vol_is_first_captured = 0;
	//__HAL_TIM_SET_COUNTER(&htim2, 0);

}
void test_volume_servo() {
	for (int i = 25; i < 100; i++) {
		htim3.Instance->CCR1 = i;
		HAL_Delay(200);
	}
}
void vol_play() {
	const uint32_t STEP_SIZE = 40;
	if ((vol_frequency > VOL_LOWEST_FREQUENCY) && (vol_frequency < 2800)) {
		current_pos = POS_SILENCE;
		vol_play_hw(current_pos);
	} else if ((vol_frequency < VOL_LOWEST_FREQUENCY)
			&& (vol_frequency > 1400)) {
		current_pos = POS_PLAY;
		vol_play_hw(current_pos);
	}

	/*if (vol_is_in_range(VOL_VALUES[0]-STEP_SIZE-1000,
	 VOL_VALUES[0] +STEP_SIZE, vol_frequency) == true) {
	 current_pos = POS_SILENCE;
	 } else if (vol_is_in_range(VOL_VALUES[1]-STEP_SIZE,
	 VOL_VALUES[1] + STEP_SIZE, vol_frequency) == true) {
	 current_pos = POS_PLAY;
	 } else if (vol_is_in_range(VOL_VALUES[2]-STEP_SIZE,
	 VOL_VALUES[2] + STEP_SIZE, vol_frequency) == true) {
	 current_pos = POS_PLAY_2;
	 }
	 vol_play_hw(current_pos);
	 */

	/*if (vol_is_in_range(VOL_LOWEST_FREQUENCY, VOL_LOWEST_FREQUENCY + 600,
	 vol_frequency)== true) {

	 float res = (vol_frequency / 20) - 70;
	 vol_play_hw(res);
	 }
	 else if(vol_frequency > VOL_LOWEST_FREQUENCY + 650){
	 vol_play_hw(60);
	 }
	 else if(vol_frequency < VOL_LOWEST_FREQUENCY -50){
	 vol_play_hw(30);
	 }*/
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim2 || htim == &htim4) {

		if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) {
			return;
		}

		/*static size_t sample_count = 0;
		 static uint32_t accomulator = 0;

		 uint32_t value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		 accomulator += value;

		 if (sample_count++ >= 32)
		 {
		 sample_count = 0;
		 accomulator = accomulator / 32;
		 const float refClock = TIMCLOCK / (PRESCALAR);
		 vol_frequency = refClock / accomulator;
		 }*/

		if (vol_is_first_captured == 0) // if the first rising edge is not captured
				{
			vol_finished_one_measurement = 0;
			vol_IC_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			vol_is_first_captured = 1;  // set the first captured as true
		}

		else // If the first rising edge is captured, now we will capture the second edge
		{
			vol_IC_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value

			if (vol_IC_val2 > vol_IC_val1) {
				vol_period_ticks = vol_IC_val2 - vol_IC_val1;
			}

			else if (vol_IC_val1 > vol_IC_val2) {
				vol_period_ticks = (0xffff - vol_IC_val1) + vol_IC_val2; //0xffff 65535
			}

			float refClock = TIMCLOCK / (PRESCALAR);

			vol_frequency = refClock / vol_period_ticks;

			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			vol_is_first_captured = 0;

			if (vol_frequency < 1) {
				vol_finished_one_measurement = 0;
			} else {
				vol_finished_one_measurement = 1;
			}

		}
	} else {
		return;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
