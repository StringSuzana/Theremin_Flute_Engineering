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
typedef enum note_name {
	NOTE_C, NOTE_D, NOTE_E, NOTE_F, NOTE_G, NOTE_A, NOTE_C2, NOTE_BB
} NOTE_NAME;

typedef struct note {
	NOTE_NAME name;
	uint32_t central_freq;
	int holes_to_play[7];
} NOTE;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TIMCLOCK   72000000 //This should be valid for all timers. But better check it.
#define PRESCALAR  1 // Input is 0 in Prescalar field in CubeMx

#define bool int
#define true 1
#define false 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint32_t IC_val1 = 0;
uint32_t IC_val2 = 0;
uint32_t period_ticks = 0;
int is_first_captured = 0;

float frequency = 0;
uint8_t finished_one_measurement = 0;

const char *timers[3] = { "HTIM_1", "HTIM_2", "WRONG" };
uint8_t current_timer = 0;

const uint32_t PITCH_LOWEST_FREQUENCY = 1400; //2 kHz
//Span 2000 - 4000 Hz
//note span 300
// central frequency - 150 & central_frequency + 149
const uint32_t CTR_FREQ[8] = { 1500, 1750, 1890, 1975, 2100, 2225, 2325 };

uint32_t pitch_val = { 0 };

const uint32_t BACK_ROW_CLOSE_POS = 25;
const uint32_t BACK_ROW_OPEN_POS = 40;

const uint32_t FRONT_ROW_CLOSE_POS = 40;
const uint32_t FRONT_ROW_OPEN_POS = 25;

//Hard coded notes to play
const NOTE note_c = { .name = NOTE_C, .central_freq = CTR_FREQ[0],
		.holes_to_play = { FRONT_ROW_CLOSE_POS, FRONT_ROW_CLOSE_POS,
				BACK_ROW_CLOSE_POS, FRONT_ROW_CLOSE_POS, BACK_ROW_CLOSE_POS,
				FRONT_ROW_CLOSE_POS, BACK_ROW_CLOSE_POS } };
const NOTE note_d = { .name = NOTE_D, .central_freq = CTR_FREQ[1],
		.holes_to_play = { FRONT_ROW_CLOSE_POS, FRONT_ROW_CLOSE_POS,
				BACK_ROW_CLOSE_POS, FRONT_ROW_CLOSE_POS, BACK_ROW_CLOSE_POS,
				FRONT_ROW_CLOSE_POS, BACK_ROW_OPEN_POS } };
const NOTE note_e = { .name = NOTE_E, .central_freq = CTR_FREQ[2],
		.holes_to_play = { FRONT_ROW_CLOSE_POS, FRONT_ROW_CLOSE_POS,
				BACK_ROW_CLOSE_POS, FRONT_ROW_CLOSE_POS, BACK_ROW_CLOSE_POS,
				FRONT_ROW_OPEN_POS, BACK_ROW_OPEN_POS } };
const NOTE note_f = { .name = NOTE_F, .central_freq = CTR_FREQ[3],
		.holes_to_play = { FRONT_ROW_CLOSE_POS, FRONT_ROW_CLOSE_POS,
				BACK_ROW_CLOSE_POS, FRONT_ROW_CLOSE_POS, BACK_ROW_OPEN_POS,
				FRONT_ROW_CLOSE_POS, BACK_ROW_CLOSE_POS } };
const NOTE note_g = { .name = NOTE_G, .central_freq = CTR_FREQ[4],
		.holes_to_play = { FRONT_ROW_CLOSE_POS, FRONT_ROW_CLOSE_POS,
				BACK_ROW_CLOSE_POS, FRONT_ROW_OPEN_POS, BACK_ROW_OPEN_POS,
				FRONT_ROW_OPEN_POS, BACK_ROW_OPEN_POS } };
const NOTE note_a = { .name = NOTE_A, .central_freq = CTR_FREQ[5],
		.holes_to_play = { FRONT_ROW_CLOSE_POS, FRONT_ROW_CLOSE_POS,
				BACK_ROW_OPEN_POS, FRONT_ROW_OPEN_POS, BACK_ROW_OPEN_POS,
				FRONT_ROW_OPEN_POS, BACK_ROW_OPEN_POS } };
const NOTE note_bb = { .name = NOTE_BB, .central_freq = CTR_FREQ[6],
		.holes_to_play = { FRONT_ROW_OPEN_POS, FRONT_ROW_OPEN_POS,
				BACK_ROW_OPEN_POS, FRONT_ROW_OPEN_POS, BACK_ROW_OPEN_POS,
				FRONT_ROW_OPEN_POS, BACK_ROW_OPEN_POS } };

/*Without thumb, this is the same as note A
 * const NOTE note_c2 = { .name = NOTE_C2, .central_freq = CTR_FREQ[6],
 .holes_to_play = { FRONT_ROW_CLOSE_POS, FRONT_ROW_OPEN_POS,
 BACK_ROW_OPEN_POS, FRONT_ROW_OPEN_POS, BACK_ROW_OPEN_POS,
 FRONT_ROW_OPEN_POS, BACK_ROW_OPEN_POS } };*/

const NOTE all_notes[7] = { note_c, note_d, note_e, note_f, note_g, note_a,
		note_bb };

NOTE_NAME current_note = NOTE_C;
uint32_t current_note_index = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void calibrate_antenna();
void print_to_com_port();
bool is_in_range(int lower_limit, int upper_limit, int number);
void play_music();
void play_all_notes();
void play_all_notes_with_delay();
void test_servos();
void play_note(uint32_t note_to_play_index);
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
	MX_TIM1_Init();
	MX_USB_DEVICE_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	TIM1->CCR1 = 50;
	//Frequency measurement on TIM1_CHANNEL_1
	//HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); //dead pin
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //closest to AIR (FIRST)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //closest to END (LAST)
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	//Initialize ALL servo-motor positions to closed
	htim3.Instance->CCR1 = FRONT_ROW_CLOSE_POS;
	htim3.Instance->CCR2 = FRONT_ROW_CLOSE_POS;
	htim3.Instance->CCR3 = BACK_ROW_CLOSE_POS;
	htim3.Instance->CCR4 = FRONT_ROW_CLOSE_POS;

	htim4.Instance->CCR1 = BACK_ROW_CLOSE_POS;
	htim4.Instance->CCR2 = FRONT_ROW_CLOSE_POS;
	htim4.Instance->CCR3 = BACK_ROW_CLOSE_POS;
	//htim4.Instance->CCR4 = CLOSE_POS;
	HAL_GPIO_WritePin(GPIOB, B14_GREEN_PITCH_LED_Pin, GPIO_PIN_SET);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		//test_servos();
		//If switch is in calibrating position, then calibrate antenna.
		if (HAL_GPIO_ReadPin(B12_SWITCH_GPIO_Port, B12_SWITCH_Pin)
				== GPIO_PIN_SET) {
			calibrate_antenna();
			print_to_com_port();
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); //indikacija switcha
			// HAL_GPIO_WritePin(GPIOB, B14_GREEN_PITCH_LED_Pin, GPIO_PIN_RESET);
		}
		//Play instrument
		else {
			HAL_GPIO_WritePin(GPIOB, B14_GREEN_PITCH_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			if (finished_one_measurement == 1) {
				play_music();
			}
		}

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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65536 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

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
	htim2.Init.Period = 65536 - 1;
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
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
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
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 1440 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1000 - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, B13_RED_PITCH_LED_Pin | B14_GREEN_PITCH_LED_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : B12_SWITCH_Pin */
	GPIO_InitStruct.Pin = B12_SWITCH_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(B12_SWITCH_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : B13_RED_PITCH_LED_Pin B14_GREEN_PITCH_LED_Pin */
	GPIO_InitStruct.Pin = B13_RED_PITCH_LED_Pin | B14_GREEN_PITCH_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : TIM1_CH1_PA9_INPUT_CAPTURE_Pin */
	GPIO_InitStruct.Pin = TIM1_CH1_PA9_INPUT_CAPTURE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TIM1_CH1_PA9_INPUT_CAPTURE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void calibrate_antenna() {
	if (is_in_range(PITCH_LOWEST_FREQUENCY - 50, PITCH_LOWEST_FREQUENCY + 50,
			frequency) == true) {
		HAL_GPIO_WritePin(GPIOB, B14_GREEN_PITCH_LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);
	} else {
		//Turn OFF GREEN LED, turn ON RED LED for PITCH
		HAL_GPIO_WritePin(GPIOB, B14_GREEN_PITCH_LED_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
	}
}
bool is_in_range(int lower_limit, int upper_limit, int number) {
	return (lower_limit <= number && number <= upper_limit);
}

void play_note(uint32_t note_to_play_index) {
	//HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);
	//HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	//is_first_captured = 0;

	htim3.Instance->CCR1 = all_notes[note_to_play_index].holes_to_play[0];
	htim3.Instance->CCR2 = all_notes[note_to_play_index].holes_to_play[1];
	htim3.Instance->CCR3 = all_notes[note_to_play_index].holes_to_play[2];
	htim3.Instance->CCR4 = all_notes[note_to_play_index].holes_to_play[3];

	htim4.Instance->CCR1 = all_notes[note_to_play_index].holes_to_play[4];
	htim4.Instance->CCR2 = all_notes[note_to_play_index].holes_to_play[5];
	htim4.Instance->CCR3 = all_notes[note_to_play_index].holes_to_play[6];

	//SHORT DELAY
	HAL_Delay(200);
	//HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	//HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}
void test_servos() {
	/*	for (int i = 25; i<=100; i=i+20){

	 htim3.Instance->CCR1 = OPEN_POS;
	 htim3.Instance->CCR2 = OPEN_POS;
	 htim3.Instance->CCR3 = OPEN_POS;
	 htim3.Instance->CCR4 = OPEN_POS;

	 htim4.Instance->CCR1 = OPEN_POS;
	 htim4.Instance->CCR2 = OPEN_POS;
	 htim4.Instance->CCR3 = OPEN_POS;
	 HAL_Delay(500);
	 }


	 htim3.Instance->CCR1 = FRONT_ROW_CLOSE_POS;
	 htim3.Instance->CCR2 = FRONT_ROW_CLOSE_POS;
	 htim3.Instance->CCR3 = BACK_ROW_CLOSE_POS;
	 htim3.Instance->CCR4 = FRONT_ROW_CLOSE_POS;

	 htim4.Instance->CCR1 = BACK_ROW_CLOSE_POS;
	 htim4.Instance->CCR2 = FRONT_ROW_CLOSE_POS;
	 htim4.Instance->CCR3 = BACK_ROW_CLOSE_POS;
	 HAL_Delay(100);

	 htim3.Instance->CCR1 = FRONT_ROW_OPEN_POS;
	 htim3.Instance->CCR2 = FRONT_ROW_OPEN_POS;
	 htim3.Instance->CCR3 = BACK_ROW_OPEN_POS;
	 htim3.Instance->CCR4 = FRONT_ROW_OPEN_POS;

	 htim4.Instance->CCR1 = BACK_ROW_OPEN_POS;
	 htim4.Instance->CCR2 = FRONT_ROW_OPEN_POS;
	 htim4.Instance->CCR3 = BACK_ROW_OPEN_POS;
	 HAL_Delay(100); */
	// C = 0
	//D = 1
	// E = 2
	// F = 3
	// G = 4
	// A = 5
	// B = 6
	play_note(5);
	HAL_Delay(1500);
	play_note(6);
	HAL_Delay(500);
}
void play_all_notes() {
	for (int i = 0; i < 7; i++) {
		play_note(i);
	}
}
void play_all_notes_with_delay() {
	for (int i = 0; i < 7; i++) {
		play_note(i);
		HAL_Delay(2000);
	}
}
void play_music() {
	const uint32_t DWN = 50;
	const uint32_t UP = 50;

	/*if (frequency < (CTR_FREQ[0] - 500)) {
	 return;
	 }*/
	/*== NOTE C ==*/
	if (is_in_range(CTR_FREQ[0] - DWN, CTR_FREQ[0] + UP, frequency) == true) {
		current_note = NOTE_C;
		current_note_index = 0;
	}
	/*== NOTE D ==*/
	else if (is_in_range(CTR_FREQ[1] - DWN, CTR_FREQ[1] + UP, frequency) == true) {
		current_note = NOTE_D;
		current_note_index = 1;
	}
	/*== NOTE E ==*/
	else if (is_in_range(CTR_FREQ[2] - DWN, CTR_FREQ[2] + UP, frequency) == true) {
		current_note = NOTE_E;
		current_note_index = 2;
	}
	/*== NOTE F ==*/
	else if (is_in_range(CTR_FREQ[3] - DWN, CTR_FREQ[3] + UP, frequency) == true) {
		current_note = NOTE_F;
		current_note_index = 3;
	}
	/*== NOTE G ==*/
	else if (is_in_range(CTR_FREQ[4] - DWN, CTR_FREQ[4] + UP, frequency) == true) {
		current_note = NOTE_G;
		current_note_index = 4;
	}
	/*== NOTE A ==*/
	else if (is_in_range(CTR_FREQ[5] - DWN, CTR_FREQ[5] + UP, frequency) == true) {
		current_note = NOTE_A;
		current_note_index = 5;
	}
	/*== NOTE BB ==*/
	else if (is_in_range(CTR_FREQ[6] - DWN, CTR_FREQ[6] + UP + 400, frequency) == true) {
		current_note = NOTE_BB;
		current_note_index = 6;
	}

	play_note(current_note_index);

}
void print_to_com_port() {
	char buffer[100];
	sprintf(buffer, "%f;%lu;%s \r\n", frequency, period_ticks,
			timers[current_timer]);
	CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));

}
void print_note_to_com_port() {
	char buffer[50];
	sprintf(buffer, "%%lu \r\n", all_notes[current_note]);
	CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		current_timer = 1;
	} else {
		return;
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {

		if (is_first_captured == 0) // if the first rising edge is not captured
				{
			finished_one_measurement = 0;
			IC_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			is_first_captured = 1;  // set the first captured as true

		}

		else // If the first rising edge is captured, now we will capture the second edge
		{
			IC_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value

			if (IC_val2 > IC_val1) {
				period_ticks = IC_val2 - IC_val1;
			}

			else if (IC_val1 > IC_val2) {
				period_ticks = (0xffff - IC_val1) + IC_val2; //0xffffffff = 4 294 967 295
			}

			float refClock = TIMCLOCK / (PRESCALAR);

			frequency = refClock / period_ticks;

			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			is_first_captured = 0; // set it back to false

			finished_one_measurement = 1;
		}
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
