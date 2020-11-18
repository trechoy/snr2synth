/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "lcd.h"
#include "visEffect.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_PRESETS 7
#define NUM_PARAMETERS 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

/* FLAGS */
int RTRENC_CCW_EVENT = 0;
int RTRENC_CW_EVENT = 0;
int RTRENC_PUSHB_EVENT = 0;
int DEMO_MODE = 0;

/* GLOBAL VARIABLES */
int CURRENT_PRESET = 1;

/* PRESET VALUES */
const char *presetBank[NUM_PRESETS] = {"Grand Piano",
									   "Electric Guitar",
									   "Lead Synth",
									   "Heavy 808",
									   "Accordion",
									   "DAC_Test",
									   "LED_Test"
										};

const char parameterVals [NUM_PRESETS - 2][NUM_PARAMETERS] = {
 {1, 2, 3, 4, 5},
 {6, 7, 8, 9, 10},
 {1, 3, 5, 7, 9},
 {2, 4, 6, 8, 10},
 {0, 10, 5, 2, 7}
};

const char *parameterNames[NUM_PARAMETERS] = {"VCO 1 Frequency",
											  "VCO 1 Volume",
											  "VCO 2 Frequency",
											  "VCO 2 Volume",
											  "LFO Frequency"
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void updateMenuDisplay()
{
	char * buf[80] = {0};;
	sprintf(buf, "#%d\r%s\r\rPush knob to select.", CURRENT_PRESET, presetBank[CURRENT_PRESET - 1]);
	lcd_showMessage(buf, huart4);

}

void clockwise_menu_event()
{
	CURRENT_PRESET++;
	if (CURRENT_PRESET > NUM_PRESETS)
	{
		CURRENT_PRESET = 1;
	}
	updateMenuDisplay();
}

void anticlockwise_menu_event()
{
	CURRENT_PRESET--;
	if (CURRENT_PRESET < 1)
	{
		CURRENT_PRESET = NUM_PRESETS;
	}
	updateMenuDisplay();
}

void reset_rtrencFlags()
{
	RTRENC_CCW_EVENT = 0;
	RTRENC_CW_EVENT = 0;
	RTRENC_PUSHB_EVENT = 0;
}

void LED_Test()
{
	lcd_showMessage("Welcome to the\rLED Test Mode!", huart4);
	HAL_Delay(1000);
	lcd_clear(huart4);

	int numLEDS = 0;
	char * buf[80] = {0};
	sprintf(buf, "Rotate knob to\rchange number\rof LEDs.");
	lcd_showMessage(buf, huart4);
	while (1)
	{
		if (RTRENC_CW_EVENT)
		{
			numLEDS++;
			if (numLEDS > 12)
			{
				numLEDS = 12;
			}
		}
		else if (RTRENC_CCW_EVENT)
		{
			numLEDS--;
			if (numLEDS < 0)
			{
				numLEDS = 0;
			}
		}
		updateRingLED(0, numLEDS);
		reset_rtrencFlags();
		HAL_Delay(80);
	}
}

void DAC_Test()
{
	lcd_showMessage("Welcome to the\rDAC Test Mode!", huart4);
	HAL_Delay(1000);
	lcd_clear(huart4);

	double adjustment = 0.3;

	double dac_vout = 0;
	int dac_write = 0;
	char * buf[80] = {0};
	sprintf(buf, "DAC V_out: %.2fV\r\rRotate knob to\radjust voltage.", dac_vout);
	lcd_showMessage(buf, huart4);
	while (1)
	{
		if (RTRENC_CW_EVENT)
		{
			dac_vout += adjustment;
			if (dac_vout > 3.3)
			{
				dac_vout = 3.3;
			}
			sprintf(buf, "DAC V_out: %.2fV\r\rRotate knob to\radjust voltage.", dac_vout);
			lcd_showMessage(buf, huart4);

			dac_write = (int) (dac_vout / (3.3/4095));
			if (dac_write == 4096)
				DAC->DHR12R1 = 4095;
			else
				DAC->DHR12R1 = dac_write;
			HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		}
		else if (RTRENC_CCW_EVENT)
		{
			dac_vout -= adjustment;
			if (dac_vout < 0)
			{
				dac_vout = 0;
			}
			sprintf(buf, "DAC V_out: %.2fV\r\rRotate knob to\radjust voltage.", dac_vout);
			lcd_showMessage(buf, huart4);

			dac_write = (int) (dac_vout / (3.3/4095));
			if (dac_write == 4096)
				DAC->DHR12R1 = 4095;
			else
				DAC->DHR12R1 = dac_write;
			HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		}
		reset_rtrencFlags();
		HAL_Delay(150);
	}
}

void Demo_Mode(int preset)
{
	int preset_ind = preset - 1;
	char * buf[80] = {0};

	reset_rtrencFlags();
	sprintf(buf, "Showing parameters\rfor %s\r\rPress knob to start.", presetBank[preset_ind]);
	lcd_showMessage(buf, huart4);
	while (!RTRENC_PUSHB_EVENT)
	{
		HAL_Delay(100);
	}
	reset_rtrencFlags();

	for (int curParameter = 0; curParameter < NUM_PARAMETERS; curParameter++)
	{
		int parameterVal = parameterVals[preset_ind][curParameter];
		sprintf(buf, "Set the\r%s\rknob to %d.\rPress knob to go on.", parameterNames[curParameter], parameterVal);
		lcd_showMessage(buf, huart4);
		if (parameterVal != 0)
		{
			flashLED(curParameter);
			HAL_Delay(600);
			flashLED(curParameter);
			HAL_Delay(600);
			for (int i = 1; i <= parameterVal; i++)
			{
				updateRingLED(curParameter, i);
				HAL_Delay(80);
			}
		}

		reset_rtrencFlags();
		while (!RTRENC_PUSHB_EVENT)
		{
			HAL_Delay(100);
			if (parameterVal == 0)
			{
				flashLED(curParameter);
				HAL_Delay(500);
			}
		}
		reset_rtrencFlags();
		clearLEDs();
	}

	//probably some specific stuff for the waveform generators

	sprintf(buf, "Parameters for\r%s\rhave been set!\rPush knob to return.", presetBank[preset_ind]);
	lcd_showMessage(buf, huart4);
	reset_rtrencFlags();
	while (!RTRENC_PUSHB_EVENT)
	{
		HAL_Delay(100);
	}
	reset_rtrencFlags();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_UART4_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(800);
  lcd_changeColor('w', huart4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  lcd_clear(huart4);
  updateMenuDisplay();
  visInit();
  clearLEDs();
  HAL_Delay(150);
  updateRingLED(0, 0);

  while (1)
  {
	  if (RTRENC_CW_EVENT)
	  {
	 	 clockwise_menu_event();
	  }
	  else if (RTRENC_CCW_EVENT)
	  {
	  	 anticlockwise_menu_event();
	  }
	  else if (CURRENT_PRESET == 6 && RTRENC_PUSHB_EVENT)
	  {
		 DAC_Test();
	  }
	  else if (CURRENT_PRESET == 7 && RTRENC_PUSHB_EVENT)
	  {
		  LED_Test();
	  }
	  else if (RTRENC_PUSHB_EVENT)
	  {
		  //demo mode bb
		  Demo_Mode(CURRENT_PRESET);
		  updateMenuDisplay();
	  }

	  reset_rtrencFlags();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(125);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 31250;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RTRENC_PSH_Pin */
  GPIO_InitStruct.Pin = RTRENC_PSH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RTRENC_PSH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RTRENC_A_Pin RTRENC_B_Pin */
  GPIO_InitStruct.Pin = RTRENC_A_Pin|RTRENC_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
