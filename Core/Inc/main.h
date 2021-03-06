/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern int RTRENC_CW_EVENT;
extern int RTRENC_CCW_EVENT;
extern int RTRENC_PUSHB_EVENT;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void increase_octave();
void decrease_octave();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define LD_OCTAVE_DOWN_Pin GPIO_PIN_0
#define LD_OCTAVE_DOWN_GPIO_Port GPIOF
#define LD_OCTAVE_UP_Pin GPIO_PIN_1
#define LD_OCTAVE_UP_GPIO_Port GPIOF
#define LD_SW_LFO_WV_Pin GPIO_PIN_2
#define LD_SW_LFO_WV_GPIO_Port GPIOF
#define LD_SW_VCO1_Pin GPIO_PIN_7
#define LD_SW_VCO1_GPIO_Port GPIOF
#define LD_SW_VCO2_Pin GPIO_PIN_8
#define LD_SW_VCO2_GPIO_Port GPIOF
#define LD_SW_LFO_TRGT_Pin GPIO_PIN_9
#define LD_SW_LFO_TRGT_GPIO_Port GPIOF
#define OCTAVE_DOWN_PUSHB_Pin GPIO_PIN_1
#define OCTAVE_DOWN_PUSHB_GPIO_Port GPIOC
#define OCTAVE_DOWN_PUSHB_EXTI_IRQn EXTI1_IRQn
#define OCTAVE_DOWN_PUSHBC2_Pin GPIO_PIN_2
#define OCTAVE_DOWN_PUSHBC2_GPIO_Port GPIOC
#define OCTAVE_DOWN_PUSHBC2_EXTI_IRQn EXTI2_IRQn
#define RTRENC_PSH_Pin GPIO_PIN_3
#define RTRENC_PSH_GPIO_Port GPIOC
#define RTRENC_PSH_EXTI_IRQn EXTI3_IRQn
#define RTRENC_A_Pin GPIO_PIN_4
#define RTRENC_A_GPIO_Port GPIOC
#define RTRENC_A_EXTI_IRQn EXTI4_IRQn
#define RTRENC_B_Pin GPIO_PIN_5
#define RTRENC_B_GPIO_Port GPIOC
#define RTRENC_B_EXTI_IRQn EXTI9_5_IRQn
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define DAC_GATE_Pin GPIO_PIN_12
#define DAC_GATE_GPIO_Port GPIOF
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
