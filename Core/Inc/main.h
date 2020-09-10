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

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Channel11_Pin GPIO_PIN_0
#define Channel11_GPIO_Port GPIOC
#define Channel12_Pin GPIO_PIN_1
#define Channel12_GPIO_Port GPIOC
#define Channel13_Pin GPIO_PIN_2
#define Channel13_GPIO_Port GPIOC
#define Channel14_Pin GPIO_PIN_3
#define Channel14_GPIO_Port GPIOC
#define Channel1_Pin GPIO_PIN_0
#define Channel1_GPIO_Port GPIOA
#define Channel2_Pin GPIO_PIN_1
#define Channel2_GPIO_Port GPIOA
#define Channel3_Pin GPIO_PIN_2
#define Channel3_GPIO_Port GPIOA
#define Channel4_Pin GPIO_PIN_3
#define Channel4_GPIO_Port GPIOA
#define Channel5_Pin GPIO_PIN_4
#define Channel5_GPIO_Port GPIOA
#define Channel6_Pin GPIO_PIN_5
#define Channel6_GPIO_Port GPIOA
#define Channel7_Pin GPIO_PIN_6
#define Channel7_GPIO_Port GPIOA
#define Channel8_Pin GPIO_PIN_7
#define Channel8_GPIO_Port GPIOA
#define Channel15_Pin GPIO_PIN_4
#define Channel15_GPIO_Port GPIOC
#define Channel16_Pin GPIO_PIN_5
#define Channel16_GPIO_Port GPIOC
#define Channel9_Pin GPIO_PIN_0
#define Channel9_GPIO_Port GPIOB
#define Channel10_Pin GPIO_PIN_1
#define Channel10_GPIO_Port GPIOB
#define Switch1_Pin GPIO_PIN_8
#define Switch1_GPIO_Port GPIOE
#define Switch2_Pin GPIO_PIN_9
#define Switch2_GPIO_Port GPIOE
#define Switch3_Pin GPIO_PIN_10
#define Switch3_GPIO_Port GPIOE
#define Switch4_Pin GPIO_PIN_11
#define Switch4_GPIO_Port GPIOE
#define Switch5_Pin GPIO_PIN_12
#define Switch5_GPIO_Port GPIOE
#define Switch6_Pin GPIO_PIN_13
#define Switch6_GPIO_Port GPIOE
#define Switch7_Pin GPIO_PIN_14
#define Switch7_GPIO_Port GPIOE
#define Switch8_Pin GPIO_PIN_15
#define Switch8_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
