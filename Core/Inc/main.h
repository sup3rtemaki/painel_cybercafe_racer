/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define Board_Led_Pin GPIO_PIN_13
#define Board_Led_GPIO_Port GPIOC
#define RPM_Led_1_Pin GPIO_PIN_0
#define RPM_Led_1_GPIO_Port GPIOA
#define RPM_Led_2_Pin GPIO_PIN_1
#define RPM_Led_2_GPIO_Port GPIOA
#define Fuel_Sensor_Pin GPIO_PIN_2
#define Fuel_Sensor_GPIO_Port GPIOA
#define Fuel_Led_1_Pin GPIO_PIN_3
#define Fuel_Led_1_GPIO_Port GPIOA
#define Fuel_Led_2_Pin GPIO_PIN_4
#define Fuel_Led_2_GPIO_Port GPIOA
#define Fuel_Led_3_Pin GPIO_PIN_5
#define Fuel_Led_3_GPIO_Port GPIOA
#define Fuel_Led_4_Pin GPIO_PIN_6
#define Fuel_Led_4_GPIO_Port GPIOA
#define Fuel_Led_5_Pin GPIO_PIN_7
#define Fuel_Led_5_GPIO_Port GPIOA
#define Fuel_Led_6_Pin GPIO_PIN_0
#define Fuel_Led_6_GPIO_Port GPIOB
#define Fuel_Led_7_Pin GPIO_PIN_1
#define Fuel_Led_7_GPIO_Port GPIOB
#define Stand_Sensor_Pin GPIO_PIN_10
#define Stand_Sensor_GPIO_Port GPIOB
#define Stand_Led_Pin GPIO_PIN_11
#define Stand_Led_GPIO_Port GPIOB
#define RPM_Pulse_Detector_Pin GPIO_PIN_12
#define RPM_Pulse_Detector_GPIO_Port GPIOB
#define RPM_Led_18_Pin GPIO_PIN_13
#define RPM_Led_18_GPIO_Port GPIOB
#define RPM_Led_17_Pin GPIO_PIN_14
#define RPM_Led_17_GPIO_Port GPIOB
#define RPM_Led_16_Pin GPIO_PIN_15
#define RPM_Led_16_GPIO_Port GPIOB
#define RPM_Led_15_Pin GPIO_PIN_8
#define RPM_Led_15_GPIO_Port GPIOA
#define RPM_Led_14_Pin GPIO_PIN_9
#define RPM_Led_14_GPIO_Port GPIOA
#define RPM_Led_13_Pin GPIO_PIN_10
#define RPM_Led_13_GPIO_Port GPIOA
#define RPM_Led_12_Pin GPIO_PIN_11
#define RPM_Led_12_GPIO_Port GPIOA
#define RPM_Led_11_Pin GPIO_PIN_12
#define RPM_Led_11_GPIO_Port GPIOA
#define RPM_Led_10_Pin GPIO_PIN_15
#define RPM_Led_10_GPIO_Port GPIOA
#define RPM_Led_9_Pin GPIO_PIN_3
#define RPM_Led_9_GPIO_Port GPIOB
#define RPM_Led_8_Pin GPIO_PIN_4
#define RPM_Led_8_GPIO_Port GPIOB
#define RPM_Led_7_Pin GPIO_PIN_5
#define RPM_Led_7_GPIO_Port GPIOB
#define RPM_Led_6_Pin GPIO_PIN_6
#define RPM_Led_6_GPIO_Port GPIOB
#define RPM_Led_5_Pin GPIO_PIN_7
#define RPM_Led_5_GPIO_Port GPIOB
#define RPM_Led_4_Pin GPIO_PIN_8
#define RPM_Led_4_GPIO_Port GPIOB
#define RPM_Led_3_Pin GPIO_PIN_9
#define RPM_Led_3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
