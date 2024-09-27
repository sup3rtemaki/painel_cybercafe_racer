/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
uint8_t flagDetectPulso = 1;
uint8_t flagDetectPulsoAux;
const uint8_t rpmLedCount = 18;
const uint8_t fuelLedCount = 7;
uint8_t rpmReadingsCounter = 0;
uint8_t standSensorDebounceCounter = 0;

uint16_t fuelReading;
const uint16_t maxRPM = 10000;
uint16_t rpmLedPins[] = {
		RPM_Led_1_Pin,
		RPM_Led_2_Pin,
		RPM_Led_3_Pin,
		RPM_Led_4_Pin,
		RPM_Led_5_Pin,
		RPM_Led_6_Pin,
		RPM_Led_7_Pin,
		RPM_Led_8_Pin,
		RPM_Led_9_Pin,
		RPM_Led_10_Pin,
		RPM_Led_11_Pin,
		RPM_Led_12_Pin,
		RPM_Led_13_Pin,
		RPM_Led_14_Pin,
		RPM_Led_15_Pin,
		RPM_Led_16_Pin,
		RPM_Led_17_Pin,
		RPM_Led_18_Pin
};
uint16_t fuelLedPins[] = {
		Fuel_Led_1_Pin,
		Fuel_Led_2_Pin,
		Fuel_Led_3_Pin,
		Fuel_Led_4_Pin,
		Fuel_Led_5_Pin,
		Fuel_Led_6_Pin,
		Fuel_Led_7_Pin
};

uint32_t intervaloPulso = 1;
uint32_t valorRPM;
uint32_t oldRPM1;
uint32_t oldRPM2;
uint32_t oldRPM3;
uint32_t oldRPM4;
uint32_t oldRPM5;
const uint32_t timerFrequency = 16000000;
const uint32_t rpmLedPorts[] = {
		RPM_Led_1_GPIO_Port,
		RPM_Led_2_GPIO_Port,
		RPM_Led_3_GPIO_Port,
		RPM_Led_4_GPIO_Port,
		RPM_Led_5_GPIO_Port,
		RPM_Led_6_GPIO_Port,
		RPM_Led_7_GPIO_Port,
		RPM_Led_8_GPIO_Port,
		RPM_Led_9_GPIO_Port,
		RPM_Led_10_GPIO_Port,
		RPM_Led_11_GPIO_Port,
		RPM_Led_12_GPIO_Port,
		RPM_Led_13_GPIO_Port,
		RPM_Led_14_GPIO_Port,
		RPM_Led_15_GPIO_Port,
		RPM_Led_16_GPIO_Port,
		RPM_Led_17_GPIO_Port,
		RPM_Led_18_GPIO_Port
};
const uint32_t fuelLedPorts[] = {
		Fuel_Led_1_GPIO_Port,
		Fuel_Led_2_GPIO_Port,
		Fuel_Led_3_GPIO_Port,
		Fuel_Led_4_GPIO_Port,
		Fuel_Led_5_GPIO_Port,
		Fuel_Led_6_GPIO_Port,
		Fuel_Led_7_GPIO_Port
};

const int minFuelReading = 3000;
const int maxFuelReading = 455;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
int map(int x, int in_min, int in_max, int out_min, int out_max);
void initialize(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  initialize();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/* ROTINA MEDIDOR COMBUSTIVEL */
	// Realiza leitura
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	fuelReading = (uint16_t)HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	// Conversao do valor lido para quantidade de LEDs
	int cursorCombustivel = map(fuelReading, minFuelReading, maxFuelReading, 1, fuelLedCount);

	// Acende LEDs
	for(uint8_t j = 0; j < fuelLedCount; j++)
	{
		if(j < cursorCombustivel)
		{
			HAL_GPIO_WritePin((uint32_t)fuelLedPorts[j], (uint16_t)fuelLedPins[j], GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin((uint32_t)fuelLedPorts[j], (uint16_t)fuelLedPins[j], GPIO_PIN_RESET);
		}
	}

	/* ------------------------------------------------------------------------------------------------ */

	/* ROTINA MARCADOR RPM */
	// Salva leitura de pulso anterior
	flagDetectPulsoAux = flagDetectPulso;

	// Le sinal no pino, se houve ou nao pulso
	if(GPIO_PIN_SET == HAL_GPIO_ReadPin(RPM_Pulse_Detector_GPIO_Port, RPM_Pulse_Detector_Pin))
	{
		flagDetectPulso = 1;
	}
	else
	{
		flagDetectPulso = 0;
	}

	// Caso houve pulso, realiza calculo do RPM
	if(1 == flagDetectPulso && 0 == flagDetectPulsoAux)
	{
//		oldRPM5 = oldRPM4;
//		oldRPM4 = oldRPM3;
//		oldRPM3 = oldRPM2;
//		oldRPM2 = oldRPM1;
//		oldRPM1 = (16000 / intervaloPulso) * 20; //* 60 * 2;
//
//		valorRPM = (oldRPM1 + oldRPM2 + oldRPM3 + oldRPM4 + oldRPM5) / 5;

		valorRPM = (16000 / intervaloPulso) * 24;

		intervaloPulso = 1;

		if(valorRPM < 700) valorRPM = 700;

		HAL_GPIO_TogglePin(Board_Led_GPIO_Port, Board_Led_Pin);
	}

	// Verificacao de seguranca
	if(intervaloPulso > timerFrequency)
	{
		intervaloPulso = 1;
	}

	// Incrementa contagem entre pulsos
	intervaloPulso++;

	// Conversao do valor calculado para quantidade de LEDs
	int cursorRPM = map(valorRPM, 0, maxRPM, 0, rpmLedCount);

	// Acende LEDs
	for(uint8_t j = 0; j < rpmLedCount; j++)
	{
		if(j < cursorRPM)
		{
			HAL_GPIO_WritePin((uint32_t)rpmLedPorts[j], (uint16_t)rpmLedPins[j], GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin((uint32_t)rpmLedPorts[j], (uint16_t)rpmLedPins[j], GPIO_PIN_RESET);
		}
	}

	/* ------------------------------------------------------------------------------------------------ */

	/* ROTINA SENSOR PEZINHO */
	if(GPIO_PIN_SET == HAL_GPIO_ReadPin(Stand_Sensor_GPIO_Port, Stand_Sensor_Pin))
	{
	  standSensorDebounceCounter++;
	}
	else
	{
	  standSensorDebounceCounter = 0;
	}

	if(standSensorDebounceCounter >= 15)
	{
	  HAL_GPIO_WritePin(Stand_Led_GPIO_Port, Stand_Led_Pin, GPIO_PIN_SET);
	  standSensorDebounceCounter = 15;
	}
	else
	{
	  HAL_GPIO_WritePin(Stand_Led_GPIO_Port, Stand_Led_Pin, GPIO_PIN_RESET);
	}


	/* ------------------------------------------------------------------------------------------------ */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Board_Led_GPIO_Port, Board_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RPM_Led_1_Pin|RPM_Led_2_Pin|Fuel_Led_1_Pin|Fuel_Led_2_Pin
                          |Fuel_Led_3_Pin|Fuel_Led_4_Pin|Fuel_Led_5_Pin|RPM_Led_15_Pin
                          |RPM_Led_14_Pin|RPM_Led_13_Pin|RPM_Led_12_Pin|RPM_Led_11_Pin
                          |RPM_Led_10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Fuel_Led_6_Pin|Fuel_Led_7_Pin|Stand_Led_Pin|RPM_Led_18_Pin
                          |RPM_Led_17_Pin|RPM_Led_16_Pin|RPM_Led_9_Pin|RPM_Led_8_Pin
                          |RPM_Led_7_Pin|RPM_Led_6_Pin|RPM_Led_5_Pin|RPM_Led_4_Pin
                          |RPM_Led_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Board_Led_Pin */
  GPIO_InitStruct.Pin = Board_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Board_Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RPM_Led_1_Pin RPM_Led_2_Pin Fuel_Led_1_Pin Fuel_Led_2_Pin
                           Fuel_Led_3_Pin Fuel_Led_4_Pin Fuel_Led_5_Pin RPM_Led_15_Pin
                           RPM_Led_14_Pin RPM_Led_13_Pin RPM_Led_12_Pin RPM_Led_11_Pin
                           RPM_Led_10_Pin */
  GPIO_InitStruct.Pin = RPM_Led_1_Pin|RPM_Led_2_Pin|Fuel_Led_1_Pin|Fuel_Led_2_Pin
                          |Fuel_Led_3_Pin|Fuel_Led_4_Pin|Fuel_Led_5_Pin|RPM_Led_15_Pin
                          |RPM_Led_14_Pin|RPM_Led_13_Pin|RPM_Led_12_Pin|RPM_Led_11_Pin
                          |RPM_Led_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Fuel_Led_6_Pin Fuel_Led_7_Pin Stand_Led_Pin RPM_Led_18_Pin
                           RPM_Led_17_Pin RPM_Led_16_Pin RPM_Led_9_Pin RPM_Led_8_Pin
                           RPM_Led_7_Pin RPM_Led_6_Pin RPM_Led_5_Pin RPM_Led_4_Pin
                           RPM_Led_3_Pin */
  GPIO_InitStruct.Pin = Fuel_Led_6_Pin|Fuel_Led_7_Pin|Stand_Led_Pin|RPM_Led_18_Pin
                          |RPM_Led_17_Pin|RPM_Led_16_Pin|RPM_Led_9_Pin|RPM_Led_8_Pin
                          |RPM_Led_7_Pin|RPM_Led_6_Pin|RPM_Led_5_Pin|RPM_Led_4_Pin
                          |RPM_Led_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Stand_Sensor_Pin RPM_Pulse_Detector_Pin */
  GPIO_InitStruct.Pin = Stand_Sensor_Pin|RPM_Pulse_Detector_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  int result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

  if(result < out_min) return out_min;
  if(result > out_max) return out_max;
  return result;
}

void initialize(void)
{
	  HAL_GPIO_WritePin(Board_Led_GPIO_Port, Board_Led_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Stand_Led_GPIO_Port, Stand_Led_Pin, GPIO_PIN_SET);

	  uint8_t rpmIndex = 0;
	  uint8_t fuelIndex = 0;

	  for(rpmIndex; rpmIndex < rpmLedCount; rpmIndex++)
	  {
		  HAL_GPIO_WritePin((uint32_t)rpmLedPorts[rpmIndex], (uint16_t)rpmLedPins[rpmIndex], GPIO_PIN_SET);

		  fuelIndex = map(rpmIndex, 0, rpmLedCount - 1, 0, fuelLedCount - 1);

		  HAL_GPIO_WritePin((uint32_t)fuelLedPorts[fuelIndex], (uint16_t)fuelLedPins[fuelIndex], GPIO_PIN_SET);

		  HAL_Delay(50);
	  }

	  for(rpmIndex = rpmLedCount - 1; rpmIndex > 0; rpmIndex--)
	  {
		  HAL_GPIO_WritePin((uint32_t)rpmLedPorts[rpmIndex], (uint16_t)rpmLedPins[rpmIndex], GPIO_PIN_RESET);

		  fuelIndex = map(rpmIndex, 0, rpmLedCount - 1, 0, fuelLedCount - 1);

		  HAL_GPIO_WritePin((uint32_t)fuelLedPorts[fuelIndex], (uint16_t)fuelLedPins[fuelIndex], GPIO_PIN_RESET);

		  HAL_Delay(50);
	  }

	  HAL_GPIO_WritePin(Board_Led_GPIO_Port, Board_Led_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Stand_Led_GPIO_Port, Stand_Led_Pin, GPIO_PIN_RESET);

}

/*===============================================*/
/*============= TEMPORIZADORES ==================*/
/*===============================================*/
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance == TIM2) //FREQ=10KHz
//	{
//		flagDetectPulsoAux = flagDetectPulso;
//
//		if(GPIO_PIN_SET == HAL_GPIO_ReadPin(Detector_Pulso_GPIO_Port, Detector_Pulso_Pin))
//		{
//			flagDetectPulso = 1;
//		}
//		else
//		{
//			flagDetectPulso = 0;
//		}
//
//		if(1 == flagDetectPulso && 0 == flagDetectPulsoAux)
//		{
//			valorRPM = (((timerFrequency / intervaloPulso) * 60) * 2);
//			intervaloPulso = 0;
//		}
//
//		if(intervaloPulso > timerFrequency)
//		{
//			intervaloPulso = 1;
//		}
//
//		intervaloPulso++;
//	}
//}
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
	  HAL_Delay(1000);
	  HAL_GPIO_TogglePin((uint32_t)rpmLedPorts[0], (uint16_t)rpmLedPins[0]);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
