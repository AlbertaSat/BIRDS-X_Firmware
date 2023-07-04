/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define BUFFER_SIZE 100
#define CHECK_FOR_BEG_CHAR(value) ((value) == 0x21)
#define CHECK_FOR_END_CHAR(value) ((value) == 0x3B)
#define OP_ADD 0x01
#define WRITE 0x80
#define READ 0x00
#define TRANSMIT_MODE 0x03
#define STDBY_MODE 0x01
#define DUMMY 0x00

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

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


typedef enum {
    WAIT,
    READY,
    RECIEVING,
} State;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

void ADC_Select_Ch4(void);
void ADC_Select_Ch5(void);
void ADC_Select_Ch6(void);
void ADC_Select_Ch7(void);
void ADC_Select_Temp(void);
bool CHECK_TRANSMISSION_MISSION_BOSS(void);
bool CHECK_SX1276(void);
bool CHECK_ADCS(void);
float raw_to_Voltage(uint16_t rawValue);
bool Power_On_Self_Test(void);

/* USER CODE BEGIN PFP */
uint8_t TX1[BUFFER_SIZE] = "Hello Mission Boss\n";
uint8_t TX2[BUFFER_SIZE] = "Success\n";
uint8_t TX3[BUFFER_SIZE] = "Failure\n";
uint8_t ADC_WORKS[BUFFER_SIZE] = "ADC channels are initialized and working\n";
uint8_t ADC_FAILED[BUFFER_SIZE] = "Failed initializing ADC channels\n";
uint8_t msg[BUFFER_SIZE];
uint8_t POST_SUCCESS[BUFFER_SIZE] = "Power on self test successfull.\n";

uint8_t SX1276_SUCCESS[BUFFER_SIZE] = "SX1276 Communication Successful\n";
uint8_t SX1276_FAILURE[BUFFER_SIZE] = "SX1276 Communication Failure\n";
uint16_t SX1276Transmit;
uint16_t SX1276Receive;

uint16_t ADC_VAL[5];
volatile State currentState = WAIT;
volatile char commandBuffer[BUFFER_SIZE];
volatile uint8_t cmndindex = 0;
volatile uint8_t rx2CompleteFlag = 0;
volatile char  RX1;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart)
{
if (huart->Instance == USART1) {
	switch(currentState)
	{
	case WAIT:
		if(CHECK_FOR_BEG_CHAR(RX1))
		{
			currentState = READY;
		}
		else
		{
		}
		break;
	case READY:
		if(CHECK_FOR_END_CHAR(RX1))
			{
				currentState = WAIT;
				cmndindex = 0;
				rx2CompleteFlag = 1;
				return;
			}
			else if(CHECK_FOR_BEG_CHAR(RX1))
			{
				currentState = READY;

			}
			else
			{
				currentState = RECIEVING;
				cmndindex = 0;
				commandBuffer[cmndindex] = RX1;
				cmndindex++;
			}

		break;
	case RECIEVING:
		if(CHECK_FOR_END_CHAR(RX1))
		{
			currentState = WAIT;
			cmndindex = 0;
			rx2CompleteFlag = 1;
			return;
		}
		else if(CHECK_FOR_BEG_CHAR(RX1))
		{
			currentState = READY;
			cmndindex = 0;
			memset(commandBuffer, 0, sizeof(commandBuffer));

		}
		else
		{
			currentState = RECIEVING;
			commandBuffer[cmndindex] = RX1;
			cmndindex++;
		}
		break;
	}
	HAL_UART_Receive_IT(&huart1,&RX1, 1);
}
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  bool ret = false;

  //ret = CHECK_TRANSMISSION_MISSION_BOSS();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  ret = Power_On_Self_Test();

  if(ret)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t *)TX2, strlen(TX2), HAL_MAX_DELAY);
  }
  else
  {
	  Error_Handler();
  }


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

bool CHECK_TRANSMISSION_MISSION_BOSS(void)
{
	uint8_t attempts = 0;


	while(attempts < 5)
	{
	if(HAL_UART_Transmit(&huart1, (uint8_t *)TX1, strlen(TX1), HAL_MAX_DELAY) != HAL_OK)
	{

		return false;
	}

	HAL_UART_Receive_IT(&huart1,&RX1, 1);

	uint32_t startTime = HAL_GetTick();
	uint32_t elapsedTime;

	while(1)
	{
		elapsedTime = HAL_GetTick() - startTime;

		if(rx2CompleteFlag)
		{
			memset(RX1, 0, sizeof(RX1));
			memset(commandBuffer, 0, sizeof(commandBuffer));
			rx2CompleteFlag = 0;
			return true;
		}

		else if(elapsedTime >= 5000)
		{
			break;
		}

	}
	attempts++;
	}

	return true;
}

bool CHECK_SX1276(void)
{
	SX1276Transmit = (((WRITE| OP_ADD) << 8) | TRANSMIT_MODE);

	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	 HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&SX1276Transmit, (uint8_t*)&SX1276Receive,1, HAL_MAX_DELAY);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);





	 SX1276Transmit = (((READ | OP_ADD) << 8) | DUMMY);

	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	 HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&SX1276Transmit,(uint8_t*)&SX1276Receive ,1, HAL_MAX_DELAY);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);




	 uint8_t RECV = (uint8_t)SX1276Receive;
	 uint8_t COMP = TRANSMIT_MODE;

	 if(RECV == COMP)
	 {
		 return true;
	 }

	 else
	 {
		 return false;
	 }



}

bool Power_On_Self_Test(void)
{

	  if (CHECK_TRANSMISSION_MISSION_BOSS())
	  {
			HAL_UART_Transmit(&huart1, (uint8_t *)TX2, strlen(TX2), HAL_MAX_DELAY);
		}

		else
		{
			return false;
		}

	    if(CHECK_SX1276)
	    {
	  	  HAL_UART_Transmit(&huart1, (uint8_t *)SX1276_SUCCESS, strlen(SX1276_SUCCESS), HAL_MAX_DELAY);
	    }
	    else
	    {
	  	  return false;
	    }

	  if(CHECK_ADCS())
	  {
		 HAL_UART_Transmit(&huart1, (uint8_t *)ADC_WORKS, strlen(ADC_WORKS), HAL_MAX_DELAY);
		 sprintf(msg, "ADC4:%x, ADC5:%x, ADC6:%x, ADC7:%x, ADC_Temp:%x\n",ADC_VAL[0],ADC_VAL[1],ADC_VAL[2],ADC_VAL[3],ADC_VAL[4]);
		 HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	  }
	  else
	  {
		 return false;
	  }
}

void ADC_Select_Ch4(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank =  1;
	  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

void ADC_Select_Ch5(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	  sConfig.Channel = ADC_CHANNEL_5;
	  sConfig.Rank =  1;
	  //sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

void ADC_Select_Ch6(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	  sConfig.Channel = ADC_CHANNEL_6;
	  sConfig.Rank =  1;
	  //sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

void ADC_Select_Ch7(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	  sConfig.Channel = ADC_CHANNEL_7;
	  sConfig.Rank =  1;
	  //sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

void ADC_Select_Temp(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	  sConfig.Rank =  1;
	  //sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

bool CHECK_ADCS(void)
{
	ADC_Select_Ch4();
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	ADC_VAL[0] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_Select_Ch5();
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	ADC_VAL[1] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_Select_Ch6();
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	ADC_VAL[2] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_Select_Ch7();
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	ADC_VAL[3] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_Select_Temp();
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	ADC_VAL[4] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	return true;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : T_NRST_Pin */
  GPIO_InitStruct.Pin = T_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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
