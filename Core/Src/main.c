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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MPU_6050_ADDR (0x68	<< 1)// I2C Address shifted 1 bit to the left per HAL I2C 8 bit requirement
#define POWER_CONFIG_ADDR 0x6B   // Power config for reset and clock select
#define GYRO_CONFIG_ADDR 0x1B    // Gyroscope config register
#define ACCEL_CONFIG_ADDR 0x1C   // Accelerometer config register
#define GYRO_ADDR 0x43			 // Gyroscope Data starting register 0x43 - 0x48
#define ACCEL_ADDR 0x3B			 // Accelerometer Data starting register 0x3B - 0x40
#define I2C_DELAY 50			 // I2C Delay 50ms
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t buf[6];
float elapsedTime, currentTime, previousTime;
float roll, pitch, yaw;
float gyroAngleX, gyroAngleY, gyroAngleZ;
int Speed;
float AccelErrorX;
float AccelErrorY;
float AccelErrorZ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void MPU_6050_Init(void);
void MPU_Get_Accel(void);
void MPU_Get_Gyro(void);
void Get_Pos(void);
void Get_Speed(void);
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
  char msg[50];
  int pot;
  int Throttle;
  Speed = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  //MPU_6050_Init();
  HAL_TIM_Base_Start(&htim2);
  float sec = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	pot = HAL_ADC_GetValue(&hadc1);

	pot = (pot - 1500) / 30;
	if (pot < 0)
	{
		pot = 0;
	}
	else if (pot > 80)
	{
		pot = 80;
	}



	//Get_Speed();
	sprintf(msg, "Pot: %i\r\n", pot);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	//if (HAL_UART_Receive(&huart1, &pot, 1, HAL_MAX_DELAY) == HAL_OK) {
	Throttle = pot + 80;
	TIM3->CCR4 = Throttle;
	//} else {
	    // Error handling if data reception fails
	//}
	HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1600-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1600-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MPU_6050_Init(void)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	//uint8_t data = 0;
	//ret = HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, POWER_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &data, I2C_MEMADD_SIZE_8BIT, I2C_DELAY);
	uint8_t data = 0x00;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, POWER_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &data, I2C_MEMADD_SIZE_8BIT, I2C_DELAY);
	if (ret != HAL_OK)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"Error Initializing MPU_6050\n", strlen("Error Initializing MPU_6050\n"), I2C_DELAY);
	}
	else
	{
		data = 0x08;
		ret = HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, GYRO_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &data, I2C_MEMADD_SIZE_8BIT, I2C_DELAY);
		if (ret != HAL_OK)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)"Error Initializing Gyro\n", strlen("Error Initializing Gyro\n"), I2C_DELAY);
		}
		else
		{
			ret = HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, ACCEL_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &data, I2C_MEMADD_SIZE_8BIT, I2C_DELAY);
			if (ret != HAL_OK)
			{
				HAL_UART_Transmit(&huart2, (uint8_t*)"Error Initializing Accel\n", strlen("Error Initializing Accel\n"), I2C_DELAY);
			}
			else
			{
				HAL_UART_Transmit(&huart2, (uint8_t*)"Initialized MPU_6050!\n", strlen("Initialized MPU_6050!\n"), I2C_DELAY);
			}
		}
	}

	int x = 0;
	AccelErrorX = 0;
	AccelErrorY = 0;
	AccelErrorZ = 0;
	int16_t dAccX;
	int16_t dAccY;
	int16_t dAccZ;
	while (x < 100)
	{
		HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, ACCEL_ADDR, I2C_MEMADD_SIZE_8BIT, buf, 6, I2C_DELAY);
		dAccX = ((int16_t)buf[0] << 8) | (buf[1]);
		dAccY = ((int16_t)buf[2] << 8) | (buf[3]);
		dAccZ = ((int16_t)buf[4] << 8) | (buf[5]);

		float AccX = dAccX / 8192.0;
		float AccY = dAccY / 8192.0;
		float AccZ = dAccZ / 8192.0;
		AccelErrorX += AccX;
		AccelErrorY += AccY;
		AccelErrorZ += AccZ;
		x++;
	}

	AccelErrorX = AccelErrorX / 100;
	AccelErrorY = AccelErrorY / 100;
	AccelErrorZ = AccelErrorZ / 100;
	return;
}

void MPU_Get_Accel(void)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	int16_t dAccX;
	int16_t dAccY;
	int16_t dAccZ;

	ret = HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, ACCEL_ADDR, I2C_MEMADD_SIZE_8BIT, buf, 6, I2C_DELAY);

;
	if (ret != HAL_OK)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"Error Reading Accel\n", strlen("Error Reading Accel\n"), I2C_DELAY);
	}
	else
	{
		dAccX = ((int16_t)buf[0] << 8) | (buf[1]);
		dAccY = ((int16_t)buf[2] << 8) | (buf[3]);
		dAccZ = ((int16_t)buf[4] << 8) | (buf[5]);

		float AccX = dAccX / 8192.0;
		float AccY = dAccY / 8192.0;
		float AccZ = dAccZ / 8192.0;

		char data[25];
		float Speedp = Speed;

		Speed = (AccY * 1) + Speedp;
		sprintf((char*)data, "%f Y: %f Z: %f\r\n", AccX, AccY, AccZ);
		HAL_UART_Transmit(&huart2, data, strlen(data), I2C_DELAY);
	}
	return;
}

void MPU_Get_Gyro(void)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	float gyroX = 0;
	float gyroY = 0;
	float gyroZ = 0;

	ret = HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, GYRO_ADDR, I2C_MEMADD_SIZE_8BIT, buf, 6, I2C_DELAY);
	if (ret != HAL_OK)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"Error Reading Gyro\n", strlen("Error Reading Gyro\n"), I2C_DELAY);
	}
	else
	{
		gyroX = ((int16_t)buf[0] << 8) | (buf[1]);
		gyroY = ((int16_t)buf[2] << 8) | (buf[3]);
		gyroZ = ((int16_t)buf[4] << 8) | (buf[5]);

		gyroX = gyroX / 65.5;
		gyroY = gyroY / 65.5;
		gyroZ = gyroZ / 65.5;

		char data[256];
		sprintf((char*)data, "Gyroscope X: %f Y: %f Z: %f\r\n", gyroX, gyroY, gyroZ);
		HAL_UART_Transmit(&huart2, data, strlen((char*)data), I2C_DELAY);
	}
	return;
}

void Get_Pos(void)
{
	float accAngleX, accAngleY;
	HAL_StatusTypeDef ret = HAL_ERROR;
	int16_t dAccX;
	int16_t dAccY;
	int16_t dAccZ;
	int16_t dGyroX;
	int16_t dGyroY;
	int16_t dGyroZ;

	ret = HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, ACCEL_ADDR, I2C_MEMADD_SIZE_8BIT, buf, 6, I2C_DELAY);
	dAccX = ((int16_t)buf[0] << 8) | (buf[1]);
	dAccY = ((int16_t)buf[2] << 8) | (buf[3]);
	dAccZ = ((int16_t)buf[4] << 8) | (buf[5]);

	float AccX = dAccX / 8192.0;
	float AccY = dAccY / 8192.0;
	float AccZ = dAccZ / 8192.0;

	accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / M_PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
	accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / M_PI) + 1.58; // AccErrorY ~(-1.58)

	previousTime = currentTime;        // Previous time is stored before the actual time read
	currentTime = TIM2->CNT;       // Current time actual time read
	currentTime = currentTime / 8;
	elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

	ret = HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, GYRO_ADDR, I2C_MEMADD_SIZE_8BIT, buf, 6, I2C_DELAY);
	dGyroX = ((int16_t)buf[0] << 8) | (buf[1]);
	dGyroY = ((int16_t)buf[2] << 8) | (buf[3]);
	dGyroZ = ((int16_t)buf[4] << 8) | (buf[5]);

	float GyroX = dGyroX / 65.5;
	float GyroY = dGyroY / 65.5;
	float GyroZ = dGyroZ / 65.5;

	GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
	GyroY = GyroY - 2; // GyroErrorY ~(2)
	GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)

	// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
	gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
	gyroAngleY = gyroAngleY + GyroY * elapsedTime;
	yaw =  yaw + GyroZ * elapsedTime;
	// Complementary filter - combine acceleromter and gyro angle values
	roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
	pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

	char data[256];
	sprintf((char*)data, "Acell X: %f Y: %f Z: %f\nGyroscope X: %f Y: %f Z: %f\r\nMilli: %f\r\n", AccX, AccY, AccZ, GyroX, GyroY, GyroZ, elapsedTime);
	HAL_UART_Transmit(&huart2, data, strlen((char*)data), I2C_DELAY);

	return;
}

void Get_Speed(void)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	int16_t dAccX;
	int16_t dAccY;
	int16_t dAccZ;


	ret = HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, ACCEL_ADDR, I2C_MEMADD_SIZE_8BIT, buf, 6, I2C_DELAY);
	dAccX = ((int16_t)buf[0] << 8) | (buf[1]);
	dAccY = ((int16_t)buf[2] << 8) | (buf[3]);
	dAccZ = ((int16_t)buf[4] << 8) | (buf[5]);

	float AccX = (dAccX / 8192.0) - AccelErrorX;
	float AccY = (dAccY / 8192.0) - AccelErrorY;
	float AccZ = (dAccZ / 8192.0) - AccelErrorZ;

	roll = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / M_PI) - AccelErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
	pitch = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / M_PI) - AccelErrorY; // AccErrorY ~(-1.58)

	previousTime = currentTime;        // Previous time is stored before the actual time read
	currentTime = TIM2->CNT;       // Current time actual time read
	currentTime = currentTime / 8;
	elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

	char data[256];
	//float Speedp = Speed;
	float SpeedX;
	float SpeedY;
	SpeedX = (AccX * 20);
	SpeedY = (AccY * 20);

	Speed = SpeedX - SpeedY;

	if (Speed < 0)
	{
		Speed *= -1;
	}

	//Speed = abs(sqrt((vx * vx) + (vy * vy) + (vz * vz))) + Speedp;
	//sprintf((char*)data, "Roll: %f Pitch %f \r\n", roll, pitch);
	//HAL_UART_Transmit(&huart2, data, strlen(data), I2C_DELAY);
	//sprintf((char*)data, "%i mph\r\n", Speed);
	//HAL_UART_Transmit(&huart2, data, strlen(data), I2C_DELAY);
}
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
