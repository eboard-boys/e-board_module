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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "spatial-lib/include/interface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define I2C_DELAY 50			 // I2C Delay 50ms
#define STM_LORA_ADDRESS 24      // Address of the STM Lora chip
#define ESP_LORA_ADDRESS 25      // Address of the ESP Lora chip
#define Min_PWM 80			 	 // PWM signal at 80 is 0 throttle for the ESC
#define Max_PWM 160		 		 // PWM signal at 160 is full throttle for the ESC
#define Min_Throttle 0			 // 0 throttle
#define Max_Throttle 80			 // 80 is the max throttle to add to the PWM
#define LORA_BUFFER_SIZE 100
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for readThrottle */
osThreadId_t readThrottleHandle;
const osThreadAttr_t readThrottle_attributes = {
  .name = "readThrottle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for sendSpeed */
osThreadId_t sendSpeedHandle;
const osThreadAttr_t sendSpeed_attributes = {
  .name = "sendSpeed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for accelUpdateTask */
osThreadId_t accelUpdateTaskHandle;
const osThreadAttr_t accelUpdateTask_attributes = {
  .name = "accelUpdateTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
char UART1_rxBuffer[LORA_BUFFER_SIZE]; // Raw data from LORA RX
char receive_data[4];    // Data stripped from LORA RX
int throttle;
float speed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void ReadThrottle(void *argument);
void SendSpeed(void *argument);
void startAccelUpdateTask(void *argument);

/* USER CODE BEGIN PFP */
void Lora_Init(void);
void Lora_Send_Data(char data[]);
void Parse_Recieve_Data(void);
void Smooth_Speed(int tempThrottle);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start the longboard by initializing the motor throttle to 0
  char StartMsg[100];
  sprintf(StartMsg, "\r\nStarting the LongBoard!\r\n");
  HAL_UART_Transmit(&huart2, StartMsg, strlen(StartMsg), 50);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  TIM3->CCR4 =  Min_PWM;
  HAL_Delay(5000);

  init_spatial(&hi2c1, &huart2);
  sprintf(StartMsg, "\r\nInitialized Accelerometer!\r\n");
  HAL_UART_Transmit(&huart2, StartMsg, strlen(StartMsg), 50);


  sprintf(StartMsg, "\r\nInitialized Throttle!\r\n");
  HAL_UART_Transmit(&huart2, StartMsg, strlen(StartMsg), 50);
  throttle = Min_Throttle;

  Lora_Init();
  sprintf(StartMsg, "\r\nInitialized LORA!\r\n");
  HAL_UART_Transmit(&huart2, StartMsg, strlen(StartMsg), 50);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of readThrottle */
  readThrottleHandle = osThreadNew(ReadThrottle, NULL, &readThrottle_attributes);

  /* creation of sendSpeed */
  sendSpeedHandle = osThreadNew(SendSpeed, NULL, &sendSpeed_attributes);

  /* creation of accelUpdateTask */
  accelUpdateTaskHandle = osThreadNew(startAccelUpdateTask, NULL, &accelUpdateTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  htim2.Init.Prescaler = 10000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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

void Lora_Init(void)
{
	// Set LORA Chip To Transmit/Receive Mode
	char msg[100] = "";
	// Sets Parameters to:
	// Spreading Factor: 7
	// Bandwidth: 500 KHz
	// Coding Rate: 1
	// Programmed Preamble: 10
	// This favors speed over dependabilitySerial2.println("AT+PARAMETER=7,9,1,4");
	sprintf(msg, "AT+PARAMETER=10,8,1,4\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(1000);

	// Sets LORA Chip address to 24
	sprintf(msg, "AT+ADDRESS=24\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(500);

	// Reads back address to verify setup
	sprintf(msg, "AT+NETWORKID=3\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(500);

	sprintf(msg, "AT+CPIN?\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(500);


	sprintf(msg, "AT+CRFOP?\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(500);

	sprintf(msg, "AT+ADDRESS?\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(500);

	sprintf(msg, "AT+NETWORKID?\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(500);

	sprintf(msg, "AT+BAND?\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(500);

	sprintf(msg, "AT+MODE=0\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(500);
}

void Lora_Send_Data(char data[])
{
	// Combines message to be sent with the data passed in
	char LoraMsg[100];
	sprintf(LoraMsg, "AT+SEND=%i,%i,%s\r\n", ESP_LORA_ADDRESS, strlen(data), data);
	HAL_UART_Transmit(&huart1, LoraMsg, strlen(LoraMsg), 50);
	osThreadSuspend(readThrottleHandle);
	osDelay(200);
	osThreadResume(readThrottleHandle);



}

void Parse_Recieve_Data(void)
{
	// Find the position of "T" in the array
		bool good = true;

	    char *start = strstr(UART1_rxBuffer, "T");

	    // Check if "T" is found
	    if (start != NULL)
	    {
	    	buffer_print(start, "str from start of parse");
	        // Find the position of the next comma after "S"
	        char *end = strchr(start, ',');

	        // Check if the comma is found
	        if (end != NULL)
	        {

	        	char *error = strchr(receive_data, '-');
	        	if (error != NULL)
	        	{
	        		good = false;
	        	}
	        	error = strchr(receive_data, '+');
	        	if (error != NULL)
	        	{
	        		good = false;
	        	}
	        	error = strchr(receive_data, '=');
	        	if (error != NULL)
	        	{
	        		good = false;
	        	}
	        	error = strchr(receive_data, ',');
	        	if (error != NULL)
	        	{
	        		good = false;
	        	}
	        	error = strchr(receive_data, ' ');
	        	if (error != NULL)
	        	{
	        		good = false;
	        	}

	        	if (good)
	        	{
	        		// Calculate the length of the substring
	        		size_t length = end - start;
	        		if (length > 4)
	        			return;

	        		// Copy the substring to the buffer
	        		strncpy(receive_data, start, length);

	        		// Null-terminate the buffer
	        		receive_data[length] = '\0';
	        	}

	        }
	    }
}

// Get stored time in timer 2 in terms of seconds
double get_timestep() {
	long double cur_time = TIM2->CNT;
	// Reset timer for next call
	TIM2->CNT = 0;
	// Division to make time in terms of seconds
	cur_time /= 8000;
	return (double)cur_time;
}

void buffer_print(void* buffer, const char* msg) {
	uint8_t modded_buffer[strlen(buffer) + strlen(msg)];
	sprintf(modded_buffer, "%s: %s\r\n", msg, buffer);
	HAL_UART_Transmit(&huart2, modded_buffer, strlen(modded_buffer), HAL_MAX_DELAY);
}

HAL_StatusTypeDef receive_lora_packet()
{
	bool received_new_packet = false;
	while (!received_new_packet) {
	    HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_rxBuffer, 1);
	    if (UART1_rxBuffer[0] == '+')
	    	break;
	}
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_rxBuffer, LORA_BUFFER_SIZE - 2);
//    buffer_print(UART1_rxBuffer, "rcv portion");
    // Packets MUST be from address 25 and have a length of 3 bytes or they will be discarded
    UART1_rxBuffer[LORA_BUFFER_SIZE - 1] = 0;
	buffer_print(UART1_rxBuffer, "rxBuffer");
//    if (strncmp(UART1_rxBuffer, "RCV=25,", 7)) {
//    	return HAL_ERROR;
//    }
    Parse_Recieve_Data();
    buffer_print(receive_data, "data");
//    while (UART1_rxBuffer[0] != '\n') {
//    	HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_rxBuffer, 1);
////        buffer_print(UART1_rxBuffer, "garbage char");
//    }
    UART1_rxBuffer[0] = 0;
	return HAL_OK;
}

void Smooth_Speed(int tempThrottle)
{
	// If the tempThrottle is above or below Max or Min set it to the Max or Min
	if (tempThrottle < Min_Throttle) {
		tempThrottle = Min_Throttle;
	}
	else if (tempThrottle > Max_Throttle) {
		tempThrottle = Max_Throttle;
	}

	// see if the difference is bigger than 1 then smooth the throttle increase or decrease
	int diffThrottle = tempThrottle - throttle;
	if (diffThrottle >= -1 && diffThrottle <= 1)
	{
		return;
	}

	if (diffThrottle > 1)
	{
		while (throttle < tempThrottle)
		{
			throttle += 1;
			TIM3->CCR4 =  Min_PWM + throttle;
			HAL_Delay(100);
		}
	}
	else if (diffThrottle < 1)
	{
		while (throttle > tempThrottle)
		{
			throttle -= 1;
			TIM3->CCR4 =  Min_PWM + throttle;
			HAL_Delay(100);
		}
	}
	return;

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ReadThrottle */
/**
* @brief Function implementing the readThrottle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadThrottle */
void ReadThrottle(void *argument)
{
  /* USER CODE BEGIN ReadThrottle */
  /* Infinite loop */
  char ThrottleMsg[50];
  for(;;) {
//	HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 25);
    
//	Parse_Recieve_Data();
//	HAL_UART_Transmit(&huart2, receive_data, strlen(receive_data), 25);
    receive_lora_packet();

	if (receive_data[0] == 'T') {
		int tempThrottle = atoi(receive_data + 1);
		Smooth_Speed(tempThrottle);
		sprintf(ThrottleMsg, "Set Throttle to: %i", throttle);
		Lora_Send_Data(ThrottleMsg);

	}
  }
  /* USER CODE END ReadThrottle */
}

/* USER CODE BEGIN Header_SendSpeed */
/**
* @brief Function implementing the sendSpeed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendSpeed */
void SendSpeed(void *argument)
{
  /* USER CODE BEGIN SendSpeed */
  /* Infinite loop */
  for(;;)
  {
	// Calculate Speed
	char formatted_speed[4] = "";
	sprintf(formatted_speed, "%.1f", current_speed);
	Lora_Send_Data(formatted_speed);
  osDelay(1000);
  }
  /* USER CODE END SendSpeed */
}

/* USER CODE BEGIN Header_startAccelUpdateTask */
/**
* @brief Function implementing the accelUpdateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startAccelUpdateTask */
void startAccelUpdateTask(void *argument)
{
  /* USER CODE BEGIN startAccelUpdateTask */
  /* Infinite loop */
  for(;;) {
    update_spatial(get_timestep());
  }
  /* USER CODE END startAccelUpdateTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
	  if (throttle > 0)
	  {
		  throttle--;
		  HAL_Delay(50);
	  }
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
