/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DataStruct.h"
//#include "LCD_Driver.h"
//#include "PowerSupply.h"
#include "PowerSupplyInterface.h"
#include "PCInterface.h"
#include "GetDependenceValue.h"
#include "manager.h"
#include "Config.h"
#include "eeprom.h"

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
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;






osThreadId defaultTaskHandle;
osThreadId PowerSupplyTaskHandle;
osThreadId SensorsPollTaskHandle;
osThreadId LedTaskHandle;
osThreadId NextionTaskHandle;
osThreadId CompRS485TaskHandle;
osThreadId ManagerTaskHandle;
osThreadId PumpTaskHandle;
osThreadId CalcAhHandle;
/* USER CODE BEGIN PV */




 struct DataStruct Values;
 enum D_OUT_PINS
		{
			VALVE,
			PUMP
		};
	
		
 //extern int16_t WarningTimer ;
 uint8_t ADC_fl = 0; // 
// volatile uint16_t ADC_Data[6];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void const * argument);
void PowerSupplyFu(void const * argument);
void SensorsPollFu(void const * argument);
void LedFU(void const * argument);
void NextionFu(void const * argument);
void CompRS485Fu(void const * argument);
void ManagerFu(void const * argument);
void PumpFu(void const * argument);
void CalcAhFu(void const * argument);

/* USER CODE BEGIN PFP */
void setPWM(uint16_t pwm_value, enum D_OUT_PINS D_out_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/*void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    {
        while (true)
        {
            portNOP();
        }
    }
}*/

void vApplicationMallocFailedHook()
{
    {
        while (true)
        {
            portNOP();
        }
    }
}


void setPWM(uint16_t value, enum D_OUT_PINS D_out_Pin)
{
	//return;
    TIM_OC_InitTypeDef sConfigOC;
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = value;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

		switch (D_out_Pin)
		{
			case PUMP:
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
				HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
				break;
			case VALVE:
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
				HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2); 
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
				break;
		}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) //check if the interrupt comes from ACD1
  {
		HAL_ADC_Stop_DMA(hadc);
		HAL_GPIO_WritePin(WQ_Active_GPIO_Port, WQ_Active_Pin, GPIO_PIN_RESET);
    ADC_fl=true;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	InitUarts();
	InitPowerSupplyInterface();
  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of PowerSupplyTask */
  osThreadDef(PowerSupplyTask, PowerSupplyFu, osPriorityHigh, 0, 1800);
  PowerSupplyTaskHandle = osThreadCreate(osThread(PowerSupplyTask), NULL);

  /* definition and creation of SensorsPollTask */
  osThreadDef(SensorsPollTask, SensorsPollFu, osPriorityLow, 0, 128);
  SensorsPollTaskHandle = osThreadCreate(osThread(SensorsPollTask), NULL);

  /* definition and creation of LedTask */
  osThreadDef(LedTask, LedFU, osPriorityLow, 0, 128);
  LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);

  /* definition and creation of NextionTask */
  osThreadDef(NextionTask, NextionFu, osPriorityLow, 0, 1200);
  NextionTaskHandle = osThreadCreate(osThread(NextionTask), NULL);

  /* definition and creation of CompRS485Task */
  osThreadDef(CompRS485Task, CompRS485Fu, osPriorityLow, 0, 1800);
  CompRS485TaskHandle = osThreadCreate(osThread(CompRS485Task), NULL);

  /* definition and creation of ManagerTask */
  osThreadDef(ManagerTask, ManagerFu, osPriorityHigh, 0, 1200);
  ManagerTaskHandle = osThreadCreate(osThread(ManagerTask), NULL);

  /* definition and creation of PumpTask */
  osThreadDef(PumpTask, PumpFu, osPriorityLow, 0, 128);
  PumpTaskHandle = osThreadCreate(osThread(PumpTask), NULL);

  /* definition and creation of CalcAh */
  osThreadDef(CalcAh, CalcAhFu, osPriorityNormal, 0, 128);
  CalcAhHandle = osThreadCreate(osThread(CalcAh), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
//	vTaskSuspend(LCD_TaskHandle);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
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
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, WQ_Active_Pin|DI1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|DI2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : WQ_Active_Pin DI1_Pin */
  GPIO_InitStruct.Pin = WQ_Active_Pin|DI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D_IN_1_Pin D_IN_2_Pin */
  GPIO_InitStruct.Pin = D_IN_1_Pin|D_IN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D_IN_3_Pin D_IN_6_Pin */
  GPIO_InitStruct.Pin = D_IN_3_Pin|D_IN_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D_IN_4_Pin PB2 FaultPin1_Pin FaultPin2_Pin
                           BTN2_Pin */
  GPIO_InitStruct.Pin = D_IN_4_Pin|GPIO_PIN_2|FaultPin1_Pin|FaultPin2_Pin
                          |BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin DI2_Pin */
  GPIO_InitStruct.Pin = LED_Pin|DI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void GeneratorSetActive(bool active)
{
    if (!active)
    {
        if (Values.OnOffButton == BTN_ON)
        {
            EEPROM_Write(PARAM_1, Values.AhCounter);
            EEPROM_Write(PARAM_2, Values.sysCounter);
            Values.OnOffButton = BTN_TRY_TO_STOP;
            Values.NextionSleepMode = NEXTION_TRY_TO_OFF;
        }
    }
		else
		{
			  if (Values.OnOffButton == BTN_OFF) 
				{
          Values.OnOffButton = BTN_TRY_TO_START;
          Values.NextionSleepMode = NEXTION_TRY_TO_ON;
        }

		}
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */

/* USER CODE END Header_StartDefaultTask */




void StartDefaultTask(void const *argument) {
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

  for (;;) {
    if (HAL_GPIO_ReadPin(D_IN_6_GPIO_Port, D_IN_6_Pin)) {
      vTaskDelay(100);
      if (HAL_GPIO_ReadPin(D_IN_6_GPIO_Port, D_IN_6_Pin)) {
        if (Values.OnOffButton == BTN_ON) {
          EEPROM_Write(PARAM_1, Values.AhCounter);
          EEPROM_Write(PARAM_2, Values.sysCounter);
          Values.OnOffButton = BTN_TRY_TO_STOP;
          Values.NextionSleepMode = NEXTION_TRY_TO_OFF;
        }
        if (Values.OnOffButton == BTN_OFF) {
          Values.OnOffButton = BTN_TRY_TO_START;
          Values.NextionSleepMode = NEXTION_TRY_TO_ON;
        }
      }
      vTaskDelay(1000);
      if (HAL_GPIO_ReadPin(D_IN_6_GPIO_Port, D_IN_6_Pin)) {
        // long click
      }
    }
    vTaskDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_PowerSupplyFu */
/**
* @brief Function implementing the PowerSupplyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PowerSupplyFu */
void PowerSupplyFu(void const * argument)
{
  /* USER CODE BEGIN PowerSupplyFu */
  /* Infinite loop */
	
  for(;;)
  {
		PollPowerSupply(&Values);
    vTaskDelay(100);
  }
  /* USER CODE END PowerSupplyFu */
}

/* USER CODE BEGIN Header_SensorsPollFu */
/**
* @brief Function implementing the SensorsPollTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorsPollFu */
void SensorsPollFu(void const* argument) {
  /* USER CODE BEGIN SensorsPollFu */
  /* Infinite loop */

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&Values.A_IN_1_RAW, 4);
  /* Infinite loop */
  for (;;) {
    if (ADC_fl) {
      Values.Press = (((float)(Values.A_IN_2_RAW)) - 143) / 658;
      Values.Temperature = GetDependenceValue(
          Values.A_IN_4_RAW, TempDependence,
          sizeof(TempDependence) / sizeof(TempDependence[0]));
      Values.WQ =
          (GetDependenceValue(Values.A_IN_3_RAW, WQ_20,
                              sizeof(WQ_20) / sizeof(WQ_20[0]))) *
          GetDependenceValueF(Values.Temperature, TempCorrectionDependence,
                              sizeof(TempCorrectionDependence) /
                                  sizeof(TempCorrectionDependence[0]));

      ADC_fl = 0;
      HAL_GPIO_WritePin(WQ_Active_GPIO_Port, WQ_Active_Pin, GPIO_PIN_SET);
      vTaskDelay(50);
      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&Values.A_IN_1_RAW, 4);
      Values.D_IN_1 = HAL_GPIO_ReadPin(D_IN_1_GPIO_Port, D_IN_1_Pin);
      Values.D_IN_2 = HAL_GPIO_ReadPin(D_IN_2_GPIO_Port, D_IN_2_Pin);
      Values.D_IN_3 = HAL_GPIO_ReadPin(D_IN_3_GPIO_Port, D_IN_3_Pin);
      Values.D_IN_4 = HAL_GPIO_ReadPin(D_IN_4_GPIO_Port, D_IN_4_Pin);
      // Values.D_IN_5 = HAL_GPIO_ReadPin(D_IN_5_GPIO_Port, D_IN_3_Pin);
      // Values.D_IN_6 = HAL_GPIO_ReadPin(D_IN_6_GPIO_Port, D_IN_6_Pin);
      vTaskDelay(100);
    }
  }
  /* USER CODE END SensorsPollFu */
}

/* USER CODE BEGIN Header_LedFU */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedFU */
void LedFU(void const * argument)
{
  /* USER CODE BEGIN LedFU */
  /* Infinite loop */
    for( ;; )
    {
			switch (Values.LedMode)
			{
				case LED_NORMAL_BLINK:
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				for (int i = 0; i < 8; i++)
				if (Values.LedMode != LED_NORMAL_BLINK) break; else vTaskDelay(50);
				break;
				case LED_FAST_BLINK:
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,GPIO_PIN_SET);	
				for (int i = 0; i < 10; i++)
				{
					vTaskDelay(50);
					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				}
				Values.LedMode = LED_NORMAL_BLINK;
				break;
				case LED_VERY_FAST_BLINK:
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,GPIO_PIN_SET);	
				for (int i = 0; i < 12; i++)
				{
					vTaskDelay(100);
					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				}
				Values.LedMode = LED_NORMAL_BLINK;
				break;
			}
				
     }	
  /* USER CODE END LedFU */
}

/* USER CODE BEGIN Header_NextionFu */
/**
* @brief Function implementing the NextionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NextionFu */
void NextionFu(void const * argument)
{
  /* USER CODE BEGIN NextionFu */
  /* Infinite loop */
	vTaskDelay(800);
	char StrNextion[80]={0xFF};
  sprintf(StrNextion, "MaxSetpoint=%d", MAX_FLOW*10);
  //sprintf(StrNextion, "page 0");
	SendToNextion(StrNextion);
	vTaskDelay(50);	
  sprintf(StrNextion, "page 0");
	SendToNextion(StrNextion);
	vTaskDelay(50);	
  sprintf(StrNextion, "sleep=1");
  //sprintf(StrNextion, "page 0");
	SendToNextion(StrNextion);
	vTaskDelay(50);

		
  for(;;)
  {
		NextionPoll(&Values);
    vTaskDelay(1000);
  }
	while(1){};
  /* USER CODE END NextionFu */
}

/* USER CODE BEGIN Header_CompRS485Fu */
/**
* @brief Function implementing the CompRS485Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CompRS485Fu */
void CompRS485Fu(void const * argument)
{
  /* USER CODE BEGIN CompRS485Fu */
  /* Infinite loop */
  for(;;)
  {
			if (PC_BuffLen)
				{
					GrabMSG((uint8_t*)PC_Buff, PC_BuffLen, &Values);
					PC_BuffLen = 0;
				}

		//SendToPC(&Values);
    //vTaskDelay(1000);
  }
  /* USER CODE END CompRS485Fu */
}

/* USER CODE BEGIN Header_ManagerFu */
/**
* @brief Function implementing the ManagerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ManagerFu */
void ManagerFu(void const * argument)
{
  /* USER CODE BEGIN ManagerFu */
  /* Infinite loop */
	vTaskDelay(1000);
  for(;;)
  {
		ManagerPoll(&Values);
		vTaskDelay(100);
  }
  /* USER CODE END ManagerFu */
}

/* USER CODE BEGIN Header_PumpFu */
/**
* @brief Function implementing the PumpTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PumpFu */
void PumpFu(void const * argument)
{
  /* USER CODE BEGIN PumpFu */
	
//	/*
	setPWM(250, PUMP);
	vTaskDelay(4000);
	setPWM(400, PUMP);
	vTaskDelay(4000);
 //*/
	
/*	for(;;)
	{
				setPWM(250, VALVE);
				vTaskDelay(TIME_TO_POUR);
				setPWM(0, VALVE);
				vTaskDelay(TIME_TO_POUR);
	}
	*/
	
  /* Infinite loop */
  for(;;)
  {
		//continue;
/*		if (Values.D_IN_1 == 1) //Hydrogen water level is high
			if ((Values.Press > MIN_PRESS_TO_POUR) && (Values.Mode == MODE_NORMAL_WORK))
			{
				setPWM(250, VALVE);
				vTaskDelay(TIME_TO_POUR);
				setPWM(0, VALVE);
				
			} */
		if (Values.Pump == PUMP_ON) 
			{
				//Config Pump
				
				
				//setPWM(200, PUMP);
				//vTaskDelay(10);
				setPWM(PUMP_PWM, PUMP);
				vTaskDelay(TIME_PUMP_ON);
				//setPWM(0, PUMP);
				//vTaskDelay(TIME_PUMP_OFF);
			}
			else	
			{
				setPWM(0, PUMP); 
				//HAL_GPIO_TogglePin(D_OUT_1_GPIO_Port, D_OUT_1_Pin);
				vTaskDelay(1000); 
			}
  }
  /* USER CODE END PumpFu */
}

/* USER CODE BEGIN Header_CalcAhFu */
/**
* @brief Function implementing the CalcAh thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CalcAhFu */
void CalcAhFu(void const * argument)
{
  /* USER CODE BEGIN CalcAhFu */
	EEPROM_Init();
	Values.AhCounter = 0;
	Values.sysCounter = 0;
	
  
  
//  EEPROM_Write(PARAM_1, 0x0);
//  EEPROM_Write(PARAM_2, 0x0);

  EEPROM_Read(PARAM_1, &Values.AhCounter);
  EEPROM_Read(PARAM_2, &Values.sysCounter);
  
  //EEPROM_Write(PARAM_2, 0x3333);
	
	
  /* Infinite loop */
  for(;;)
  {
		for (uint16_t index = 0; index < 3600 ;index++)
		{
			if ((Values.Mode == 		MODE_LEAK_CHECK1) || (Values.Mode == MODE_LEAK_CHECK2) || (Values.Mode == MODE_NORMAL_WORK))
			{
				Values.AhCounter += Values.Amperage/10;
				Values.sysCounter++;
			}
			//if (WarningTimer > 0) { WarningTimer--; }
			osDelay(1000);
		}
		//сохраняем во флеш
		EEPROM_Write(PARAM_1, Values.AhCounter);
		EEPROM_Write(PARAM_2, Values.sysCounter);
			
  }
  /* USER CODE END CalcAhFu */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
