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
#include "stdbool.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FREQ 70000000 // Clock frequency is 70 MHz
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int64_t currentTime, t1, t2, t3, t4 = 0; // Timing variables
int i = 1; 								 // Number of seconds to wait before LED starts toggling
int64_t PTPs = 0; 						 // Number of times PTP algorithm has been completed
uint8_t buffer[8];						 // UART buffer
int ERROR_TX, ERROR_RX = 0;				 // Error indicators
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Function for transmitting a uint32_t message (msg) via UART
void uart_tx(int64_t msg)
{
	// Break up the 64-bit integer into 8-bit segments
	for (int i = 0; i < 8; i++)
	{
	  buffer[i] = (msg >> (i * 8)) & 0xFF;
	}

	// Send the data
	if (HAL_UART_Transmit(&huart1, buffer, 8, 100) != HAL_OK)
	{
	  // Transmission Error
	  ERROR_TX = 1;
	  Error_Handler();
	}
}

// Function for receiving a uint32_t message (msg) via UART
void uart_rx(int64_t *msg)
{
	// Clear UART buffer
	memset(buffer, 0, sizeof(buffer));

	// Receive the data
	if (HAL_UART_Receive(&huart1, buffer, 8, 100) != HAL_OK)
	{
	  // Reception Error
	  ERROR_RX = 1;
	  *msg = 0;
	  HAL_UART_DeInit(&huart1);
	  HAL_UART_Init(&huart1);
	  Error_Handler();
	  return;
	}

	// Reconstruct the 64-bit integer from the 8-bit segments
	*msg = 0;
	for (int i = 0; i < 8; i++)
	{
	  *msg |= ((int64_t)buffer[i]) << (i * 8);
	}
}

// Function for toggling logic level of a timer pin
void toggle_pulse(TIM_HandleTypeDef* htim, uint32_t channel){

    // Determine channel flag
	uint32_t channelFlag;
    switch(channel){
        case TIM_CHANNEL_1:
            channelFlag = TIM_FLAG_CC1;
            break;
        case TIM_CHANNEL_2:
            channelFlag = TIM_FLAG_CC2;
            break;
        case TIM_CHANNEL_3:
            channelFlag = TIM_FLAG_CC3;
            break;
        case TIM_CHANNEL_4:
            channelFlag = TIM_FLAG_CC4;
            break;
        default:
            return; // Invalid channel
    }

    // Set compare 250 clock cycles (~3.5us) ahead of current time
    currentTime = __HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COMPARE(htim, channel, (currentTime + 250) % __HAL_TIM_GET_AUTORELOAD(htim)); // Modulo accounts for overflow

    // Wait for OC event to trigger
    while(__HAL_TIM_GET_FLAG(htim, channelFlag) == RESET){
        // OC not yet triggered
    }

    // Clear OC flag in preparation for a future OC event
    __HAL_TIM_CLEAR_FLAG(htim, channelFlag);
}

// Master Sync Function
void sync_m(void)
{
	// Wait for the STATUS input pin to go high
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET){
		// Slave not ready for PTP
	}

	// Toggle master OC pin high
	toggle_pulse(&htim2, TIM_CHANNEL_4);

	// Record OC time as t1
	t1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);

	// Toggle master OC pin low
	toggle_pulse(&htim2, TIM_CHANNEL_4);
}

// Master Follow Up Function
void follow_up_m(void)
{
	// Wait for the STATUS input pin to go low
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET){
		// Slave not ready for UART
	}

	// Transmit t1 to slave
	uart_tx(t1);
}

// Master Delay Req Function
void delay_req_m(void)
{
	// Wait for IC event to trigger
	while(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC3) == RESET){
		// Waiting for slave to respond
	}

	// Clear IC flag in preparation for a future IC event
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC3);

	// Record IC time as t4
	t4 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
}

// Master Delay Res Function
void delay_res_m(void)
{
	// Wait for the STATUS input pin to go low
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET){
		// Slave not ready for UART
	}

	// Transmit t4 to slave
	uart_tx(t4);
}

// Slave Sync Function
void sync_s(void)
{
	// Set the STATUS output pin high (not ready to receive UART)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	// Wait for IC event to trigger
	while(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC3) == RESET){
		// Waiting for master to respond
	}

	// Clear IC flag in preparation for a future IC event
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC3);

	// Record IC time as t2
	t2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
}

// Slave Follow Up Function
void follow_up_s(void)
{
	HAL_Delay(1);
	// Set the STATUS output pin low (ready to receive UART)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	// Receive t1 from master
	uart_rx(&t1);

	// Remove time offset
	if(t1 != 0){
	__HAL_TIM_SET_COUNTER(&htim2, __HAL_TIM_GET_COUNTER(&htim2) + (t1 - t2));
	}

	// Set the STATUS output pin high (not ready to receive UART)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

// Slave Delay Req Function
void delay_req_s(void)
{
	// Toggle slave OC pin high
	toggle_pulse(&htim2, TIM_CHANNEL_2);

	// Record OC time as t3
	t3 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);

	// Toggle slave OC pin low
	toggle_pulse(&htim2, TIM_CHANNEL_2);
}

// Slave Delay Res Function
void delay_res_s(void)
{
	HAL_Delay(1);
	// Set the STATUS output pin low (ready to receive UART)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	// Receive t4 from master
	uart_rx(&t4);

	// Remove propagation delay
	if(t1 != 0 && t4 != 0){
	__HAL_TIM_SET_COUNTER(&htim2, __HAL_TIM_GET_COUNTER(&htim2) + round((t4 - t3)/2)); // Round() is required as t4-t3 could be an odd number
	}

	// Set the STATUS output pin low (ready to receive UART)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

// Function for configuring the output pin (timer 2, channel 1) for synchronisation performance testing
void update_output(void)
{
	// If an OC event has been triggered
	if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) == SET){

		i++;

		if(i > __HAL_TIM_GET_AUTORELOAD(&htim2) / FREQ){
			i = 1; // Reset i to 1 on timer overflow
		}

		// Clear OC flag in preparation for a future OC event
		__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);

		// Set compare 1 second ahead
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (i*FREQ) % __HAL_TIM_GET_AUTORELOAD(&htim2)); // Modulo accounts for timer overflow
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start timer 2
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1); // OC test channel
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2); // OC slave channel
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_3); // IC master channel
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_4); // OC master channel
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i * FREQ); // Set test OC to start after i seconds

  // Set STATUS output pin high to start PTP with master
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // PTP as slave
	  sync_s();
	  follow_up_s();
	  delay_req_s();
	  delay_res_s();

	  // PTP as master
	  sync_m();
	  follow_up_m();
	  delay_req_m();
	  delay_res_m();

	  // Configure test output
	  update_output();
	  PTPs++;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 4294967295;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 1750000;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_OUT_GPIO_Port, STATUS_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STATUS_OUT_Pin */
  GPIO_InitStruct.Pin = STATUS_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : STATUS_IN_Pin */
  GPIO_InitStruct.Pin = STATUS_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(STATUS_IN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
