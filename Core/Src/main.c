/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    MODE_IDLE,
    MODE_STEP_LAUNCH,
    MODE_WAVE_LAUNCH,
	MODE_STEP_RUNNING,
	MODE_FREQ_RUNNING
} SystemState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEP_SAMPLES 2000
#define LUT_SAMPLES 1000
// Received command buffer size
#define RX_BUFFER_SIZE 32
// Binary protocol mode headers
#define BIN_HEADER_1 0xAA
#define BIN_HEADER_2 0xBB
#define BIN_MODE_STEP 0x01
#define BIN_MODE_FREQ 0x02

// Board timers frequency
#define MCU_TIMER_CLOCK_FREQ 84000000

// Transmission variables (ping-pong buffer)
#define CHUNK_SIZE (2*LUT_SAMPLES)
#define TOTAL_BUFFER_SIZE (2*CHUNK_SIZE)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
volatile uint16_t ADC_buffer[TOTAL_BUFFER_SIZE];
uint16_t DAC_Sine[LUT_SAMPLES];
uint16_t DAC_Saw[LUT_SAMPLES];
uint16_t* current_waveform_ptr = DAC_Sine;

// System state initialization
SystemState_t current_state = MODE_IDLE;

// UART command reception variables
uint8_t rx_byte;              	// current byte
char rx_buffer[RX_BUFFER_SIZE]; // complete command buffer
uint8_t rx_index = 0;			// current byte index

// UART data transmission header
const uint8_t header_seq[2] = {BIN_HEADER_1, BIN_HEADER_2};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_DAC_Init();
  MX_ADC2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  // Create one sine period LookUpTable
  for (int i = 0; i < LUT_SAMPLES; i++){
	  DAC_Sine[i] = (uint16_t)((4095.0f/2.0f) * (1.0f + sin(2.0f*M_PI*i/LUT_SAMPLES)));
  }

  // Create Sawtooth LookUpTable
  for(int i = 0; i < LUT_SAMPLES; i++){
	  DAC_Saw[i] = (uint16_t)(4095.0f/(LUT_SAMPLES-1)*i);
  }

  // Trigger connection with the user
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Start step response routine
	  if (current_state == MODE_STEP_LAUNCH)stepResponse();

	  // Start frequency analysis routine
	  if (current_state == MODE_WAVE_LAUNCH)waveformResponse();

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  sConfig.DAC_Trigger = DAC_TRIGGER_T8_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 8399;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin STEP_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * The ADC conversion complete callback is called when the whole ADC buffer has
 * been filled with conversions.
 * For the step response, which is a one shot event, the code in this callback
 * reasonably stops the conversion and sends the whole buffer to the PC.
 * For the frequency response, this callback handles the 'PONG' part of the buffer
 * transmission, sending the second half of the ADC conversion buffer, without
 * interrupting the conversion, as we want to continuously measure voltage.
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (current_state == MODE_IDLE) return;

    // Handles STEP, measured on ADC1
    if (hadc->Instance == ADC1)
    {
        // Stop conversion (One-Shot)
        HAL_TIM_Base_Stop(&htim2);
        HAL_ADC_Stop_DMA(&hadc1);

        // Send the measurements
        sendData(&ADC_buffer[0], STEP_SAMPLES, BIN_MODE_STEP);

        // Reset system state
        current_state = MODE_IDLE;
    }

    // Handles frequency response, measured on ADC2
    if (hadc->Instance == ADC2)
    {
        // Send only the second part of the buffer, without stopping the conversion
        sendData(&ADC_buffer[TOTAL_BUFFER_SIZE/2], TOTAL_BUFFER_SIZE/2, BIN_MODE_FREQ);
    }
}

/*
 * This callback handles the 'PING' part of the buffer transmission, sending the
 * first half of the ADC conversion buffer, without  interrupting the conversion,
 * as we want to continuously measure voltage.
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC2)
    {
    	// Send only the first part of the buffer, without stopping the conversion
        sendData(&ADC_buffer[0], TOTAL_BUFFER_SIZE/2, BIN_MODE_FREQ);
    }
}

/*
 * This callback handles the command reception. Every received byte is stored in an array,
 * when a line terminator is received, the whole command, stored in the array is parsed and
 * the system state is updated accordingly
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2){

		// Check if a line terminator has been sent, indicating a complete command
		if (rx_byte == '\n' || rx_byte == '\r'){
			// Close the string with the null terminator
			rx_buffer[rx_index] = '\0';

			if (rx_buffer[0] == 'S'){
				current_state = MODE_STEP_LAUNCH;
			}

			if (rx_buffer[0] == 'F'){
				int freq_val = atoi(&rx_buffer[1]);
				changeFreq(freq_val);
				current_waveform_ptr = DAC_Sine;
				current_state = MODE_WAVE_LAUNCH;
			}
			if(rx_buffer[0] == 'T'){
				int freq_val = atoi(&rx_buffer[1]);
				changeFreq(freq_val);
				current_waveform_ptr = DAC_Saw;
				current_state = MODE_WAVE_LAUNCH;
			}

			if (rx_buffer[0] == 'A'){
				// Reset system state
				current_state = MODE_IDLE;

				// Stop everything
				HAL_TIM_Base_Stop(&htim2);
				HAL_TIM_Base_Stop(&htim8);
				HAL_ADC_Stop_DMA(&hadc1);
				setPinMode(STEP_GPIO_Port, STEP_Pin, GPIO_MODE_INPUT);
				HAL_ADC_Stop_DMA(&hadc2);
				HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
				setPinMode(DAC_GPIO_Port, DAC_Pin, GPIO_MODE_INPUT);

			}
			rx_index = 0;
		}
		else{
			// Buffer overflow protection
			if (rx_index < RX_BUFFER_SIZE){
				rx_buffer[rx_index] = rx_byte;
				rx_index++;
			}
		}
		HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
	}
}

void stepResponse(void)
{
	// Update state
	current_state = MODE_STEP_RUNNING;

	// Configure the DAC pin to analog input to avoid short circuits
	setPinMode(DAC_GPIO_Port, DAC_Pin, GPIO_MODE_INPUT);

	// Configure the step pin to digital output in order to produce the step signal
	setPinMode(STEP_GPIO_Port, STEP_Pin, GPIO_MODE_OUTPUT_PP);

	// Wait to ensure the capacitor is fully discharged
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_Delay(500);

	// Start DMA ADC conversion, still waiting for the timer trigger
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_buffer, STEP_SAMPLES);

	// Start the timer that triggers the ADC conversion
	HAL_TIM_Base_Start(&htim2);

	// Send a voltage step to the circuit
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void waveformResponse(void)
{
	// Update state
	current_state = MODE_FREQ_RUNNING;

	// Configure the step pin to analog input to avoid short circuits
	setPinMode(STEP_GPIO_Port, STEP_Pin, GPIO_MODE_INPUT);

	// Configure the DAC pin to analog output in order to produce the sine signal
	setPinMode(DAC_GPIO_Port, DAC_Pin, GPIO_MODE_ANALOG);

	// Start DMA DAC, still waiting for the timer trigger
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) current_waveform_ptr, LUT_SAMPLES, DAC_ALIGN_12B_R);

	// Start DMA ADC conversion, still waiting for the timer trigger
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC_buffer, TOTAL_BUFFER_SIZE);

	// Start the timer that triggers the ADC and DAC
	HAL_TIM_Base_Start(&htim8);
}

void sendData(volatile uint16_t* buffer, uint16_t buffer_length, uint8_t mode)
{
    // Check if the UART is idle, otherwise skip
    if (huart2.gState != HAL_UART_STATE_READY) return;

    // Send chunk header + mode header
    HAL_UART_Transmit(&huart2, (uint8_t*)header_seq, 2, 1);
    HAL_UART_Transmit(&huart2, &mode, 1, 1);

    // Send ADC data through DMA
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, buffer_length * 2);
}

/*
 *  Wrapper function used to switch pins configuration mode
 */
void setPinMode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t Mode)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = Mode;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/*
 * Wrapper function used to change waveforms
 */
void changeFreq(uint16_t sine_freq)
{
	// Avoid too high, null or negative frequencies
	if (sine_freq > 2 && sine_freq <= 200){

		uint16_t new_ARR = (uint16_t) ((MCU_TIMER_CLOCK_FREQ / ((uint32_t)sine_freq*LUT_SAMPLES)) - 1);
		__HAL_TIM_SET_AUTORELOAD(&htim8, new_ARR);
		htim8.Instance->EGR = TIM_EGR_UG;
	}

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
