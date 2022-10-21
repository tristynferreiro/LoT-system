/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REF 0.5				// Reference used for determining value of incoming bits
#define PERIOD 1000			// Time between samples
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t listening = 0;		// Variable that is changed when blue push button is pressed
uint8_t data[8];			// Array to store data values as they are received
uint8_t samples = 0;		// Counter to keep track of number of data samples received
uint8_t allSamples[20];		// Array to store data samples once they have been converted from an array to a single byte value

char VAL_print[40];			// Array to use for printing to UART
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
uint8_t arrayToData(void);
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
  MX_ADC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	  // Wait until push button is pressed to start listening
	  	  if (listening == 1)
	  	  {
	  		  //print to UART
	  		  memset(VAL_print, 0, sizeof(VAL_print));
	  		  sprintf(VAL_print, "Waiting...\r\n\r\n");
	  		  HAL_UART_Transmit(&huart2, VAL_print, sizeof(VAL_print), 1000);

	  		  // Check the value of the signal input pin
	  		  uint32_t state = HAL_GPIO_ReadPin(GPIOA, Signal_in_Pin);

	  		  // Wait for start bit from transmitter
	  		  while(state<REF)
	  		  {
	  			  // Continue checking state if input pin until input goes HIGH
	  			  state = HAL_GPIO_ReadPin(GPIOA, Signal_in_Pin);
	  		  }

	  		  // Once state of input pin goes high, begin to read and store the data being received
	  		  int cont = 1;
	  		  //while loop to check if more data is transmitted
	  		  while(cont)
	  		  {
	  			  readSignal();
	  			  HAL_Delay(PERIOD);
	  			  cont = HAL_GPIO_ReadPin(GPIOA, Signal_in_Pin);
	  		  }
	  	  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
  ADC1->CR|=ADC_CR_ADEN;

  // Wait for ISR to be set
  while((ADC1->ISR & ADC_ISR_ADRDY)==0);
  /* USER CODE END ADC_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Signal_in_Pin */
  GPIO_InitStruct.Pin = Signal_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Signal_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
	/* Interrupt that is called when the blue push button is pressed */

	//if the button is pressed set the mode to listening
	if (B1_Pin)
		{
			if (listening == 0)
			{
				// Change listening to 1 so that STM starts waiting for start bit
				listening = 1;
			}
		}

	HAL_GPIO_EXTI_IRQHandler(B1_Pin);
}

uint8_t arrayToData(void)
{
	/* Convert the data array into a single byte value */

	// Local variable to store data value
	uint8_t output = 0;

	for(int i = 0; i < 8; i++)
	{
		output = output + ((data[i])<<i);
	}

	//reset data to 0
	memset(data, 0, sizeof data);
	return output;
}

void readSignal(void)
{
	/* Function called to read the input signal and determine if the input is a data packet or the number of samples already sent */

	//add an initial delay
	HAL_Delay(PERIOD);

	// Local variable to store the state of the input signal (0 means data packet being sent, 1 means number of samples being sent
	uint32_t state = HAL_GPIO_ReadPin(GPIOA, Signal_in_Pin);

	//check mode of operation
	if (state < REF) //save data mode
	{
		// Let computer know we are receiving data
		memset(VAL_print, 0, sizeof(VAL_print))
		sprintf(VAL_print, "Receiving data values...\r\n");
		HAL_UART_Transmit(&huart2, VAL_print, sizeof(VAL_print), 1000);

		// Once mode bit has been received, store the next 8 bits of data in the data array
		for (int i = 7; i >= 0; i--)
		{
			// Add delay between samples
			HAL_Delay(PERIOD);

			// Read pin value
			state = HAL_GPIO_ReadPin(GPIOA, Signal_in_Pin);
			if (state < REF)
			{
				data[i] = 0;
			}
			else
			{
				data[i] = 1;
			}
		}

		// Convert data into single 8 bit int and store in allSamples array
		allSamples[samples] = arrayToData();

		// Increase packet counter
		samples++;

		// Stop listening
		listening = 0;

		// Transmit the value of data receiver
		memset(VAL_print, 0, sizeof(VAL_print));
		sprintf(VAL_print, "Value Received: %d\r\n\r\n", allSamples[samples-1]);
		HAL_UART_Transmit(&huart2, VAL_print, sizeof(VAL_print), 1000);
	}
	else //compare no samples
	{
		memset(VAL_print, 0, sizeof(VAL_print));
		sprintf(VAL_print, "Comparing number of transmissions...\r\n");
		HAL_UART_Transmit(&huart2, VAL_print, sizeof(VAL_print), 1000);
		for(int i = 7; i >= 0; i--)
		{
			// Add delay between samples
			HAL_Delay(PERIOD);

			// Read pin value
			state = HAL_GPIO_ReadPin(GPIOA, Signal_in_Pin);

			if (state < REF)
			{
				data[i] = 0;
			}
			else
			{
				data[i] = 1;
			}
		}

		// Transmit number of samples received
		uint8_t transmit_samples = arrayToData();
		memset(VAL_print, 0, sizeof(VAL_print));
		sprintf(VAL_print, "Number of transmissions: %d\r\n",transmit_samples);
		HAL_UART_Transmit(&huart2, VAL_print, sizeof(VAL_print), 1000);


		// Check if number of samples received equals number of samples sent by transmitter
		if (transmit_samples == samples)
		{
			// Let computer know the values are equal
			memset(VAL_print, 0, sizeof(VAL_print));
			sprintf(VAL_print, "It's the same!! :)\r\n\r\n");
			HAL_UART_Transmit(&huart2, VAL_print, sizeof(VAL_print), 1000);

			// Toggle green pin on and off if values equal
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
			HAL_Delay(500);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
		}
		else
		{
			// Let computer know the values are not equal
			memset(VAL_print, 0, sizeof(VAL_print));
			sprintf(VAL_print, "It's not the same!! :(\r\n\r\n");
			HAL_UART_Transmit(&huart2, VAL_print, sizeof(VAL_print), 1000);

			// Toggle blue pin on and off if values equal
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			HAL_Delay(500);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

			// Change number of samples to number of samples sent by transmitted
			samples = transmit_samples;
		}
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
