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
#include "btt6200_4esa.h"
#include "btt6200_config.h"
#include "tmp1075.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t i_mA;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static TMP1075_HandleTypeDef htmp1075;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
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
  char uart_buf[100];

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI4_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // Debug: Preveri I2C2 devices
  uint8_t msg[] = "\r\n=== BMU IOC Initialization ===\r\n";
  HAL_UART_Transmit(&huart1, msg, sizeof(msg)-1, 100);

  // I2C scan za TMP1075
  if (HAL_I2C_IsDeviceReady(&hi2c2, 0x48 << 1, 3, 100) == HAL_OK) {
      uint8_t found[] = "TMP1075 detected at 0x48\r\n";
      HAL_UART_Transmit(&huart1, found, sizeof(found)-1, 100);
  } else {
      uint8_t notfound[] = "TMP1075 NOT detected at 0x48!\r\n";
      HAL_UART_Transmit(&huart1, notfound, sizeof(notfound)-1, 100);
  }

  // Inicializacija TMP1075
  HAL_StatusTypeDef tmp_status = TMP1075_Init(&htmp1075, &hi2c2, 0x48);
  if (tmp_status == HAL_OK) {
      uint8_t init_ok[] = "TMP1075 initialized OK\r\n";
      HAL_UART_Transmit(&huart1, init_ok, sizeof(init_ok)-1, 100);

      // Nastavi low temperature threshold na 0°C (alert pri < 0°C)
      if (TMP1075_SetLowThreshold(&htmp1075, 0.0f) == HAL_OK) {
          uint8_t thresh_ok[] = "Low threshold set to 0.0C\r\n";
          HAL_UART_Transmit(&huart1, thresh_ok, sizeof(thresh_ok)-1, 100);
      }
  } else {
      uint8_t init_fail[] = "TMP1075 initialization FAILED!\r\n";
      HAL_UART_Transmit(&huart1, init_fail, sizeof(init_fail)-1, 100);
  }

  /* inicializacija BTT6200 modulov */
  BTT6200_Config_Init(&hadc1);   // ali &hadc, isto kot si nastavil v config.c
  BTT6200_Init(&btt6200_modules[0]);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(PWR_24V_EN_GPIO_Port, PWR_24V_EN_Pin, SET);
	  HAL_GPIO_WritePin(PWR_SLEEP_GPIO_Port, PWR_SLEEP_Pin, SET);
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // LED toggle za heartbeat

	  // Preberi in izpiši temperaturo
	  float tC = 0.0f;
	  if (TMP1075_ReadTemperature(&htmp1075, &tC) == HAL_OK) {
	      // Uspešno branje temperature
	      int temp_int = (int)tC;
	      int temp_frac = (int)((tC - temp_int) * 100);
	      if (temp_frac < 0) temp_frac = -temp_frac;  // Absolutna vrednost za decimale

	      // ALERT: Temperatura pod 0°C
	      if (tC < 0.0f) {
	          sprintf(uart_buf, "*** ALERT! Temperature: %d.%02d C (BELOW 0C!) ***\r\n", temp_int, temp_frac);
	          HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
	          // Prižgi LED - hitro utripanje za alarm
	          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
	          HAL_Delay(100);
	          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
	          HAL_Delay(100);
	          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
	      } else {
	          sprintf(uart_buf, "Temperature: %d.%02d C\r\n", temp_int, temp_frac);
	          HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
	      }
	  } else {
	      uint8_t err[] = "Temperature read FAILED!\r\n";
	      HAL_UART_Transmit(&huart1, err, sizeof(err)-1, 100);
	  }

	  HAL_Delay(1000);

	  BTT6200_SetChannel(&btt6200_modules[0], BTT6200_CH0, true);
	  BTT6200_SetChannel(&btt6200_modules[0], BTT6200_CH1, true);
	  BTT6200_SetChannel(&btt6200_modules[0], BTT6200_CH2, true);
	  BTT6200_SetChannel(&btt6200_modules[0], BTT6200_CH3, true);
	  HAL_Delay(1000);
	  BTT6200_SetChannel(&btt6200_modules[1], BTT6200_CH0, true);
	  BTT6200_SetChannel(&btt6200_modules[1], BTT6200_CH1, true);
	  BTT6200_SetChannel(&btt6200_modules[1], BTT6200_CH2, true);
	  BTT6200_SetChannel(&btt6200_modules[1], BTT6200_CH3, true);
	  HAL_Delay(1000);
	  BTT6200_SetChannel(&btt6200_modules[2], BTT6200_CH0, true);
	  BTT6200_SetChannel(&btt6200_modules[2], BTT6200_CH1, true);
	  BTT6200_SetChannel(&btt6200_modules[2], BTT6200_CH2, true);
	  BTT6200_SetChannel(&btt6200_modules[2], BTT6200_CH3, true);
	  HAL_Delay(1000);
	  BTT6200_SetChannel(&btt6200_modules[3], BTT6200_CH0, true);
	  BTT6200_SetChannel(&btt6200_modules[3], BTT6200_CH1, true);
	  BTT6200_SetChannel(&btt6200_modules[3], BTT6200_CH2, true);
	  BTT6200_SetChannel(&btt6200_modules[3], BTT6200_CH3, true);
	  HAL_Delay(1000);
	  BTT6200_SetChannel(&btt6200_modules[4], BTT6200_CH0, true);
	  BTT6200_SetChannel(&btt6200_modules[4], BTT6200_CH1, true);
	  BTT6200_SetChannel(&btt6200_modules[4], BTT6200_CH2, true);
	  BTT6200_SetChannel(&btt6200_modules[4], BTT6200_CH3, true);
	  if (BTT6200_ReadChannelCurrent(&btt6200_modules[1], BTT6200_CH1, &i_mA) == HAL_OK) {
	      // i_mA = tok v mA (po tvoji formuli v driverju)
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, PWR_24V_EN_Pin|PWR_SLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, EN_5V_Pin|EN_3V3A_Pin|DSEL1_0_Pin|OUT3_0_Pin
                          |OUT2_0_Pin|DSEL0_0_Pin|DEN_0_Pin|OUT1_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT0_0_Pin|DSEL1_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OUT3_1_Pin|OUT2_1_Pin|DSEL0_1_Pin|DEN_1_Pin
                          |OUT1_1_Pin|OUT0_1_Pin|DSEL1_2_Pin|OUT3_2_Pin
                          |OUT2_4_Pin|OUT3_4_Pin|DSEL1_4_Pin|ISOSPI_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, OUT2_2_Pin|DSEL0_2_Pin|DEN_2_Pin|OUT1_2_Pin
                          |OUT0_2_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DEN_3_Pin|DSEL0_3_Pin|DSEL1_3_Pin|OUT1_4_Pin
                          |DEN_4_Pin|DSEL0_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT0_3_Pin|OUT1_3_Pin|OUT2_3_Pin|OUT3_3_Pin
                          |OUT0_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEM_OC_6_Pin IN_11_Pin IN_10_Pin */
  GPIO_InitStruct.Pin = LEM_OC_6_Pin|IN_11_Pin|IN_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LEM_OC_5_Pin LEM_OC_4_Pin */
  GPIO_InitStruct.Pin = LEM_OC_5_Pin|LEM_OC_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_9_Pin IN_8_Pin IN_7_Pin IN_6_Pin
                           IN_5_Pin IN_4_Pin IN_3_Pin IN_2_Pin
                           IN_1_Pin PG_5V_Pin LEM_OC_8_Pin */
  GPIO_InitStruct.Pin = IN_9_Pin|IN_8_Pin|IN_7_Pin|IN_6_Pin
                          |IN_5_Pin|IN_4_Pin|IN_3_Pin|IN_2_Pin
                          |IN_1_Pin|PG_5V_Pin|LEM_OC_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_FLT_Pin LEM_OC_2_Pin */
  GPIO_InitStruct.Pin = PWR_FLT_Pin|LEM_OC_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PG_3V3A_Pin */
  GPIO_InitStruct.Pin = PG_3V3A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PG_3V3A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_24V_EN_Pin PWR_SLEEP_Pin */
  GPIO_InitStruct.Pin = PWR_24V_EN_Pin|PWR_SLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_5V_Pin EN_3V3A_Pin DSEL1_0_Pin OUT3_0_Pin
                           OUT2_0_Pin DSEL0_0_Pin DEN_0_Pin OUT1_0_Pin */
  GPIO_InitStruct.Pin = EN_5V_Pin|EN_3V3A_Pin|DSEL1_0_Pin|OUT3_0_Pin
                          |OUT2_0_Pin|DSEL0_0_Pin|DEN_0_Pin|OUT1_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LEM_OC_7_Pin */
  GPIO_InitStruct.Pin = LEM_OC_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LEM_OC_7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT0_0_Pin DSEL1_1_Pin */
  GPIO_InitStruct.Pin = OUT0_0_Pin|DSEL1_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LEM_OC_3_Pin IN_15_Pin IN_14_Pin IN_13_Pin
                           IN_12_Pin */
  GPIO_InitStruct.Pin = LEM_OC_3_Pin|IN_15_Pin|IN_14_Pin|IN_13_Pin
                          |IN_12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT3_1_Pin OUT2_1_Pin DSEL0_1_Pin DEN_1_Pin
                           OUT1_1_Pin OUT0_1_Pin DSEL1_2_Pin OUT3_2_Pin
                           OUT2_4_Pin OUT3_4_Pin DSEL1_4_Pin ISOSPI_EN_Pin */
  GPIO_InitStruct.Pin = OUT3_1_Pin|OUT2_1_Pin|DSEL0_1_Pin|DEN_1_Pin
                          |OUT1_1_Pin|OUT0_1_Pin|DSEL1_2_Pin|OUT3_2_Pin
                          |OUT2_4_Pin|OUT3_4_Pin|DSEL1_4_Pin|ISOSPI_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT2_2_Pin DSEL0_2_Pin DEN_2_Pin OUT1_2_Pin
                           OUT0_2_Pin LED_Pin */
  GPIO_InitStruct.Pin = OUT2_2_Pin|DSEL0_2_Pin|DEN_2_Pin|OUT1_2_Pin
                          |OUT0_2_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LEM_OC_9_Pin LEM_OC_10_Pin */
  GPIO_InitStruct.Pin = LEM_OC_9_Pin|LEM_OC_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : DEN_3_Pin DSEL0_3_Pin DSEL1_3_Pin OUT1_4_Pin
                           DEN_4_Pin DSEL0_4_Pin */
  GPIO_InitStruct.Pin = DEN_3_Pin|DSEL0_3_Pin|DSEL1_3_Pin|OUT1_4_Pin
                          |DEN_4_Pin|DSEL0_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT0_3_Pin OUT1_3_Pin OUT2_3_Pin OUT3_3_Pin
                           OUT0_4_Pin */
  GPIO_InitStruct.Pin = OUT0_3_Pin|OUT1_3_Pin|OUT2_3_Pin|OUT3_3_Pin
                          |OUT0_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LEM_OC_1_Pin */
  GPIO_InitStruct.Pin = LEM_OC_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LEM_OC_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_20_Pin IN_19_Pin IN_18_Pin IN_17_Pin
                           IN_16_Pin */
  GPIO_InitStruct.Pin = IN_20_Pin|IN_19_Pin|IN_18_Pin|IN_17_Pin
                          |IN_16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
