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
#include "cy15b256j.h"
#include "temp_logger.h"
#include "lem_hoys.h"
#include "lem_config.h"
#include "bmu_can.h"
#include "adc_dma.h"
#include "can_diagnostics.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static TMP1075_HandleTypeDef htmp1075;
static CY15B256J_HandleTypeDef hfram;
static TempLogger_HandleTypeDef htemplogger;
static BMU_CAN_HandleTypeDef hbmucan;
ADC_DMA_HandleTypeDef hadc_dma;  // Non-static - used by lem_hoys.c and btt6200_4esa.c

// CAN message counters
static uint32_t can_heartbeat_counter = 0;

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

  // I2C scan za FRAM CY15B256J
  if (HAL_I2C_IsDeviceReady(&hi2c2, 0x50 << 1, 3, 100) == HAL_OK) {
      uint8_t fram_found[] = "FRAM CY15B256J detected at 0x50\r\n";
      HAL_UART_Transmit(&huart1, fram_found, sizeof(fram_found)-1, 100);
  } else {
      uint8_t fram_notfound[] = "FRAM NOT detected at 0x50!\r\n";
      HAL_UART_Transmit(&huart1, fram_notfound, sizeof(fram_notfound)-1, 100);
  }

  // Inicializacija FRAM
  HAL_StatusTypeDef fram_status = CY15B256J_Init(&hfram, &hi2c2, 0x50, NULL, 0);
  if (fram_status == HAL_OK) {
      uint8_t fram_ok[] = "FRAM initialized OK\r\n";
      HAL_UART_Transmit(&huart1, fram_ok, sizeof(fram_ok)-1, 100);

      // Inicializacija Temperature Logger (reset_stats = false za ohranitev podatkov)
      if (TempLogger_Init(&htemplogger, &hfram, false) == HAL_OK) {
          uint8_t logger_ok[] = "Temperature Logger initialized OK\r\n";
          HAL_UART_Transmit(&huart1, logger_ok, sizeof(logger_ok)-1, 100);

          // Izpiši trenutne statistike
          TempLog_Stats_t stats;
          if (TempLogger_GetStats(&htemplogger, &stats) == HAL_OK) {
              (void)snprintf(uart_buf, sizeof(uart_buf), "Stored samples: %lu, Alerts: %lu\r\n",
                      stats.sample_count, stats.alert_count);
              HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);

              if (stats.sample_count > 0) {
                  (void)snprintf(uart_buf, sizeof(uart_buf), "Min: %d.%02dC, Max: %d.%02dC\r\n",
                          stats.min_temp/100, abs(stats.min_temp%100),
                          stats.max_temp/100, abs(stats.max_temp%100));
                  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
              }
          }
      }
  } else {
      uint8_t fram_fail[] = "FRAM initialization FAILED!\r\n";
      HAL_UART_Transmit(&huart1, fram_fail, sizeof(fram_fail)-1, 100);
  }

  /* Inicializacija LEM HOYS current sensorjev */
  uint8_t lem_msg[] = "\r\n=== LEM HOYS Current Sensors ===\r\n";
  HAL_UART_Transmit(&huart1, lem_msg, sizeof(lem_msg)-1, 100);

  if (LEM_Config_Init(&hadc1) == HAL_OK) {
      uint8_t lem_ok[] = "LEM sensors initialized OK\r\n";
      HAL_UART_Transmit(&huart1, lem_ok, sizeof(lem_ok)-1, 100);

      // Kalibriraj vse senzorje (POMEMBNO: mora biti 0A!)
      uint8_t cal_msg[] = "Calibrating LEM sensors (ensure 0A)...\r\n";
      HAL_UART_Transmit(&huart1, cal_msg, sizeof(cal_msg)-1, 100);

      if (LEM_Config_CalibrateAll(50) == HAL_OK) {
          uint8_t cal_ok[] = "LEM calibration complete\r\n";
          HAL_UART_Transmit(&huart1, cal_ok, sizeof(cal_ok)-1, 100);
      } else {
          uint8_t cal_fail[] = "LEM calibration FAILED!\r\n";
          HAL_UART_Transmit(&huart1, cal_fail, sizeof(cal_fail)-1, 100);
      }
  } else {
      uint8_t lem_fail[] = "LEM initialization FAILED!\r\n";
      HAL_UART_Transmit(&huart1, lem_fail, sizeof(lem_fail)-1, 100);
  }

  /* Inicializacija CAN protokola (500 kbps) */
  uint8_t can_msg[] = "\r\n=== CAN Bus Protocol ===\r\n";
  HAL_UART_Transmit(&huart1, can_msg, sizeof(can_msg)-1, 100);

  // Debug: Preveri CAN1 state pred inicializacijo
  (void)snprintf(uart_buf, sizeof(uart_buf), "CAN1 State before init: 0x%02X\r\n", hcan1.State);
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);

  if (BMU_CAN_Init(&hbmucan, &hcan1, &hcan2) == HAL_OK) {
      uint8_t can_ok[] = "CAN bus initialized OK (500 kbps)\r\n";
      HAL_UART_Transmit(&huart1, can_ok, sizeof(can_ok)-1, 100);

      // Debug: Preveri CAN1 error code
      uint32_t can_error = HAL_CAN_GetError(&hcan1);
      (void)snprintf(uart_buf, sizeof(uart_buf), "CAN1 Error Code: 0x%08lX\r\n", can_error);
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);

      /* DIAGNOSTICS: Uncomment to run full CAN diagnostics */
      #if 1  // Set to 1 to enable CAN diagnostics, 0 to disable
      CAN_RunDiagnostics(&hcan1, &huart1);
      #endif
  } else {
      uint8_t can_fail[] = "CAN initialization FAILED!\r\n";
      HAL_UART_Transmit(&huart1, can_fail, sizeof(can_fail)-1, 100);

      // Debug: Izpiši error code
      uint32_t can_error = HAL_CAN_GetError(&hcan1);
      (void)snprintf(uart_buf, sizeof(uart_buf), "CAN1 Error Code: 0x%08lX\r\n", can_error);
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
      (void)snprintf(uart_buf, sizeof(uart_buf), "CAN1 State: 0x%02X\r\n", hcan1.State);
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
  }

  /* Inicializacija ADC DMA - avtomatsko kontinuirano branje vseh 16 kanalov */
  if (ADC_DMA_Init(&hadc_dma, &hadc1, &hdma_adc1) == HAL_OK) {
    ADC_DMA_Start(&hadc_dma);
  }

  /* inicializacija BTT6200 modulov */
  BTT6200_Config_Init(&hadc1);   // ali &hadc, isto kot si nastavil v config.c
  BTT6200_Init(&btt6200_modules[0]);

  uint8_t ready[] = "\r\n*** System Ready ***\r\n\r\n";
  HAL_UART_Transmit(&huart1, ready, sizeof(ready)-1, 100);

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
	  int16_t temp_x100 = 0;  // Deklarirano zunaj if bloka za CAN
	  bool is_alert = false;   // Deklarirano zunaj if bloka za CAN
	  if (TMP1075_ReadTemperature(&htmp1075, &tC) == HAL_OK) {
	      // Uspešno branje temperature
	      int temp_int = (int)tC;
	      int temp_frac = (int)((tC - temp_int) * 100);
	      if (temp_frac < 0) temp_frac = -temp_frac;  // Absolutna vrednost za decimale

	      // Konvertiraj v int16_t (°C × 100) za shranjevanje v FRAM
	      temp_x100 = (int16_t)(tC * 100.0f);
	      is_alert = (tC < 0.0f);

	      // Shrani v FRAM (če je inicializiran)
	      if (htemplogger.is_initialized) {
	          if (TempLogger_LogTemperature(&htemplogger, temp_x100, is_alert) == HAL_OK) {
	              // Uspešno shranjeno v FRAM
	          }
	      }

	      // ALERT: Temperatura pod 0°C
	      if (is_alert) {
	          (void)snprintf(uart_buf, sizeof(uart_buf), "*** ALERT! Temperature: %d.%02d C (BELOW 0C!) ***\r\n", temp_int, temp_frac);
	          HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
	          // Prižgi LED - hitro utripanje za alarm
	          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
	          HAL_Delay(100);
	          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
	          HAL_Delay(100);
	          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
	      } else {
	          (void)snprintf(uart_buf, sizeof(uart_buf), "Temperature: %d.%02d C\r\n", temp_int, temp_frac);
	          HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
	      }
	  } else {
	      uint8_t err[] = "Temperature read FAILED!\r\n";
	      HAL_UART_Transmit(&huart1, err, sizeof(err)-1, 100);
	  }

	  // ADC DMA Debug: Prikaz ADC surovih vrednosti in DMA statistike
	  #if 0  // Set to 1 to enable ADC DMA debug output
	  if (hadc_dma.is_initialized) {
	      uint32_t conv_count, error_count;
	      if (ADC_DMA_GetStats(&hadc_dma, &conv_count, &error_count) == HAL_OK) {
	          (void)snprintf(uart_buf, sizeof(uart_buf), "[ADC DMA] Conv: %lu, Errors: %lu\r\n", conv_count, error_count);
	          HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
	      }

	      // Prikaz surovih ADC vrednosti za LEM senzorje 1-3
	      uint16_t adc_value;
	      for (uint8_t i = 0; i < 3; i++) {
	          if (ADC_DMA_GetValue(&hadc_dma, (ADC_DMA_Channel_t)i, &adc_value) == HAL_OK) {
	              (void)snprintf(uart_buf, sizeof(uart_buf), "ADC_CH%d: %4u  ", i, adc_value);
	              HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
	          }
	      }
	      uint8_t nl[] = "\r\n";
	      HAL_UART_Transmit(&huart1, nl, sizeof(nl)-1, 100);
	  }
	  #endif

	  // Preberi LEM sensorje (prikaz prvih 3 za demo)
	  float lem_currents[3];
	  for (uint8_t i = 0; i < 3; i++) {
	      if (LEM_Config_ReadCurrentFiltered(i, &lem_currents[i]) == HAL_OK) {
	          int curr_int = (int)lem_currents[i];
	          int curr_frac = (int)(fabsf(lem_currents[i] - curr_int) * 1000);
	          (void)snprintf(uart_buf, sizeof(uart_buf), "LEM_%d: %d.%03d A  ", i+1, curr_int, curr_frac);
	          HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
	      }
	  }
	  uint8_t newline[] = "\r\n";
	  HAL_UART_Transmit(&huart1, newline, sizeof(newline)-1, 100);

	  // Preveri overcurrent na vseh LEM senzorjih
	  uint16_t oc_flags = 0;
	  LEM_Config_CheckOvercurrents(&oc_flags);
	  if (oc_flags != 0) {
	      (void)snprintf(uart_buf, sizeof(uart_buf), "*** OVERCURRENT DETECTED! Flags: 0x%04X ***\r\n", oc_flags);
	      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
	  }

	  // ========== CAN Bus Data Transmission ==========
	  #if 1  // Debug: Check CAN state before TX
	  static uint32_t debug_counter = 0;
	  if (++debug_counter % 10 == 0) {  // Every 10 seconds
	      HAL_CAN_StateTypeDef can_state = HAL_CAN_GetState(&hcan1);
	      (void)snprintf(uart_buf, sizeof(uart_buf), "[DEBUG] CAN Init:%d State:0x%02X\r\n",
	                    hbmucan.is_initialized, can_state);
	      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
	  }
	  #endif

	  if (hbmucan.is_initialized) {
	      // 1. Pošlji Temperature message (0x101)
	      BMU_Temperature_Msg_t temp_msg = {0};
	      temp_msg.temperature_C = temp_x100;
	      temp_msg.alert_flag = is_alert ? 1 : 0;
	      temp_msg.sensor_status = 0;
	      if (htemplogger.is_initialized) {
	          TempLog_Stats_t stats;
	          if (TempLogger_GetStats(&htemplogger, &stats) == HAL_OK) {
	              temp_msg.min_temp_C = stats.min_temp;
	              temp_msg.max_temp_C = stats.max_temp;
	          }
	      }
	      BMU_CAN_SendTemperature(&hbmucan, &temp_msg);

	      // 2. Pošlji Power Supply Status (0x102)
	      BMU_PowerSupply_Msg_t pwr_msg = {0};
	      pwr_msg.pg_5v = HAL_GPIO_ReadPin(PG_5V_GPIO_Port, PG_5V_Pin);
	      pwr_msg.pg_3v3a = HAL_GPIO_ReadPin(PG_3V3A_GPIO_Port, PG_3V3A_Pin);
	      pwr_msg.pwr_sleep_state = HAL_GPIO_ReadPin(PWR_SLEEP_GPIO_Port, PWR_SLEEP_Pin);

	      /* PWR_CURRENT: Read from ADC_IN15 (PC5) via DMA
	       * PWR_VOLTAGE: Not configured in hardware - using nominal value
	       *
	       * NOTE: Za pravo merjenje 24V napetosti je potreben:
	       * 1. Voltage divider circuit (npr. 10k/1k = 11:1 ratio za 24V->2.18V)
	       * 2. Dodaten ADC kanal v CubeMX konfiguraciji
	       * 3. Kalibracija voltage divider faktorja
	       */

	      // Read PWR_CURRENT from ADC DMA buffer (IN15)
	      uint16_t pwr_current_adc = 0;
	      if (hadc_dma.is_initialized) {
	          ADC_DMA_GetValue(&hadc_dma, ADC_DMA_PWR_CURRENT, &pwr_current_adc);
	      }

	      // Convert ADC to current (assuming current sense resistor circuit)
	      // TODO: Adjust conversion factor based on hardware design
	      // Typical: V_adc = (I_load * R_sense * Gain) where R_sense ~0.1 ohm, Gain=50
	      // I_load = V_adc / (R_sense * Gain)
	      // For 3.3V ref, 12-bit ADC: V_adc = (adc_value * 3300) / 4095
	      uint32_t voltage_uV = ((uint32_t)pwr_current_adc * 3300000UL) / 4095UL;
	      pwr_msg.pwr_current_mA = (uint16_t)(voltage_uV / 5000UL);  // Adjust divisor based on R_sense and gain

	      // PWR_VOLTAGE: Use nominal value (no ADC sensing configured)
	      // Alternative: Could monitor PG_24V flag and return 0 if power fault
	      pwr_msg.pwr_voltage_mV = 24000U;  // Nominal 24V (hardware sensing not implemented)
	      pwr_msg.pg_24v = (pwr_msg.pwr_voltage_mV > 20000U) ? 1U : 0U;
	      BMU_CAN_SendPowerSupply(&hbmucan, &pwr_msg);

	      // 3. Pošlji Input States (0x103)
	      BMU_InputStates_Msg_t input_msg = {0};
	      input_msg.input_states = 0;
	      // Preberi vseh 20 digitalnih inputov (IN_1 do IN_20)
	      if (HAL_GPIO_ReadPin(IN_1_GPIO_Port, IN_1_Pin)) input_msg.input_states |= (1 << 0);
	      if (HAL_GPIO_ReadPin(IN_2_GPIO_Port, IN_2_Pin)) input_msg.input_states |= (1 << 1);
	      if (HAL_GPIO_ReadPin(IN_3_GPIO_Port, IN_3_Pin)) input_msg.input_states |= (1 << 2);
	      if (HAL_GPIO_ReadPin(IN_4_GPIO_Port, IN_4_Pin)) input_msg.input_states |= (1 << 3);
	      if (HAL_GPIO_ReadPin(IN_5_GPIO_Port, IN_5_Pin)) input_msg.input_states |= (1 << 4);
	      if (HAL_GPIO_ReadPin(IN_6_GPIO_Port, IN_6_Pin)) input_msg.input_states |= (1 << 5);
	      if (HAL_GPIO_ReadPin(IN_7_GPIO_Port, IN_7_Pin)) input_msg.input_states |= (1 << 6);
	      if (HAL_GPIO_ReadPin(IN_8_GPIO_Port, IN_8_Pin)) input_msg.input_states |= (1 << 7);
	      if (HAL_GPIO_ReadPin(IN_9_GPIO_Port, IN_9_Pin)) input_msg.input_states |= (1 << 8);
	      if (HAL_GPIO_ReadPin(IN_10_GPIO_Port, IN_10_Pin)) input_msg.input_states |= (1 << 9);
	      if (HAL_GPIO_ReadPin(IN_11_GPIO_Port, IN_11_Pin)) input_msg.input_states |= (1 << 10);
	      if (HAL_GPIO_ReadPin(IN_12_GPIO_Port, IN_12_Pin)) input_msg.input_states |= (1 << 11);
	      if (HAL_GPIO_ReadPin(IN_13_GPIO_Port, IN_13_Pin)) input_msg.input_states |= (1 << 12);
	      if (HAL_GPIO_ReadPin(IN_14_GPIO_Port, IN_14_Pin)) input_msg.input_states |= (1 << 13);
	      if (HAL_GPIO_ReadPin(IN_15_GPIO_Port, IN_15_Pin)) input_msg.input_states |= (1 << 14);
	      if (HAL_GPIO_ReadPin(IN_16_GPIO_Port, IN_16_Pin)) input_msg.input_states |= (1 << 15);
	      if (HAL_GPIO_ReadPin(IN_17_GPIO_Port, IN_17_Pin)) input_msg.input_states |= (1 << 16);
	      if (HAL_GPIO_ReadPin(IN_18_GPIO_Port, IN_18_Pin)) input_msg.input_states |= (1 << 17);
	      if (HAL_GPIO_ReadPin(IN_19_GPIO_Port, IN_19_Pin)) input_msg.input_states |= (1 << 18);
	      if (HAL_GPIO_ReadPin(IN_20_GPIO_Port, IN_20_Pin)) input_msg.input_states |= (1 << 19);
	      BMU_CAN_SendInputStates(&hbmucan, &input_msg);

	      // 4. Pošlji LEM Current messages (0x110-0x112) - FIXED
	      float all_lem_currents[10];
	      LEM_Config_ReadAllCurrents(all_lem_currents);

	      BMU_LEM_Current_Msg_t lem_msg1 = {0};
	      lem_msg1.current_1_mA = (int16_t)(all_lem_currents[0] * 1000);
	      lem_msg1.current_2_mA = (int16_t)(all_lem_currents[1] * 1000);
	      lem_msg1.current_3_mA = (int16_t)(all_lem_currents[2] * 1000);
	      lem_msg1.current_4_mA = (int16_t)(all_lem_currents[3] * 1000);
	      BMU_CAN_SendLEMCurrent(&hbmucan, CAN_ID_LEM_CURRENT_1, &lem_msg1);

	      BMU_LEM_Current_Msg_t lem_msg2 = {0};
	      lem_msg2.current_1_mA = (int16_t)(all_lem_currents[4] * 1000);
	      lem_msg2.current_2_mA = (int16_t)(all_lem_currents[5] * 1000);
	      lem_msg2.current_3_mA = (int16_t)(all_lem_currents[6] * 1000);
	      lem_msg2.current_4_mA = (int16_t)(all_lem_currents[7] * 1000);
	      BMU_CAN_SendLEMCurrent(&hbmucan, CAN_ID_LEM_CURRENT_2, &lem_msg2);

	      BMU_LEM_Current_Msg_t lem_msg3 = {0};
	      lem_msg3.current_1_mA = (int16_t)(all_lem_currents[8] * 1000);
	      lem_msg3.current_2_mA = (int16_t)(all_lem_currents[9] * 1000);
	      lem_msg3.current_3_mA = 0;
	      lem_msg3.current_4_mA = 0;
	      BMU_CAN_SendLEMCurrent(&hbmucan, CAN_ID_LEM_CURRENT_3, &lem_msg3);

	      // 5. Pošlji BTT6200 Detailed Status (0x124-0x128) - vseh 20 outputov
	      for (uint8_t msg_idx = 0; msg_idx < 5; msg_idx++) {
	          BMU_BTT6200_Detailed_Msg_t btt_detail = {0};
	          uint8_t base_out = msg_idx * 4;

	          // STATUS_OK means channel is enabled and working
	          btt_detail.out0_state = (BTT6200_Config_GetStatus(base_out + 0) == BTT6200_STATUS_OK) ? 1U : 0U;
	          btt_detail.out1_state = (BTT6200_Config_GetStatus(base_out + 1) == BTT6200_STATUS_OK) ? 1U : 0U;
	          btt_detail.out2_state = (BTT6200_Config_GetStatus(base_out + 2) == BTT6200_STATUS_OK) ? 1U : 0U;
	          btt_detail.out3_state = (BTT6200_Config_GetStatus(base_out + 3) == BTT6200_STATUS_OK) ? 1U : 0U;

	          uint32_t curr_mA;
	          BTT6200_Config_ReadCurrent(base_out + 0, &curr_mA);
	          btt_detail.out0_current_mA = (uint16_t)curr_mA;
	          BTT6200_Config_ReadCurrent(base_out + 1, &curr_mA);
	          btt_detail.out1_current_mA = (uint16_t)curr_mA;

	          BMU_CAN_SendBTTDetailed(&hbmucan, CAN_ID_BTT6200_DETAIL_1 + msg_idx, &btt_detail);
	      }

	      // 6. Pošlji Heartbeat (0x1FF)
	      BMU_CAN_SendHeartbeat(&hbmucan, can_heartbeat_counter++);

	      // Debug: CAN stats
	      if (can_heartbeat_counter % 10 == 0) {
	          uint32_t tx_count, rx_count, err_count;
	          BMU_CAN_GetStats(&hbmucan, &tx_count, &rx_count, &err_count);
	          (void)snprintf(uart_buf, sizeof(uart_buf), "[CAN] TX:%lu RX:%lu ERR:%lu\r\n", tx_count, rx_count, err_count);
	          HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
	      }
	  }

	  // ========== Main Loop Delay ==========
	  HAL_Delay(1000);

#if 0  /* TEST CODE - Disabled for production */
	  // BTT6200 channel test sequence (enable manually for testing)
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
	  uint32_t i_mA;
	  if (BTT6200_ReadChannelCurrent(&btt6200_modules[1], BTT6200_CH1, &i_mA) == HAL_OK) {
	      // i_mA = tok v mA (po tvoji formuli v driverju)
	  }
#endif  /* TEST CODE */

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
  hadc1.Init.ScanConvMode = ENABLE;                    // Enable for multiple channels
  hadc1.Init.ContinuousConvMode = ENABLE;              // Enable for continuous conversion
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 16;                     // 16 channels total
  hadc1.Init.DMAContinuousRequests = ENABLE;           // Enable DMA continuous requests
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;          // End of sequence
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure all 16 ADC channels in sequence
   *  Channels mapped to:
   *  - IN0-IN9: LEM sensors 1-10 (PA0-PA7, PB0-PB1)
   *  - IN10-IN14: BTT6200 IS channels 0-4 (PC0-PC4)
   *  - IN15: Power current (PC5)
  */
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

  // LEM sensors (IN0-IN9)
  sConfig.Channel = ADC_CHANNEL_0; sConfig.Rank = 1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_1; sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_2; sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_3; sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_4; sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_5; sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_6; sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_7; sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_8; sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_9; sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  // BTT6200 IS channels (IN10-IN14)
  sConfig.Channel = ADC_CHANNEL_10; sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_11; sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_12; sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_13; sConfig.Rank = 14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_14; sConfig.Rank = 15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  // Power current (IN15)
  sConfig.Channel = ADC_CHANNEL_15; sConfig.Rank = 16;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

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
  hcan1.Init.Prescaler = 2;  // 500 kbps: 16MHz / (2 * 16) = 500 kbps
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;  // Auto recovery iz bus-off
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;  // Ponovno pošiljanje
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
  hcan2.Init.Prescaler = 2;  // 500 kbps: 16MHz / (2 * 16) = 500 kbps
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;  // Auto recovery iz bus-off
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;  // Ponovno pošiljanje
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

/**
  * @brief  CAN RX FIFO 0 message pending callback
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    // Call BMU CAN RX handler
    BMU_CAN_RxCallback(hcan);
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
