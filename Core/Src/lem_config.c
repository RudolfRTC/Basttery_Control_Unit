/**
  ******************************************************************************
  * @file           : lem_config.c
  * @brief          : LEM HOYS Sensors Configuration Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lem_config.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

LEM_HOYS_HandleTypeDef lem_sensors[LEM_NUM_SENSORS];

/* Private variables ---------------------------------------------------------*/

// Sensor configuration table
static const LEM_SensorConfig_t lem_config_table[LEM_NUM_SENSORS] = {
    // sensor_id, adc_channel,      model,         oc_port,            oc_pin,          name
    {0, ADC_CHANNEL_0,  LEM_HOYS_25A, LEM_OC_1_GPIO_Port,  LEM_OC_1_Pin,  "LEM_1"},
    {1, ADC_CHANNEL_1,  LEM_HOYS_25A, LEM_OC_2_GPIO_Port,  LEM_OC_2_Pin,  "LEM_2"},
    {2, ADC_CHANNEL_2,  LEM_HOYS_25A, LEM_OC_3_GPIO_Port,  LEM_OC_3_Pin,  "LEM_3"},
    {3, ADC_CHANNEL_3,  LEM_HOYS_25A, LEM_OC_4_GPIO_Port,  LEM_OC_4_Pin,  "LEM_4"},
    {4, ADC_CHANNEL_4,  LEM_HOYS_25A, LEM_OC_5_GPIO_Port,  LEM_OC_5_Pin,  "LEM_5"},
    {5, ADC_CHANNEL_5,  LEM_HOYS_25A, LEM_OC_6_GPIO_Port,  LEM_OC_6_Pin,  "LEM_6"},
    {6, ADC_CHANNEL_6,  LEM_HOYS_25A, LEM_OC_7_GPIO_Port,  LEM_OC_7_Pin,  "LEM_7"},
    {7, ADC_CHANNEL_7,  LEM_HOYS_25A, LEM_OC_8_GPIO_Port,  LEM_OC_8_Pin,  "LEM_8"},
    {8, ADC_CHANNEL_8,  LEM_HOYS_25A, LEM_OC_9_GPIO_Port,  LEM_OC_9_Pin,  "LEM_9"},
    {9, ADC_CHANNEL_9,  LEM_HOYS_25A, LEM_OC_10_GPIO_Port, LEM_OC_10_Pin, "LEM_10"}
};

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Inicializira vse LEM sensorje
  */
HAL_StatusTypeDef LEM_Config_Init(ADC_HandleTypeDef* hadc)
{
    if (hadc == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_OK;

    // Initialize all sensors
    for (uint8_t i = 0; i < LEM_NUM_SENSORS; i++) {
        const LEM_SensorConfig_t* cfg = &lem_config_table[i];

        HAL_StatusTypeDef init_status = LEM_HOYS_Init(
            &lem_sensors[i],
            hadc,
            cfg->adc_channel,
            cfg->model,
            cfg->sensor_id,
            cfg->oc_port,
            cfg->oc_pin
        );

        if (init_status != HAL_OK) {
            status = HAL_ERROR;  // Flag error but continue with others
        }
    }

    return status;
}

/**
  * @brief  Kalibriraj vse LEM sensorje
  */
HAL_StatusTypeDef LEM_Config_CalibrateAll(uint16_t samples)
{
    HAL_StatusTypeDef status = HAL_OK;

    for (uint8_t i = 0; i < LEM_NUM_SENSORS; i++) {
        HAL_StatusTypeDef cal_status = LEM_HOYS_Calibrate(&lem_sensors[i], samples);
        if (cal_status != HAL_OK) {
            status = HAL_ERROR;
        }
    }

    return status;
}

/**
  * @brief  Preberi tok na določenem senzorju
  */
HAL_StatusTypeDef LEM_Config_ReadCurrent(uint8_t sensor_id, float* current_A)
{
    if (sensor_id >= LEM_NUM_SENSORS || current_A == NULL) {
        return HAL_ERROR;
    }

    return LEM_HOYS_ReadCurrent(&lem_sensors[sensor_id], current_A);
}

/**
  * @brief  Preberi tok na določenem senzorju (filtered)
  */
HAL_StatusTypeDef LEM_Config_ReadCurrentFiltered(uint8_t sensor_id,
                                                 float* current_A)
{
    if (sensor_id >= LEM_NUM_SENSORS || current_A == NULL) {
        return HAL_ERROR;
    }

    return LEM_HOYS_ReadCurrentFiltered(&lem_sensors[sensor_id], current_A);
}

/**
  * @brief  Preberi vse tokove (filtered)
  */
HAL_StatusTypeDef LEM_Config_ReadAllCurrents(float* currents_A)
{
    if (currents_A == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_OK;

    for (uint8_t i = 0; i < LEM_NUM_SENSORS; i++) {
        HAL_StatusTypeDef read_status =
            LEM_HOYS_ReadCurrentFiltered(&lem_sensors[i], &currents_A[i]);
        if (read_status != HAL_OK) {
            currents_A[i] = 0.0f;  // Set to 0 on error
            status = HAL_ERROR;
        }
    }

    return status;
}

/**
  * @brief  Preveri overcurrent na vseh senzorjih
  */
HAL_StatusTypeDef LEM_Config_CheckOvercurrents(uint16_t* oc_flags)
{
    if (oc_flags == NULL) {
        return HAL_ERROR;
    }

    *oc_flags = 0;

    for (uint8_t i = 0; i < LEM_NUM_SENSORS; i++) {
        if (LEM_HOYS_IsOvercurrent(&lem_sensors[i])) {
            *oc_flags |= (1 << i);  // Set bit for this sensor
        }
    }

    return HAL_OK;
}

/**
  * @brief  Pridobi statistike za senzor
  */
HAL_StatusTypeDef LEM_Config_GetSensorStats(uint8_t sensor_id,
                                            float* last_current,
                                            uint32_t* oc_count)
{
    if (sensor_id >= LEM_NUM_SENSORS) {
        return HAL_ERROR;
    }

    LEM_HOYS_HandleTypeDef* sensor = &lem_sensors[sensor_id];

    if (!sensor->is_initialized) {
        return HAL_ERROR;
    }

    if (last_current != NULL) {
        *last_current = sensor->last_current_A;
    }

    if (oc_count != NULL) {
        *oc_count = sensor->oc_count;
    }

    return HAL_OK;
}

/**
  * @brief  Reset statistike za vse senzorje
  */
HAL_StatusTypeDef LEM_Config_ResetAllStats(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    for (uint8_t i = 0; i < LEM_NUM_SENSORS; i++) {
        HAL_StatusTypeDef reset_status = LEM_HOYS_ResetStats(&lem_sensors[i]);
        if (reset_status != HAL_OK) {
            status = HAL_ERROR;
        }
    }

    return status;
}

/**
  * @brief  Print sensor info via UART (debug)
  */
HAL_StatusTypeDef LEM_Config_PrintSensorInfo(UART_HandleTypeDef* uart,
                                             uint8_t sensor_id)
{
    if (uart == NULL || sensor_id >= LEM_NUM_SENSORS) {
        return HAL_ERROR;
    }

    LEM_HOYS_HandleTypeDef* sensor = &lem_sensors[sensor_id];
    const LEM_SensorConfig_t* cfg = &lem_config_table[sensor_id];

    if (!sensor->is_initialized) {
        char msg[] = "Sensor not initialized\r\n";
        HAL_UART_Transmit(uart, (uint8_t*)msg, strlen(msg), 100);
        return HAL_ERROR;
    }

    char buffer[200];
    int len;

    // Sensor name and model
    const char* model_str;
    switch (sensor->model) {
        case LEM_HOYS_6A:   model_str = "HOYS-6A";   break;
        case LEM_HOYS_15A:  model_str = "HOYS-15A";  break;
        case LEM_HOYS_25A:  model_str = "HOYS-25A";  break;
        case LEM_HOYS_50A:  model_str = "HOYS-50A";  break;
        case LEM_HOYS_100A: model_str = "HOYS-100A"; break;
        default:            model_str = "Unknown";   break;
    }

    len = snprintf(buffer, sizeof(buffer), "\r\n=== %s (%s) ===\r\n", cfg->name, model_str);
    HAL_UART_Transmit(uart, (uint8_t*)buffer, len, 100);

    // Nominal and max current
    len = snprintf(buffer, sizeof(buffer), "Nominal: %.1fA, Max: %.1fA\r\n",
                  sensor->nominal_current, sensor->max_current);
    HAL_UART_Transmit(uart, (uint8_t*)buffer, len, 100);

    // Calibration status
    len = snprintf(buffer, sizeof(buffer), "Calibrated: %s\r\n",
                  sensor->calibration.is_calibrated ? "YES" : "NO");
    HAL_UART_Transmit(uart, (uint8_t*)buffer, len, 100);

    if (sensor->calibration.is_calibrated) {
        len = snprintf(buffer, sizeof(buffer), "Zero offset: %u ADC, Sensitivity: %.2f mV/A\r\n",
                      sensor->calibration.zero_offset,
                      sensor->calibration.sensitivity_mV_A);
        HAL_UART_Transmit(uart, (uint8_t*)buffer, len, 100);
    }

    // Last reading
    int current_int = (int)sensor->last_current_A;
    int current_frac = (int)(fabsf(sensor->last_current_A - current_int) * 1000);
    len = snprintf(buffer, sizeof(buffer), "Last reading: %d.%03d A\r\n",
                  current_int, current_frac);
    HAL_UART_Transmit(uart, (uint8_t*)buffer, len, 100);

    // OC counter
    len = snprintf(buffer, sizeof(buffer), "Overcurrent events: %lu\r\n", sensor->oc_count);
    HAL_UART_Transmit(uart, (uint8_t*)buffer, len, 100);

    // Status flags
    len = snprintf(buffer, sizeof(buffer), "Status: 0x%02X ", sensor->status);
    HAL_UART_Transmit(uart, (uint8_t*)buffer, len, 100);

    if (sensor->status == LEM_STATUS_OK) {
        char ok[] = "(OK)\r\n";
        HAL_UART_Transmit(uart, (uint8_t*)ok, strlen(ok), 100);
    } else {
        /* MISRA C 2012: Use snprintf instead of strcat for buffer safety */
        char flags[80];
        int offset = 0;
        offset += snprintf(flags + offset, sizeof(flags) - offset, "(");
        if (sensor->status & LEM_STATUS_OVERCURRENT)  offset += snprintf(flags + offset, sizeof(flags) - offset, "OC ");
        if (sensor->status & LEM_STATUS_OUT_OF_RANGE) offset += snprintf(flags + offset, sizeof(flags) - offset, "RANGE ");
        if (sensor->status & LEM_STATUS_NOT_CALIB)    offset += snprintf(flags + offset, sizeof(flags) - offset, "UNCAL ");
        if (sensor->status & LEM_STATUS_ADC_ERROR)    offset += snprintf(flags + offset, sizeof(flags) - offset, "ADC_ERR ");
        (void)snprintf(flags + offset, sizeof(flags) - offset, ")\r\n");
        HAL_UART_Transmit(uart, (uint8_t*)flags, strlen(flags), 100);
    }

    return HAL_OK;
}
