/**
  ******************************************************************************
  * @file           : lem_config.h
  * @brief          : LEM HOYS Sensors Configuration for BMU
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  * @attention
  *
  * Konfiguracija za 10x LEM HOYS current sensorjev
  * - LEM_1 do LEM_10
  * - Overcurrent detection pins (LEM_OC_1 do LEM_OC_10)
  * - ADC channels ADC_IN0 do ADC_IN9
  *
  ******************************************************************************
  */

#ifndef __LEM_CONFIG_H
#define __LEM_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lem_hoys.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  LEM sensor configuration structure
  */
typedef struct {
    uint8_t sensor_id;              // Sensor ID (0-9)
    uint32_t adc_channel;           // ADC channel
    LEM_HOYS_Model_t model;         // Sensor model
    GPIO_TypeDef* oc_port;          // OC pin port
    uint16_t oc_pin;                // OC pin number
    const char* name;               // Sensor name (debug)
} LEM_SensorConfig_t;

/* Exported constants --------------------------------------------------------*/

#define LEM_NUM_SENSORS         10      // Total number of LEM sensors

/* Exported variables --------------------------------------------------------*/

extern LEM_HOYS_HandleTypeDef lem_sensors[LEM_NUM_SENSORS];

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Inicializira vse LEM sensorje
  * @param  hadc: Pointer na ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_Config_Init(ADC_HandleTypeDef* hadc);

/**
  * @brief  Kalibriraj vse LEM sensorje
  * @note   Kliči to funkcijo brez toka skozi sensorje!
  * @param  samples: Število vzorcev za averaging
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_Config_CalibrateAll(uint16_t samples);

/**
  * @brief  Preberi tok na določenem senzorju
  * @param  sensor_id: Sensor ID (0-9)
  * @param  current_A: Pointer na float za rezultat
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_Config_ReadCurrent(uint8_t sensor_id, float* current_A);

/**
  * @brief  Preberi tok na določenem senzorju (filtered)
  * @param  sensor_id: Sensor ID (0-9)
  * @param  current_A: Pointer na float za rezultat
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_Config_ReadCurrentFiltered(uint8_t sensor_id,
                                                 float* current_A);

/**
  * @brief  Preberi vse tokove (filtered)
  * @param  currents_A: Array za 10 rezultatov
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_Config_ReadAllCurrents(float* currents_A);

/**
  * @brief  Preveri overcurrent na vseh senzorjih
  * @param  oc_flags: Pointer na uint16_t za bitfield (bit 0 = LEM_1, itd.)
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_Config_CheckOvercurrents(uint16_t* oc_flags);

/**
  * @brief  Pridobi statistike za senzor
  * @param  sensor_id: Sensor ID (0-9)
  * @param  last_current: Pointer za zadnji tok
  * @param  oc_count: Pointer za OC counter
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_Config_GetSensorStats(uint8_t sensor_id,
                                            float* last_current,
                                            uint32_t* oc_count);

/**
  * @brief  Reset statistike za vse senzorje
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_Config_ResetAllStats(void);

/**
  * @brief  Print sensor info via UART (debug)
  * @param  uart: UART handle
  * @param  sensor_id: Sensor ID (0-9)
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_Config_PrintSensorInfo(UART_HandleTypeDef* uart,
                                             uint8_t sensor_id);

#ifdef __cplusplus
}
#endif

#endif /* __LEM_CONFIG_H */
