/**
  ******************************************************************************
  * @file           : lem_hoys.h
  * @brief          : LEM HOYS-S/SP33 Current Sensor Driver Header
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  * @attention
  *
  * Driver za LEM HOYS-S/SP33 series Hall-effect current sensors
  * - Bidirectional current measurement
  * - Analog output (Vref ± proportional to current)
  * - Overcurrent detection (OC pin)
  * - Configurable sensitivity and ranges
  *
  * Supported models:
  * - HOYS 6-P/SP33   : ±6A nominal, 25mV/A
  * - HOYS 15-P/SP33  : ±15A nominal, 10mV/A
  * - HOYS 25-P/SP33  : ±25A nominal, 6mV/A
  * - HOYS 50-P/SP33  : ±50A nominal, 3mV/A
  * - HOYS 100-P/SP33 : ±100A nominal, 1.5mV/A
  *
  ******************************************************************************
  */

#ifndef __LEM_HOYS_H
#define __LEM_HOYS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  LEM HOYS sensor model type
  */
typedef enum {
    LEM_HOYS_6A   = 0,    // ±6A nominal, 25mV/A sensitivity
    LEM_HOYS_15A  = 1,    // ±15A nominal, 10mV/A sensitivity
    LEM_HOYS_25A  = 2,    // ±25A nominal, 6mV/A sensitivity
    LEM_HOYS_50A  = 3,    // ±50A nominal, 3mV/A sensitivity
    LEM_HOYS_100A = 4     // ±100A nominal, 1.5mV/A sensitivity
} LEM_HOYS_Model_t;

/**
  * @brief  LEM sensor status flags
  */
typedef enum {
    LEM_STATUS_OK           = 0x00,  // Normal operation
    LEM_STATUS_OVERCURRENT  = 0x01,  // OC pin triggered
    LEM_STATUS_OUT_OF_RANGE = 0x02,  // ADC reading out of valid range
    LEM_STATUS_NOT_CALIB    = 0x04,  // Sensor not calibrated
    LEM_STATUS_ADC_ERROR    = 0x08   // ADC read error
} LEM_Status_t;

/**
  * @brief  LEM sensor calibration data
  */
typedef struct {
    uint16_t zero_offset;      // ADC value at 0A (Vref)
    float sensitivity_mV_A;    // mV per Ampere
    float gain_factor;         // Calibration gain adjustment
    bool is_calibrated;        // Calibration valid flag
} LEM_Calibration_t;

/**
  * @brief  LEM GPIO configuration
  */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} LEM_GPIO_t;

/**
  * @brief  LEM HOYS sensor handle structure
  */
typedef struct {
    // Hardware configuration
    ADC_HandleTypeDef* hadc;        // ADC handle
    uint32_t adc_channel;           // ADC channel (ADC_CHANNEL_0 to ADC_CHANNEL_15)
    LEM_GPIO_t oc_pin;              // Overcurrent detection pin (optional)
    bool has_oc_pin;                // OC pin available

    // Sensor specification
    LEM_HOYS_Model_t model;         // Sensor model
    float nominal_current;          // Nominal current rating (A)
    float max_current;              // Maximum measurable current (A)

    // Calibration
    LEM_Calibration_t calibration;  // Calibration data

    // ADC configuration
    uint16_t adc_resolution;        // ADC resolution (12-bit = 4096)
    float vref;                     // ADC reference voltage (V)

    // Filtering
    uint8_t filter_samples;         // Number of samples for averaging
    float* filter_buffer;           // Circular buffer for filtering
    uint8_t filter_index;           // Current buffer index

    // Status
    LEM_Status_t status;            // Current status
    uint8_t sensor_id;              // Sensor ID (0-9 for LEM_1 to LEM_10)
    bool is_initialized;            // Initialization flag

    // Runtime data
    float last_current_A;           // Last measured current (A)
    uint32_t oc_count;              // Overcurrent event counter

} LEM_HOYS_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/

// Default sensor parameters (at 25°C)
#define LEM_HOYS_6A_SENSITIVITY     25.0f   // mV/A
#define LEM_HOYS_15A_SENSITIVITY    10.0f   // mV/A
#define LEM_HOYS_25A_SENSITIVITY    6.0f    // mV/A
#define LEM_HOYS_50A_SENSITIVITY    3.0f    // mV/A
#define LEM_HOYS_100A_SENSITIVITY   1.5f    // mV/A

// Default Vref (mid-supply voltage)
#define LEM_HOYS_VREF_DEFAULT       1.65f   // V (for 3.3V supply)

// ADC configuration
#define LEM_ADC_RESOLUTION_12BIT    4096
#define LEM_VREF_VOLTAGE            3.3f    // V

// Filtering
#define LEM_FILTER_SAMPLES_DEFAULT  8       // Moving average filter depth

// Overcurrent detection
#define LEM_OC_DEBOUNCE_MS          10      // Debounce time for OC pin

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Inicializira LEM HOYS sensor
  * @param  handle: Pointer na LEM_HOYS_HandleTypeDef
  * @param  hadc: Pointer na ADC handle
  * @param  adc_channel: ADC channel number (ADC_CHANNEL_0 to ADC_CHANNEL_15)
  * @param  model: Sensor model (LEM_HOYS_6A to LEM_HOYS_100A)
  * @param  sensor_id: Sensor ID (0-9)
  * @param  oc_port: OC pin GPIO port (NULL če ni)
  * @param  oc_pin: OC pin GPIO pin number
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_HOYS_Init(LEM_HOYS_HandleTypeDef* handle,
                                ADC_HandleTypeDef* hadc,
                                uint32_t adc_channel,
                                LEM_HOYS_Model_t model,
                                uint8_t sensor_id,
                                GPIO_TypeDef* oc_port,
                                uint16_t oc_pin);

/**
  * @brief  Kalibriraj sensor (zero current offset)
  * @note   Kliči to funkcijo brez toka skozi sensor!
  * @param  handle: Pointer na LEM_HOYS_HandleTypeDef
  * @param  samples: Število vzorcev za averaging (priporočeno 100)
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_HOYS_Calibrate(LEM_HOYS_HandleTypeDef* handle,
                                     uint16_t samples);

/**
  * @brief  Preberi trenutni tok (raw ADC read)
  * @param  handle: Pointer na LEM_HOYS_HandleTypeDef
  * @param  current_A: Pointer na float za rezultat v Amperih
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_HOYS_ReadCurrent(LEM_HOYS_HandleTypeDef* handle,
                                       float* current_A);

/**
  * @brief  Preberi trenutni tok z averaging filtrom
  * @param  handle: Pointer na LEM_HOYS_HandleTypeDef
  * @param  current_A: Pointer na float za rezultat v Amperih
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_HOYS_ReadCurrentFiltered(LEM_HOYS_HandleTypeDef* handle,
                                               float* current_A);

/**
  * @brief  Preberi raw ADC vrednost
  * @param  handle: Pointer na LEM_HOYS_HandleTypeDef
  * @param  adc_value: Pointer na uint16_t za ADC vrednost
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_HOYS_ReadRawADC(LEM_HOYS_HandleTypeDef* handle,
                                      uint16_t* adc_value);

/**
  * @brief  Preveri overcurrent status (OC pin)
  * @param  handle: Pointer na LEM_HOYS_HandleTypeDef
  * @retval true = overcurrent, false = OK
  */
bool LEM_HOYS_IsOvercurrent(LEM_HOYS_HandleTypeDef* handle);

/**
  * @brief  Nastavi custom kalibracijo
  * @param  handle: Pointer na LEM_HOYS_HandleTypeDef
  * @param  zero_offset: ADC value pri 0A
  * @param  sensitivity_mV_A: Sensitivity v mV/A
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_HOYS_SetCalibration(LEM_HOYS_HandleTypeDef* handle,
                                          uint16_t zero_offset,
                                          float sensitivity_mV_A);

/**
  * @brief  Pridobi kalibracijske podatke
  * @param  handle: Pointer na LEM_HOYS_HandleTypeDef
  * @param  calibration: Pointer na LEM_Calibration_t strukturo
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_HOYS_GetCalibration(LEM_HOYS_HandleTypeDef* handle,
                                          LEM_Calibration_t* calibration);

/**
  * @brief  Reset statistike (OC counter)
  * @param  handle: Pointer na LEM_HOYS_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_HOYS_ResetStats(LEM_HOYS_HandleTypeDef* handle);

/**
  * @brief  Pridobi status senzorja
  * @param  handle: Pointer na LEM_HOYS_HandleTypeDef
  * @retval LEM_Status_t flags
  */
LEM_Status_t LEM_HOYS_GetStatus(LEM_HOYS_HandleTypeDef* handle);

/**
  * @brief  Nastavi filter depth (število vzorcev)
  * @param  handle: Pointer na LEM_HOYS_HandleTypeDef
  * @param  samples: Število vzorcev (1-32)
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_HOYS_SetFilterDepth(LEM_HOYS_HandleTypeDef* handle,
                                          uint8_t samples);

/**
  * @brief  Konvertiraj ADC vrednost v tok
  * @param  handle: Pointer na LEM_HOYS_HandleTypeDef
  * @param  adc_value: ADC vrednost
  * @param  current_A: Pointer na float za rezultat
  * @retval HAL status
  */
HAL_StatusTypeDef LEM_HOYS_ADCToCurrent(LEM_HOYS_HandleTypeDef* handle,
                                        uint16_t adc_value,
                                        float* current_A);

#ifdef __cplusplus
}
#endif

#endif /* __LEM_HOYS_H */
