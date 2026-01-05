/**
  ******************************************************************************
  * @file           : tmp1075.h
  * @brief          : TMP1075 Temperature Sensor Driver Header
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  * @attention
  *
  * Driver za Texas Instruments TMP1075 I2C Temperature Sensor
  * - ±1°C accuracy
  * - 12-bit resolution (0.0625°C)
  * - Alert funkcionalnost
  * - I2C interface
  *
  ******************************************************************************
  */

#ifndef __TMP1075_H
#define __TMP1075_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  TMP1075 register addresses
  */
typedef enum {
    TMP1075_REG_TEMP        = 0x00,  // Temperature Register (read-only)
    TMP1075_REG_CONFIG      = 0x01,  // Configuration Register
    TMP1075_REG_TLOW        = 0x02,  // Low Temperature Threshold
    TMP1075_REG_THIGH       = 0x03   // High Temperature Threshold
} TMP1075_Register_t;

/**
  * @brief  TMP1075 configuration bits
  */
typedef struct {
    uint8_t shutdown_mode;       // 1 = shutdown, 0 = continuous
    uint8_t thermostat_mode;     // 0 = comparator, 1 = interrupt
    uint8_t alert_polarity;      // 0 = active low, 1 = active high
    uint8_t fault_queue;         // 00 = 1 fault, 01 = 2, 10 = 4, 11 = 6
    uint8_t conversion_rate;     // 00 = 27.5ms, 01 = 55ms, 10 = 110ms, 11 = 220ms
} TMP1075_Config_t;

/**
  * @brief  TMP1075 device handle
  */
typedef struct {
    I2C_HandleTypeDef* hi2c;     // I2C handle
    uint8_t device_address;      // 7-bit I2C address (0x48-0x4F)
    TMP1075_Config_t config;     // Configuration
    bool is_initialized;         // Initialization flag
} TMP1075_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/

// Default I2C address (A2=A1=A0=GND)
#define TMP1075_I2C_ADDR_DEFAULT    0x48

// I2C address range (based on A2, A1, A0 pins)
#define TMP1075_I2C_ADDR_MIN        0x48
#define TMP1075_I2C_ADDR_MAX        0x4F

// Temperature conversion
#define TMP1075_TEMP_RESOLUTION     0.0625f  // °C per LSB (12-bit)

// Configuration register bits
#define TMP1075_CFG_SD_BIT          0x01     // Shutdown bit
#define TMP1075_CFG_TM_BIT          0x02     // Thermostat mode bit
#define TMP1075_CFG_POL_BIT         0x04     // Polarity bit
#define TMP1075_CFG_F0_BIT          0x08     // Fault queue bit 0
#define TMP1075_CFG_F1_BIT          0x10     // Fault queue bit 1
#define TMP1075_CFG_R0_BIT          0x20     // Conversion rate bit 0
#define TMP1075_CFG_R1_BIT          0x40     // Conversion rate bit 1
#define TMP1075_CFG_OS_BIT          0x80     // One-shot bit

// Conversion rates
#define TMP1075_CONV_RATE_27_5MS    0x00     // 27.5ms
#define TMP1075_CONV_RATE_55MS      0x01     // 55ms
#define TMP1075_CONV_RATE_110MS     0x02     // 110ms
#define TMP1075_CONV_RATE_220MS     0x03     // 220ms

// Fault queue settings
#define TMP1075_FAULT_QUEUE_1       0x00     // 1 fault
#define TMP1075_FAULT_QUEUE_2       0x01     // 2 faults
#define TMP1075_FAULT_QUEUE_4       0x02     // 4 faults
#define TMP1075_FAULT_QUEUE_6       0x03     // 6 faults

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Inicializira TMP1075 sensor
  * @param  handle: Pointer na TMP1075_HandleTypeDef
  * @param  hi2c: Pointer na I2C handle
  * @param  device_addr: I2C address (0x48-0x4F)
  * @retval HAL status
  */
HAL_StatusTypeDef TMP1075_Init(TMP1075_HandleTypeDef* handle,
                               I2C_HandleTypeDef* hi2c,
                               uint8_t device_addr);

/**
  * @brief  Preberi temperaturo v °C
  * @param  handle: Pointer na TMP1075_HandleTypeDef
  * @param  temperature: Pointer na float za rezultat v °C
  * @retval HAL status
  */
HAL_StatusTypeDef TMP1075_ReadTemperature(TMP1075_HandleTypeDef* handle,
                                          float* temperature);

/**
  * @brief  Preberi temperaturo v °C * 100 (integer)
  * @param  handle: Pointer na TMP1075_HandleTypeDef
  * @param  temperature_x100: Pointer na int16_t za rezultat (npr. 2525 = 25.25°C)
  * @retval HAL status
  */
HAL_StatusTypeDef TMP1075_ReadTemperature_Int(TMP1075_HandleTypeDef* handle,
                                              int16_t* temperature_x100);

/**
  * @brief  Nastavi konfiguracijo
  * @param  handle: Pointer na TMP1075_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef TMP1075_WriteConfig(TMP1075_HandleTypeDef* handle);

/**
  * @brief  Preberi konfiguracijo
  * @param  handle: Pointer na TMP1075_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef TMP1075_ReadConfig(TMP1075_HandleTypeDef* handle);

/**
  * @brief  Nastavi low temperature threshold
  * @param  handle: Pointer na TMP1075_HandleTypeDef
  * @param  temp_low: Low threshold v °C
  * @retval HAL status
  */
HAL_StatusTypeDef TMP1075_SetLowThreshold(TMP1075_HandleTypeDef* handle,
                                          float temp_low);

/**
  * @brief  Nastavi high temperature threshold
  * @param  handle: Pointer na TMP1075_HandleTypeDef
  * @param  temp_high: High threshold v °C
  * @retval HAL status
  */
HAL_StatusTypeDef TMP1075_SetHighThreshold(TMP1075_HandleTypeDef* handle,
                                           float temp_high);

/**
  * @brief  Preberi low temperature threshold
  * @param  handle: Pointer na TMP1075_HandleTypeDef
  * @param  temp_low: Pointer na float za rezultat
  * @retval HAL status
  */
HAL_StatusTypeDef TMP1075_GetLowThreshold(TMP1075_HandleTypeDef* handle,
                                          float* temp_low);

/**
  * @brief  Preberi high temperature threshold
  * @param  handle: Pointer na TMP1075_HandleTypeDef
  * @param  temp_high: Pointer na float za rezultat
  * @retval HAL status
  */
HAL_StatusTypeDef TMP1075_GetHighThreshold(TMP1075_HandleTypeDef* handle,
                                           float* temp_high);

/**
  * @brief  Nastavi shutdown mode
  * @param  handle: Pointer na TMP1075_HandleTypeDef
  * @param  shutdown: true = shutdown, false = continuous
  * @retval HAL status
  */
HAL_StatusTypeDef TMP1075_SetShutdownMode(TMP1075_HandleTypeDef* handle,
                                          bool shutdown);

/**
  * @brief  Trigger one-shot conversion (v shutdown mode)
  * @param  handle: Pointer na TMP1075_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef TMP1075_TriggerOneShot(TMP1075_HandleTypeDef* handle);

/**
  * @brief  Nastavi conversion rate
  * @param  handle: Pointer na TMP1075_HandleTypeDef
  * @param  rate: TMP1075_CONV_RATE_xxx
  * @retval HAL status
  */
HAL_StatusTypeDef TMP1075_SetConversionRate(TMP1075_HandleTypeDef* handle,
                                            uint8_t rate);

/**
  * @brief  Preveri če je TMP1075 prisoten na I2C
  * @param  hi2c: Pointer na I2C handle
  * @param  device_addr: I2C address
  * @retval true = device detected, false = not detected
  */
bool TMP1075_IsDeviceReady(I2C_HandleTypeDef* hi2c, uint8_t device_addr);

#ifdef __cplusplus
}
#endif

#endif /* __TMP1075_H */
