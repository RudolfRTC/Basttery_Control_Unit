/**
  ******************************************************************************
  * @file           : adc_dma.h
  * @brief          : ADC DMA Continuous Reading Header
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  * @attention
  *
  * DMA za kontinuirano branje ADC kanalov v ozadju
  * - 10x LEM sensors (ADC_IN0 do ADC_IN9)
  * - 5x BTT6200 IS channels (ADC_IN10 do ADC_IN14)
  * - 1x Power current (ADC_IN15)
  * - Total: 16 ADC channels
  * - Circular DMA buffer
  *
  ******************************************************************************
  */

#ifndef __ADC_DMA_H
#define __ADC_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  ADC channel mapping
  */
typedef enum {
    ADC_DMA_LEM_1 = 0,      // ADC_IN0  (PA0)
    ADC_DMA_LEM_2 = 1,      // ADC_IN1  (PA1)
    ADC_DMA_LEM_3 = 2,      // ADC_IN2  (PA2)
    ADC_DMA_LEM_4 = 3,      // ADC_IN3  (PA3)
    ADC_DMA_LEM_5 = 4,      // ADC_IN4  (PA4)
    ADC_DMA_LEM_6 = 5,      // ADC_IN5  (PA5)
    ADC_DMA_LEM_7 = 6,      // ADC_IN6  (PA6)
    ADC_DMA_LEM_8 = 7,      // ADC_IN7  (PA7)
    ADC_DMA_LEM_9 = 8,      // ADC_IN8  (PB0)
    ADC_DMA_LEM_10 = 9,     // ADC_IN9  (PB1)
    ADC_DMA_IS_0 = 10,      // ADC_IN10 (PC0)
    ADC_DMA_IS_1 = 11,      // ADC_IN11 (PC1)
    ADC_DMA_IS_2 = 12,      // ADC_IN12 (PC2)
    ADC_DMA_IS_3 = 13,      // ADC_IN13 (PC3)
    ADC_DMA_IS_4 = 14,      // ADC_IN14 (PC4)
    ADC_DMA_PWR_CURRENT = 15 // ADC_IN15 (PC5)
} ADC_DMA_Channel_t;

/**
  * @brief  ADC DMA handle structure
  */
typedef struct {
    ADC_HandleTypeDef* hadc;    // ADC handle
    DMA_HandleTypeDef* hdma;    // DMA handle

    // DMA buffer (circular, 16 channels)
    uint16_t adc_buffer[16];

    // Conversion complete flag
    volatile bool conversion_complete;

    // Statistics
    uint32_t conversion_count;
    uint32_t error_count;

    bool is_initialized;

} ADC_DMA_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/

#define ADC_DMA_NUM_CHANNELS    16      // Total ADC channels
#define ADC_DMA_BUFFER_SIZE     16      // DMA buffer size

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Inicializira ADC DMA
  * @param  handle: Pointer na ADC_DMA_HandleTypeDef
  * @param  hadc: Pointer na ADC handle
  * @param  hdma: Pointer na DMA handle
  * @retval HAL status
  */
HAL_StatusTypeDef ADC_DMA_Init(ADC_DMA_HandleTypeDef* handle,
                               ADC_HandleTypeDef* hadc,
                               DMA_HandleTypeDef* hdma);

/**
  * @brief  Zaženi kontinuirano ADC DMA konverzijo
  * @param  handle: Pointer na ADC_DMA_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef ADC_DMA_Start(ADC_DMA_HandleTypeDef* handle);

/**
  * @brief  Ustavi ADC DMA konverzijo
  * @param  handle: Pointer na ADC_DMA_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef ADC_DMA_Stop(ADC_DMA_HandleTypeDef* handle);

/**
  * @brief  Preberi ADC vrednost iz DMA bufferja
  * @param  handle: Pointer na ADC_DMA_HandleTypeDef
  * @param  channel: ADC kanal (ADC_DMA_LEM_1 do ADC_DMA_PWR_CURRENT)
  * @param  value: Pointer na uint16_t za rezultat
  * @retval HAL status
  */
HAL_StatusTypeDef ADC_DMA_GetValue(ADC_DMA_HandleTypeDef* handle,
                                   ADC_DMA_Channel_t channel,
                                   uint16_t* value);

/**
  * @brief  Preberi vse ADC vrednosti
  * @param  handle: Pointer na ADC_DMA_HandleTypeDef
  * @param  values: Array za 16 rezultatov
  * @retval HAL status
  */
HAL_StatusTypeDef ADC_DMA_GetAllValues(ADC_DMA_HandleTypeDef* handle,
                                       uint16_t* values);

/**
  * @brief  Preveri ali je konverzija končana
  * @param  handle: Pointer na ADC_DMA_HandleTypeDef
  * @retval true = conversion complete, false = busy
  */
bool ADC_DMA_IsConversionComplete(ADC_DMA_HandleTypeDef* handle);

/**
  * @brief  Počisti conversion complete flag
  * @param  handle: Pointer na ADC_DMA_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef ADC_DMA_ClearFlag(ADC_DMA_HandleTypeDef* handle);

/**
  * @brief  DMA conversion complete callback
  * @param  hadc: Pointer na ADC handle
  * @retval None
  */
void ADC_DMA_ConvCpltCallback(ADC_HandleTypeDef* hadc);

/**
  * @brief  Pridobi statistiko
  * @param  handle: Pointer na ADC_DMA_HandleTypeDef
  * @param  conv_count: Pointer za conversion count
  * @param  error_count: Pointer za error count
  * @retval HAL status
  */
HAL_StatusTypeDef ADC_DMA_GetStats(ADC_DMA_HandleTypeDef* handle,
                                   uint32_t* conv_count,
                                   uint32_t* error_count);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_DMA_H */
