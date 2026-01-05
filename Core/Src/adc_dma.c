/**
  ******************************************************************************
  * @file           : adc_dma.c
  * @brief          : ADC DMA Continuous Reading Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc_dma.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static ADC_DMA_HandleTypeDef* g_adc_dma_handle = NULL;

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Inicializira ADC DMA
  */
HAL_StatusTypeDef ADC_DMA_Init(ADC_DMA_HandleTypeDef* handle,
                               ADC_HandleTypeDef* hadc,
                               DMA_HandleTypeDef* hdma)
{
    if (handle == NULL || hadc == NULL) {
        return HAL_ERROR;
    }

    memset(handle, 0, sizeof(ADC_DMA_HandleTypeDef));

    handle->hadc = hadc;
    handle->hdma = hdma;
    handle->conversion_complete = false;
    handle->conversion_count = 0;
    handle->error_count = 0;
    handle->is_initialized = true;

    // Save global handle for callbacks
    g_adc_dma_handle = handle;

    return HAL_OK;
}

/**
  * @brief  Zaženi kontinuirano ADC DMA konverzijo
  */
HAL_StatusTypeDef ADC_DMA_Start(ADC_DMA_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    // Start ADC DMA in circular mode
    HAL_StatusTypeDef status = HAL_ADC_Start_DMA(handle->hadc,
                                                 (uint32_t*)handle->adc_buffer,
                                                 ADC_DMA_BUFFER_SIZE);

    if (status != HAL_OK) {
        handle->error_count++;
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief  Ustavi ADC DMA konverzijo
  */
HAL_StatusTypeDef ADC_DMA_Stop(ADC_DMA_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    return HAL_ADC_Stop_DMA(handle->hadc);
}

/**
  * @brief  Preberi ADC vrednost iz DMA bufferja
  */
HAL_StatusTypeDef ADC_DMA_GetValue(ADC_DMA_HandleTypeDef* handle,
                                   ADC_DMA_Channel_t channel,
                                   uint16_t* value)
{
    if (handle == NULL || value == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    if (channel >= ADC_DMA_NUM_CHANNELS) {
        return HAL_ERROR;
    }

    // Read from circular buffer (DMA updates this automatically)
    *value = handle->adc_buffer[channel];

    return HAL_OK;
}

/**
  * @brief  Preberi vse ADC vrednosti
  */
HAL_StatusTypeDef ADC_DMA_GetAllValues(ADC_DMA_HandleTypeDef* handle,
                                       uint16_t* values)
{
    if (handle == NULL || values == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    // Copy entire buffer
    memcpy(values, handle->adc_buffer, ADC_DMA_NUM_CHANNELS * sizeof(uint16_t));

    return HAL_OK;
}

/**
  * @brief  Preveri ali je konverzija končana
  */
bool ADC_DMA_IsConversionComplete(ADC_DMA_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return false;
    }

    return handle->conversion_complete;
}

/**
  * @brief  Počisti conversion complete flag
  */
HAL_StatusTypeDef ADC_DMA_ClearFlag(ADC_DMA_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    handle->conversion_complete = false;

    return HAL_OK;
}

/**
  * @brief  DMA conversion complete callback
  */
void ADC_DMA_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (g_adc_dma_handle != NULL) {
        g_adc_dma_handle->conversion_complete = true;
        g_adc_dma_handle->conversion_count++;
    }
}

/**
  * @brief  Pridobi statistiko
  */
HAL_StatusTypeDef ADC_DMA_GetStats(ADC_DMA_HandleTypeDef* handle,
                                   uint32_t* conv_count,
                                   uint32_t* error_count)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    if (conv_count != NULL) {
        *conv_count = handle->conversion_count;
    }
    if (error_count != NULL) {
        *error_count = handle->error_count;
    }

    return HAL_OK;
}

/* Weak callback function that can be overridden */
__weak void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    ADC_DMA_ConvCpltCallback(hadc);
}
