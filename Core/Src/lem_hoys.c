/**
  ******************************************************************************
  * @file           : lem_hoys.c
  * @brief          : LEM HOYS-S/SP33 Current Sensor Driver Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lem_hoys.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define LEM_ADC_TIMEOUT         100     // ms
#define LEM_MAX_FILTER_SAMPLES  32      // Maximum filter buffer size

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef LEM_ConfigureADC(LEM_HOYS_HandleTypeDef* handle);
static float LEM_GetSensitivity(LEM_HOYS_Model_t model);
static float LEM_GetNominalCurrent(LEM_HOYS_Model_t model);
static float LEM_GetMaxCurrent(LEM_HOYS_Model_t model);
static HAL_StatusTypeDef LEM_AllocateFilterBuffer(LEM_HOYS_HandleTypeDef* handle);
static void LEM_FreeFilterBuffer(LEM_HOYS_HandleTypeDef* handle);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Inicializira LEM HOYS sensor
  */
HAL_StatusTypeDef LEM_HOYS_Init(LEM_HOYS_HandleTypeDef* handle,
                                ADC_HandleTypeDef* hadc,
                                uint32_t adc_channel,
                                LEM_HOYS_Model_t model,
                                uint8_t sensor_id,
                                GPIO_TypeDef* oc_port,
                                uint16_t oc_pin)
{
    if (handle == NULL || hadc == NULL) {
        return HAL_ERROR;
    }

    // Initialize handle structure
    memset(handle, 0, sizeof(LEM_HOYS_HandleTypeDef));

    handle->hadc = hadc;
    handle->adc_channel = adc_channel;
    handle->model = model;
    handle->sensor_id = sensor_id;

    // OC pin configuration
    if (oc_port != NULL) {
        handle->oc_pin.port = oc_port;
        handle->oc_pin.pin = oc_pin;
        handle->has_oc_pin = true;
    } else {
        handle->has_oc_pin = false;
    }

    // Set sensor parameters based on model
    handle->nominal_current = LEM_GetNominalCurrent(model);
    handle->max_current = LEM_GetMaxCurrent(model);
    handle->calibration.sensitivity_mV_A = LEM_GetSensitivity(model);

    // ADC configuration
    handle->adc_resolution = LEM_ADC_RESOLUTION_12BIT;
    handle->vref = LEM_VREF_VOLTAGE;

    // Default calibration (will be updated by actual calibration)
    handle->calibration.zero_offset = handle->adc_resolution / 2;  // Mid-point
    handle->calibration.gain_factor = 1.0f;
    handle->calibration.is_calibrated = false;

    // Filter configuration
    handle->filter_samples = LEM_FILTER_SAMPLES_DEFAULT;
    handle->filter_index = 0;

    // Allocate filter buffer
    if (LEM_AllocateFilterBuffer(handle) != HAL_OK) {
        return HAL_ERROR;
    }

    // Initialize status
    handle->status = LEM_STATUS_NOT_CALIB;
    handle->last_current_A = 0.0f;
    handle->oc_count = 0;
    handle->is_initialized = true;

    return HAL_OK;
}

/**
  * @brief  Kalibriraj sensor (zero current offset)
  */
HAL_StatusTypeDef LEM_HOYS_Calibrate(LEM_HOYS_HandleTypeDef* handle,
                                     uint16_t samples)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    if (samples == 0) {
        samples = 100;  // Default
    }

    uint32_t sum = 0;
    uint16_t adc_value;

    // Accumulate samples
    for (uint16_t i = 0; i < samples; i++) {
        if (LEM_HOYS_ReadRawADC(handle, &adc_value) != HAL_OK) {
            return HAL_ERROR;
        }
        sum += adc_value;
        HAL_Delay(1);  // Small delay between samples
    }

    // Calculate average
    handle->calibration.zero_offset = sum / samples;
    handle->calibration.is_calibrated = true;

    // Update status
    handle->status &= ~LEM_STATUS_NOT_CALIB;

    return HAL_OK;
}

/**
  * @brief  Preberi trenutni tok (raw ADC read)
  */
HAL_StatusTypeDef LEM_HOYS_ReadCurrent(LEM_HOYS_HandleTypeDef* handle,
                                       float* current_A)
{
    if (handle == NULL || current_A == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    uint16_t adc_value;
    HAL_StatusTypeDef status = LEM_HOYS_ReadRawADC(handle, &adc_value);

    if (status == HAL_OK) {
        status = LEM_HOYS_ADCToCurrent(handle, adc_value, current_A);
        if (status == HAL_OK) {
            handle->last_current_A = *current_A;
        }
    }

    return status;
}

/**
  * @brief  Preberi trenutni tok z averaging filtrom
  */
HAL_StatusTypeDef LEM_HOYS_ReadCurrentFiltered(LEM_HOYS_HandleTypeDef* handle,
                                               float* current_A)
{
    if (handle == NULL || current_A == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    float single_reading;
    HAL_StatusTypeDef status = LEM_HOYS_ReadCurrent(handle, &single_reading);

    if (status != HAL_OK) {
        return status;
    }

    // Add to circular buffer
    handle->filter_buffer[handle->filter_index] = single_reading;
    handle->filter_index = (handle->filter_index + 1) % handle->filter_samples;

    // Calculate moving average
    float sum = 0.0f;
    for (uint8_t i = 0; i < handle->filter_samples; i++) {
        sum += handle->filter_buffer[i];
    }

    *current_A = sum / handle->filter_samples;
    handle->last_current_A = *current_A;

    return HAL_OK;
}

/**
  * @brief  Preberi raw ADC vrednost
  */
HAL_StatusTypeDef LEM_HOYS_ReadRawADC(LEM_HOYS_HandleTypeDef* handle,
                                      uint16_t* adc_value)
{
    if (handle == NULL || adc_value == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = handle->adc_channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;  // Medium sampling time

    // Configure ADC channel
    if (HAL_ADC_ConfigChannel(handle->hadc, &sConfig) != HAL_OK) {
        handle->status |= LEM_STATUS_ADC_ERROR;
        return HAL_ERROR;
    }

    // Start ADC conversion
    if (HAL_ADC_Start(handle->hadc) != HAL_OK) {
        handle->status |= LEM_STATUS_ADC_ERROR;
        return HAL_ERROR;
    }

    // Wait for conversion
    if (HAL_ADC_PollForConversion(handle->hadc, LEM_ADC_TIMEOUT) != HAL_OK) {
        handle->status |= LEM_STATUS_ADC_ERROR;
        HAL_ADC_Stop(handle->hadc);
        return HAL_ERROR;
    }

    // Get ADC value
    *adc_value = HAL_ADC_GetValue(handle->hadc);

    // Stop ADC
    HAL_ADC_Stop(handle->hadc);

    // Clear ADC error flag
    handle->status &= ~LEM_STATUS_ADC_ERROR;

    return HAL_OK;
}

/**
  * @brief  Preveri overcurrent status (OC pin)
  */
bool LEM_HOYS_IsOvercurrent(LEM_HOYS_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized || !handle->has_oc_pin) {
        return false;
    }

    // OC pin is active HIGH (typical for LEM HOYS)
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(handle->oc_pin.port,
                                               handle->oc_pin.pin);

    bool is_oc = (pin_state == GPIO_PIN_SET);

    if (is_oc) {
        handle->status |= LEM_STATUS_OVERCURRENT;
        handle->oc_count++;
    } else {
        handle->status &= ~LEM_STATUS_OVERCURRENT;
    }

    return is_oc;
}

/**
  * @brief  Nastavi custom kalibracijo
  */
HAL_StatusTypeDef LEM_HOYS_SetCalibration(LEM_HOYS_HandleTypeDef* handle,
                                          uint16_t zero_offset,
                                          float sensitivity_mV_A)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    handle->calibration.zero_offset = zero_offset;
    handle->calibration.sensitivity_mV_A = sensitivity_mV_A;
    handle->calibration.is_calibrated = true;

    handle->status &= ~LEM_STATUS_NOT_CALIB;

    return HAL_OK;
}

/**
  * @brief  Pridobi kalibracijske podatke
  */
HAL_StatusTypeDef LEM_HOYS_GetCalibration(LEM_HOYS_HandleTypeDef* handle,
                                          LEM_Calibration_t* calibration)
{
    if (handle == NULL || calibration == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    memcpy(calibration, &handle->calibration, sizeof(LEM_Calibration_t));

    return HAL_OK;
}

/**
  * @brief  Reset statistike (OC counter)
  */
HAL_StatusTypeDef LEM_HOYS_ResetStats(LEM_HOYS_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    handle->oc_count = 0;
    handle->last_current_A = 0.0f;

    return HAL_OK;
}

/**
  * @brief  Pridobi status senzorja
  */
LEM_Status_t LEM_HOYS_GetStatus(LEM_HOYS_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return LEM_STATUS_ADC_ERROR;
    }

    return handle->status;
}

/**
  * @brief  Nastavi filter depth (število vzorcev)
  */
HAL_StatusTypeDef LEM_HOYS_SetFilterDepth(LEM_HOYS_HandleTypeDef* handle,
                                          uint8_t samples)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    if (samples == 0 || samples > LEM_MAX_FILTER_SAMPLES) {
        return HAL_ERROR;
    }

    // Free old buffer
    LEM_FreeFilterBuffer(handle);

    // Update sample count
    handle->filter_samples = samples;
    handle->filter_index = 0;

    // Allocate new buffer
    return LEM_AllocateFilterBuffer(handle);
}

/**
  * @brief  Konvertiraj ADC vrednost v tok
  */
HAL_StatusTypeDef LEM_HOYS_ADCToCurrent(LEM_HOYS_HandleTypeDef* handle,
                                        uint16_t adc_value,
                                        float* current_A)
{
    if (handle == NULL || current_A == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    // Convert ADC value to voltage
    float voltage_mV = ((float)adc_value / handle->adc_resolution) *
                       (handle->vref * 1000.0f);

    // Calculate voltage relative to zero offset
    float zero_voltage_mV = ((float)handle->calibration.zero_offset /
                             handle->adc_resolution) *
                            (handle->vref * 1000.0f);

    float delta_mV = voltage_mV - zero_voltage_mV;

    // Convert to current using sensitivity
    // I = delta_V / sensitivity
    *current_A = (delta_mV / handle->calibration.sensitivity_mV_A) *
                 handle->calibration.gain_factor;

    // Check range
    if (fabsf(*current_A) > handle->max_current) {
        handle->status |= LEM_STATUS_OUT_OF_RANGE;
    } else {
        handle->status &= ~LEM_STATUS_OUT_OF_RANGE;
    }

    return HAL_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Get sensitivity based on model
  */
static float LEM_GetSensitivity(LEM_HOYS_Model_t model)
{
    switch (model) {
        case LEM_HOYS_6A:   return LEM_HOYS_6A_SENSITIVITY;
        case LEM_HOYS_15A:  return LEM_HOYS_15A_SENSITIVITY;
        case LEM_HOYS_25A:  return LEM_HOYS_25A_SENSITIVITY;
        case LEM_HOYS_50A:  return LEM_HOYS_50A_SENSITIVITY;
        case LEM_HOYS_100A: return LEM_HOYS_100A_SENSITIVITY;
        default:            return LEM_HOYS_15A_SENSITIVITY;
    }
}

/**
  * @brief  Get nominal current based on model
  */
static float LEM_GetNominalCurrent(LEM_HOYS_Model_t model)
{
    switch (model) {
        case LEM_HOYS_6A:   return 6.0f;
        case LEM_HOYS_15A:  return 15.0f;
        case LEM_HOYS_25A:  return 25.0f;
        case LEM_HOYS_50A:  return 50.0f;
        case LEM_HOYS_100A: return 100.0f;
        default:            return 15.0f;
    }
}

/**
  * @brief  Get maximum current based on model
  */
static float LEM_GetMaxCurrent(LEM_HOYS_Model_t model)
{
    // Max current is typically 2x nominal
    return LEM_GetNominalCurrent(model) * 2.0f;
}

/**
  * @brief  Allocate filter buffer
  */
static HAL_StatusTypeDef LEM_AllocateFilterBuffer(LEM_HOYS_HandleTypeDef* handle)
{
    if (handle->filter_samples == 0) {
        return HAL_ERROR;
    }

    handle->filter_buffer = (float*)malloc(handle->filter_samples * sizeof(float));

    if (handle->filter_buffer == NULL) {
        return HAL_ERROR;
    }

    // Initialize buffer to zero
    memset(handle->filter_buffer, 0, handle->filter_samples * sizeof(float));

    return HAL_OK;
}

/**
  * @brief  Free filter buffer
  */
static void LEM_FreeFilterBuffer(LEM_HOYS_HandleTypeDef* handle)
{
    if (handle->filter_buffer != NULL) {
        free(handle->filter_buffer);
        handle->filter_buffer = NULL;
    }
}

/**
  * @brief  Configure ADC for LEM reading
  */
static HAL_StatusTypeDef LEM_ConfigureADC(LEM_HOYS_HandleTypeDef* handle)
{
    // This function is reserved for future use (DMA configuration, etc.)
    return HAL_OK;
}
