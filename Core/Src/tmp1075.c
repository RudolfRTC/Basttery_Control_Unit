/**
  ******************************************************************************
  * @file           : tmp1075.c
  * @brief          : TMP1075 Temperature Sensor Driver Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tmp1075.h"

/* Private defines -----------------------------------------------------------*/
#define TMP1075_I2C_TIMEOUT     100    // I2C timeout (ms)

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef TMP1075_WriteRegister(TMP1075_HandleTypeDef* handle,
                                               uint8_t reg_addr,
                                               uint16_t value);
static HAL_StatusTypeDef TMP1075_ReadRegister(TMP1075_HandleTypeDef* handle,
                                              uint8_t reg_addr,
                                              uint16_t* value);
static float TMP1075_RawToTemperature(uint16_t raw_value);
static uint16_t TMP1075_TemperatureToRaw(float temperature);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Inicializira TMP1075 sensor
  */
HAL_StatusTypeDef TMP1075_Init(TMP1075_HandleTypeDef* handle,
                               I2C_HandleTypeDef* hi2c,
                               uint8_t device_addr)
{
    if (handle == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }

    // Preveri če je I2C address veljaven
    if (device_addr < TMP1075_I2C_ADDR_MIN || device_addr > TMP1075_I2C_ADDR_MAX) {
        return HAL_ERROR;
    }

    handle->hi2c = hi2c;
    handle->device_address = device_addr << 1;  // Convert to 8-bit address

    // Preveri če je device prisoten
    if (!TMP1075_IsDeviceReady(hi2c, device_addr)) {
        return HAL_ERROR;
    }

    // Preberi trenutno konfiguracijo
    HAL_StatusTypeDef status = TMP1075_ReadConfig(handle);
    if (status != HAL_OK) {
        return status;
    }

    // Nastavi default konfiguracijo
    handle->config.shutdown_mode = 0;       // Continuous mode
    handle->config.thermostat_mode = 0;     // Comparator mode
    handle->config.alert_polarity = 0;      // Active low
    handle->config.fault_queue = TMP1075_FAULT_QUEUE_1;
    handle->config.conversion_rate = TMP1075_CONV_RATE_27_5MS;

    // Zapiši konfiguracijo
    status = TMP1075_WriteConfig(handle);
    if (status != HAL_OK) {
        return status;
    }

    handle->is_initialized = true;

    return HAL_OK;
}

/**
  * @brief  Preberi temperaturo v °C
  */
HAL_StatusTypeDef TMP1075_ReadTemperature(TMP1075_HandleTypeDef* handle,
                                          float* temperature)
{
    if (handle == NULL || temperature == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    uint16_t raw_value;
    HAL_StatusTypeDef status = TMP1075_ReadRegister(handle, TMP1075_REG_TEMP, &raw_value);

    if (status == HAL_OK) {
        *temperature = TMP1075_RawToTemperature(raw_value);
    }

    return status;
}

/**
  * @brief  Preberi temperaturo v °C * 100 (integer)
  */
HAL_StatusTypeDef TMP1075_ReadTemperature_Int(TMP1075_HandleTypeDef* handle,
                                              int16_t* temperature_x100)
{
    if (handle == NULL || temperature_x100 == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    float temp_float;
    HAL_StatusTypeDef status = TMP1075_ReadTemperature(handle, &temp_float);

    if (status == HAL_OK) {
        *temperature_x100 = (int16_t)(temp_float * 100.0f);
    }

    return status;
}

/**
  * @brief  Nastavi konfiguracijo
  */
HAL_StatusTypeDef TMP1075_WriteConfig(TMP1075_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    uint16_t config_value = 0;

    // Byte 0 (MSB)
    if (handle->config.shutdown_mode) config_value |= TMP1075_CFG_SD_BIT;
    if (handle->config.thermostat_mode) config_value |= TMP1075_CFG_TM_BIT;
    if (handle->config.alert_polarity) config_value |= TMP1075_CFG_POL_BIT;

    config_value |= (handle->config.fault_queue & 0x03) << 3;
    config_value |= (handle->config.conversion_rate & 0x03) << 5;

    return TMP1075_WriteRegister(handle, TMP1075_REG_CONFIG, config_value);
}

/**
  * @brief  Preberi konfiguracijo
  */
HAL_StatusTypeDef TMP1075_ReadConfig(TMP1075_HandleTypeDef* handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    uint16_t config_value;
    HAL_StatusTypeDef status = TMP1075_ReadRegister(handle, TMP1075_REG_CONFIG, &config_value);

    if (status == HAL_OK) {
        handle->config.shutdown_mode = (config_value & TMP1075_CFG_SD_BIT) ? 1 : 0;
        handle->config.thermostat_mode = (config_value & TMP1075_CFG_TM_BIT) ? 1 : 0;
        handle->config.alert_polarity = (config_value & TMP1075_CFG_POL_BIT) ? 1 : 0;
        handle->config.fault_queue = (config_value >> 3) & 0x03;
        handle->config.conversion_rate = (config_value >> 5) & 0x03;
    }

    return status;
}

/**
  * @brief  Nastavi low temperature threshold
  */
HAL_StatusTypeDef TMP1075_SetLowThreshold(TMP1075_HandleTypeDef* handle,
                                          float temp_low)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    uint16_t raw_value = TMP1075_TemperatureToRaw(temp_low);
    return TMP1075_WriteRegister(handle, TMP1075_REG_TLOW, raw_value);
}

/**
  * @brief  Nastavi high temperature threshold
  */
HAL_StatusTypeDef TMP1075_SetHighThreshold(TMP1075_HandleTypeDef* handle,
                                           float temp_high)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    uint16_t raw_value = TMP1075_TemperatureToRaw(temp_high);
    return TMP1075_WriteRegister(handle, TMP1075_REG_THIGH, raw_value);
}

/**
  * @brief  Preberi low temperature threshold
  */
HAL_StatusTypeDef TMP1075_GetLowThreshold(TMP1075_HandleTypeDef* handle,
                                          float* temp_low)
{
    if (handle == NULL || temp_low == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    uint16_t raw_value;
    HAL_StatusTypeDef status = TMP1075_ReadRegister(handle, TMP1075_REG_TLOW, &raw_value);

    if (status == HAL_OK) {
        *temp_low = TMP1075_RawToTemperature(raw_value);
    }

    return status;
}

/**
  * @brief  Preberi high temperature threshold
  */
HAL_StatusTypeDef TMP1075_GetHighThreshold(TMP1075_HandleTypeDef* handle,
                                           float* temp_high)
{
    if (handle == NULL || temp_high == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    uint16_t raw_value;
    HAL_StatusTypeDef status = TMP1075_ReadRegister(handle, TMP1075_REG_THIGH, &raw_value);

    if (status == HAL_OK) {
        *temp_high = TMP1075_RawToTemperature(raw_value);
    }

    return status;
}

/**
  * @brief  Nastavi shutdown mode
  */
HAL_StatusTypeDef TMP1075_SetShutdownMode(TMP1075_HandleTypeDef* handle,
                                          bool shutdown)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    handle->config.shutdown_mode = shutdown ? 1 : 0;
    return TMP1075_WriteConfig(handle);
}

/**
  * @brief  Trigger one-shot conversion
  */
HAL_StatusTypeDef TMP1075_TriggerOneShot(TMP1075_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    // Read current config
    uint16_t config_value;
    HAL_StatusTypeDef status = TMP1075_ReadRegister(handle, TMP1075_REG_CONFIG, &config_value);
    if (status != HAL_OK) return status;

    // Set OS bit
    config_value |= TMP1075_CFG_OS_BIT;

    return TMP1075_WriteRegister(handle, TMP1075_REG_CONFIG, config_value);
}

/**
  * @brief  Nastavi conversion rate
  */
HAL_StatusTypeDef TMP1075_SetConversionRate(TMP1075_HandleTypeDef* handle,
                                            uint8_t rate)
{
    if (handle == NULL || !handle->is_initialized || rate > 3) {
        return HAL_ERROR;
    }

    handle->config.conversion_rate = rate;
    return TMP1075_WriteConfig(handle);
}

/**
  * @brief  Preveri če je device ready
  */
bool TMP1075_IsDeviceReady(I2C_HandleTypeDef* hi2c, uint8_t device_addr)
{
    if (hi2c == NULL) {
        return false;
    }

    return HAL_I2C_IsDeviceReady(hi2c, device_addr << 1, 3, TMP1075_I2C_TIMEOUT) == HAL_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Write register
  */
static HAL_StatusTypeDef TMP1075_WriteRegister(TMP1075_HandleTypeDef* handle,
                                               uint8_t reg_addr,
                                               uint16_t value)
{
    uint8_t data[3];
    data[0] = reg_addr;
    data[1] = (value >> 8) & 0xFF;  // MSB
    data[2] = value & 0xFF;          // LSB

    return HAL_I2C_Master_Transmit(handle->hi2c, handle->device_address,
                                   data, 3, TMP1075_I2C_TIMEOUT);
}

/**
  * @brief  Read register
  */
static HAL_StatusTypeDef TMP1075_ReadRegister(TMP1075_HandleTypeDef* handle,
                                              uint8_t reg_addr,
                                              uint16_t* value)
{
    uint8_t data[2];

    // Write register address
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(handle->hi2c,
                                                       handle->device_address,
                                                       &reg_addr, 1,
                                                       TMP1075_I2C_TIMEOUT);
    if (status != HAL_OK) return status;

    // Read register value
    status = HAL_I2C_Master_Receive(handle->hi2c, handle->device_address,
                                    data, 2, TMP1075_I2C_TIMEOUT);
    if (status == HAL_OK) {
        *value = (data[0] << 8) | data[1];
    }

    return status;
}

/**
  * @brief  Convert raw value to temperature (°C)
  */
static float TMP1075_RawToTemperature(uint16_t raw_value)
{
    // TMP1075 uses 12-bit resolution in 16-bit register (4 LSBs are 0)
    // Temperature = raw_value / 256 * 0.0625 = raw_value / 16
    int16_t temp_raw = (int16_t)raw_value >> 4;  // Right shift to get 12-bit value
    return (float)temp_raw * TMP1075_TEMP_RESOLUTION;
}

/**
  * @brief  Convert temperature (°C) to raw value
  */
static uint16_t TMP1075_TemperatureToRaw(float temperature)
{
    int16_t temp_raw = (int16_t)(temperature / TMP1075_TEMP_RESOLUTION);
    return (uint16_t)(temp_raw << 4);  // Left shift to align with register format
}
