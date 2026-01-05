/**
  ******************************************************************************
  * @file           : cy15b256j.c
  * @brief          : CY15B256J FRAM Driver Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "cy15b256j.h"
#include <string.h>

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Inicializira CY15B256J FRAM
  */
HAL_StatusTypeDef CY15B256J_Init(CY15B256J_HandleTypeDef* handle,
                                 I2C_HandleTypeDef* hi2c,
                                 uint8_t device_addr,
                                 GPIO_TypeDef* wp_port,
                                 uint16_t wp_pin)
{
    if (handle == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }

    // Preveri I2C address range
    if (device_addr < CY15B256J_I2C_ADDR_MIN || device_addr > CY15B256J_I2C_ADDR_MAX) {
        return HAL_ERROR;
    }

    handle->hi2c = hi2c;
    handle->device_address = device_addr << 1;  // Convert to 8-bit address

    // Write protect pin setup
    if (wp_port != NULL) {
        handle->wp_port = wp_port;
        handle->wp_pin = wp_pin;
        handle->has_wp_pin = true;

        // Default: disable write protect (WP = HIGH)
        HAL_GPIO_WritePin(handle->wp_port, handle->wp_pin, GPIO_PIN_SET);
    } else {
        handle->has_wp_pin = false;
    }

    // Preveri če je device prisoten
    if (!CY15B256J_IsDeviceReady(hi2c, device_addr)) {
        return HAL_ERROR;
    }

    handle->is_initialized = true;

    return HAL_OK;
}

/**
  * @brief  Zapiši byte
  */
HAL_StatusTypeDef CY15B256J_WriteByte(CY15B256J_HandleTypeDef* handle,
                                      uint16_t mem_address,
                                      uint8_t data)
{
    return CY15B256J_Write(handle, mem_address, &data, 1);
}

/**
  * @brief  Preberi byte
  */
HAL_StatusTypeDef CY15B256J_ReadByte(CY15B256J_HandleTypeDef* handle,
                                     uint16_t mem_address,
                                     uint8_t* data)
{
    return CY15B256J_Read(handle, mem_address, data, 1);
}

/**
  * @brief  Zapiši buffer
  */
HAL_StatusTypeDef CY15B256J_Write(CY15B256J_HandleTypeDef* handle,
                                  uint16_t mem_address,
                                  uint8_t* data,
                                  uint16_t size)
{
    if (handle == NULL || data == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    // Preveri memory bounds
    if ((uint32_t)mem_address + size > CY15B256J_SIZE_BYTES) {
        return HAL_ERROR;
    }

    // HAL_I2C_Mem_Write uses 16-bit memory address
    return HAL_I2C_Mem_Write(handle->hi2c,
                            handle->device_address,
                            mem_address,
                            I2C_MEMADD_SIZE_16BIT,
                            data,
                            size,
                            CY15B256J_I2C_TIMEOUT);
}

/**
  * @brief  Preberi buffer
  */
HAL_StatusTypeDef CY15B256J_Read(CY15B256J_HandleTypeDef* handle,
                                 uint16_t mem_address,
                                 uint8_t* data,
                                 uint16_t size)
{
    if (handle == NULL || data == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    // Preveri memory bounds
    if ((uint32_t)mem_address + size > CY15B256J_SIZE_BYTES) {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(handle->hi2c,
                           handle->device_address,
                           mem_address,
                           I2C_MEMADD_SIZE_16BIT,
                           data,
                           size,
                           CY15B256J_I2C_TIMEOUT);
}

/**
  * @brief  Napolni memory z vrednostjo
  */
HAL_StatusTypeDef CY15B256J_Fill(CY15B256J_HandleTypeDef* handle,
                                 uint16_t mem_address,
                                 uint8_t value,
                                 uint16_t size)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    // Preveri memory bounds
    if ((uint32_t)mem_address + size > CY15B256J_SIZE_BYTES) {
        return HAL_ERROR;
    }

    // Fill v chunkih (da ne presežemo stack size)
    #define FILL_CHUNK_SIZE 128
    uint8_t buffer[FILL_CHUNK_SIZE];
    memset(buffer, value, FILL_CHUNK_SIZE);

    uint16_t remaining = size;
    uint16_t current_address = mem_address;

    while (remaining > 0) {
        uint16_t chunk_size = (remaining > FILL_CHUNK_SIZE) ? FILL_CHUNK_SIZE : remaining;

        HAL_StatusTypeDef status = CY15B256J_Write(handle, current_address, buffer, chunk_size);
        if (status != HAL_OK) {
            return status;
        }

        current_address += chunk_size;
        remaining -= chunk_size;
    }

    return HAL_OK;
}

/**
  * @brief  Nastavi write protect
  */
HAL_StatusTypeDef CY15B256J_SetWriteProtect(CY15B256J_HandleTypeDef* handle,
                                            bool protect)
{
    if (handle == NULL || !handle->is_initialized || !handle->has_wp_pin) {
        return HAL_ERROR;
    }

    // WP pin: LOW = protected, HIGH = unprotected
    GPIO_PinState state = protect ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(handle->wp_port, handle->wp_pin, state);

    return HAL_OK;
}

/**
  * @brief  Preveri če je device ready
  */
bool CY15B256J_IsDeviceReady(I2C_HandleTypeDef* hi2c, uint8_t device_addr)
{
    if (hi2c == NULL) {
        return false;
    }

    return HAL_I2C_IsDeviceReady(hi2c, device_addr << 1, 3, CY15B256J_I2C_TIMEOUT) == HAL_OK;
}
