/**
  ******************************************************************************
  * @file           : cy15b256j.h
  * @brief          : CY15B256J FRAM Driver Header
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  * @attention
  *
  * Driver za Cypress/Infineon CY15B256J I2C FRAM
  * - 256Kbit (32K × 8) FRAM
  * - I2C interface
  * - Write protect support
  *
  ******************************************************************************
  */

#ifndef __CY15B256J_H
#define __CY15B256J_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  CY15B256J device handle
  */
typedef struct {
    I2C_HandleTypeDef* hi2c;     // I2C handle
    uint8_t device_address;      // 7-bit I2C address (0x50-0x57)
    GPIO_TypeDef* wp_port;       // Write protect GPIO port (optional)
    uint16_t wp_pin;             // Write protect GPIO pin (optional)
    bool has_wp_pin;             // Write protect pin enabled
    bool is_initialized;         // Initialization flag
} CY15B256J_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/

// Default I2C address (A0=A1=A2=GND)
#define CY15B256J_I2C_ADDR_DEFAULT  0x50

// I2C address range (based on A2, A1, A0 pins)
#define CY15B256J_I2C_ADDR_MIN      0x50
#define CY15B256J_I2C_ADDR_MAX      0x57

// FRAM size
#define CY15B256J_SIZE_BYTES        32768   // 32KB
#define CY15B256J_PAGE_SIZE         32      // No page boundary restrictions

// I2C timeout
#define CY15B256J_I2C_TIMEOUT       100     // ms

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Inicializira CY15B256J FRAM
  * @param  handle: Pointer na CY15B256J_HandleTypeDef
  * @param  hi2c: Pointer na I2C handle
  * @param  device_addr: I2C address (0x50-0x57)
  * @param  wp_port: Write protect GPIO port (NULL če ni)
  * @param  wp_pin: Write protect GPIO pin
  * @retval HAL status
  */
HAL_StatusTypeDef CY15B256J_Init(CY15B256J_HandleTypeDef* handle,
                                 I2C_HandleTypeDef* hi2c,
                                 uint8_t device_addr,
                                 GPIO_TypeDef* wp_port,
                                 uint16_t wp_pin);

/**
  * @brief  Zapiši byte na memory address
  * @param  handle: Pointer na CY15B256J_HandleTypeDef
  * @param  mem_address: Memory address (0-32767)
  * @param  data: Byte za pisanje
  * @retval HAL status
  */
HAL_StatusTypeDef CY15B256J_WriteByte(CY15B256J_HandleTypeDef* handle,
                                      uint16_t mem_address,
                                      uint8_t data);

/**
  * @brief  Preberi byte z memory address
  * @param  handle: Pointer na CY15B256J_HandleTypeDef
  * @param  mem_address: Memory address (0-32767)
  * @param  data: Pointer na byte za branje
  * @retval HAL status
  */
HAL_StatusTypeDef CY15B256J_ReadByte(CY15B256J_HandleTypeDef* handle,
                                     uint16_t mem_address,
                                     uint8_t* data);

/**
  * @brief  Zapiši buffer na memory address
  * @param  handle: Pointer na CY15B256J_HandleTypeDef
  * @param  mem_address: Memory address (0-32767)
  * @param  data: Pointer na data buffer
  * @param  size: Število bytov za pisanje
  * @retval HAL status
  */
HAL_StatusTypeDef CY15B256J_Write(CY15B256J_HandleTypeDef* handle,
                                  uint16_t mem_address,
                                  uint8_t* data,
                                  uint16_t size);

/**
  * @brief  Preberi buffer z memory address
  * @param  handle: Pointer na CY15B256J_HandleTypeDef
  * @param  mem_address: Memory address (0-32767)
  * @param  data: Pointer na data buffer
  * @param  size: Število bytov za branje
  * @retval HAL status
  */
HAL_StatusTypeDef CY15B256J_Read(CY15B256J_HandleTypeDef* handle,
                                 uint16_t mem_address,
                                 uint8_t* data,
                                 uint16_t size);

/**
  * @brief  Napolni memory z določeno vrednostjo
  * @param  handle: Pointer na CY15B256J_HandleTypeDef
  * @param  mem_address: Start memory address
  * @param  value: Byte vrednost za fill
  * @param  size: Število bytov
  * @retval HAL status
  */
HAL_StatusTypeDef CY15B256J_Fill(CY15B256J_HandleTypeDef* handle,
                                 uint16_t mem_address,
                                 uint8_t value,
                                 uint16_t size);

/**
  * @brief  Nastavi write protect
  * @param  handle: Pointer na CY15B256J_HandleTypeDef
  * @param  protect: true = protect (WP = LOW), false = unprotect (WP = HIGH)
  * @retval HAL status
  */
HAL_StatusTypeDef CY15B256J_SetWriteProtect(CY15B256J_HandleTypeDef* handle,
                                            bool protect);

/**
  * @brief  Preveri če je FRAM prisoten na I2C
  * @param  hi2c: Pointer na I2C handle
  * @param  device_addr: I2C address
  * @retval true = device detected, false = not detected
  */
bool CY15B256J_IsDeviceReady(I2C_HandleTypeDef* hi2c, uint8_t device_addr);

#ifdef __cplusplus
}
#endif

#endif /* __CY15B256J_H */
