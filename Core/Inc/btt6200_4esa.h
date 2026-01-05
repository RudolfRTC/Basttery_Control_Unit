/**
  ******************************************************************************
  * @file           : btt6200_4esa.h
  * @brief          : BTT6200-4ESA Driver Library Header
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  * @attention
  *
  * Driver za Infineon BTT6200-4ESA quad-channel smart high-side switch
  * - 4 kanale na čip (OUT0-OUT3)
  * - Current sensing diagnostika preko IS pina
  * - Overcurrent protection
  * - Diagnostic channel selection (DSEL0, DSEL1)
  *
  ******************************************************************************
  */

#ifndef __BTT6200_4ESA_H
#define __BTT6200_4ESA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Kanal ID (0-3)
  */
typedef enum {
    BTT6200_CH0 = 0,
    BTT6200_CH1 = 1,
    BTT6200_CH2 = 2,
    BTT6200_CH3 = 3
} BTT6200_Channel_t;

/**
  * @brief  Status kanala
  */
typedef enum {
    BTT6200_STATUS_OK        = 0x00,  // Vse OK
    BTT6200_STATUS_OVERCURRENT = 0x01,  // Overcurrent detection
    BTT6200_STATUS_DISABLED    = 0x02,  // Kanal disabled
    BTT6200_STATUS_ERROR       = 0xFF   // Splošna napaka
} BTT6200_Status_t;

/**
  * @brief  GPIO pin configuration
  */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} BTT6200_GPIO_t;

/**
  * @brief  BTT6200-4ESA module handle struktura
  */
typedef struct {
    // Output control pins (IN0-IN3)
    BTT6200_GPIO_t out[4];

    // Diagnostic control pins
    BTT6200_GPIO_t den;      // Diagnostic Enable
    BTT6200_GPIO_t dsel0;    // Diagnostic Select bit 0
    BTT6200_GPIO_t dsel1;    // Diagnostic Select bit 1

    // Current sense ADC channel (opcijsko)
    ADC_HandleTypeDef* hadc;
    uint32_t is_adc_channel; // ADC channel za IS pin

    // Overcurrent interrupt pin (opcijsko)
    BTT6200_GPIO_t oc_pin;   // Overcurrent detection pin
    bool has_oc_pin;

    // Module ID
    uint8_t module_id;

    // Status array za vsak kanal
    bool channel_enabled[4];

} BTT6200_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/

// Diagnostic channel selection mapping
#define BTT6200_DSEL_CH0    0x00  // DSEL1=0, DSEL0=0
#define BTT6200_DSEL_CH1    0x01  // DSEL1=0, DSEL0=1
#define BTT6200_DSEL_CH2    0x02  // DSEL1=1, DSEL0=0
#define BTT6200_DSEL_CH3    0x03  // DSEL1=1, DSEL0=1

// Current sense ratio (tipična vrednost iz datasheet-a)
// IL = IIS * kILIS (kjer je kILIS tipično 1000-2000)
#define BTT6200_CURRENT_SENSE_RATIO  1400  // Tipična vrednost

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Inicializira BTT6200-4ESA modul
  * @param  handle: Pointer na BTT6200_HandleTypeDef strukturo
  * @retval HAL status
  */
HAL_StatusTypeDef BTT6200_Init(BTT6200_HandleTypeDef* handle);

/**
  * @brief  Omogoči/onemogoči izhodni kanal
  * @param  handle: Pointer na BTT6200_HandleTypeDef strukturo
  * @param  channel: Kanal (BTT6200_CH0 do BTT6200_CH3)
  * @param  enable: true = omogoči, false = onemogoči
  * @retval HAL status
  */
HAL_StatusTypeDef BTT6200_SetChannel(BTT6200_HandleTypeDef* handle,
                                     BTT6200_Channel_t channel,
                                     bool enable);

/**
  * @brief  Omogoči izhodni kanal (ON)
  * @param  handle: Pointer na BTT6200_HandleTypeDef strukturo
  * @param  channel: Kanal (BTT6200_CH0 do BTT6200_CH3)
  * @retval HAL status
  */
HAL_StatusTypeDef BTT6200_ChannelOn(BTT6200_HandleTypeDef* handle,
                                    BTT6200_Channel_t channel);

/**
  * @brief  Onemogoči izhodni kanal (OFF)
  * @param  handle: Pointer na BTT6200_HandleTypeDef strukturo
  * @param  channel: Kanal (BTT6200_CH0 do BTT6200_CH3)
  * @retval HAL status
  */
HAL_StatusTypeDef BTT6200_ChannelOff(BTT6200_HandleTypeDef* handle,
                                     BTT6200_Channel_t channel);

/**
  * @brief  Omogoči diagnostiko za izbrani kanal
  * @param  handle: Pointer na BTT6200_HandleTypeDef strukturo
  * @param  channel: Kanal za diagnostiko (BTT6200_CH0 do BTT6200_CH3)
  * @retval HAL status
  */
HAL_StatusTypeDef BTT6200_SelectDiagnosticChannel(BTT6200_HandleTypeDef* handle,
                                                   BTT6200_Channel_t channel);

/**
  * @brief  Omogoči/onemogoči diagnostiko
  * @param  handle: Pointer na BTT6200_HandleTypeDef strukturo
  * @param  enable: true = omogoči, false = onemogoči
  * @retval HAL status
  */
HAL_StatusTypeDef BTT6200_EnableDiagnostic(BTT6200_HandleTypeDef* handle,
                                           bool enable);

/**
  * @brief  Preberi trenutni tok preko IS pina (current sensing)
  * @param  handle: Pointer na BTT6200_HandleTypeDef strukturo
  * @param  channel: Kanal za branje (BTT6200_CH0 do BTT6200_CH3)
  * @param  current_mA: Pointer na spremenljivko za rezultat v mA
  * @retval HAL status
  * @note   Zahteva konfigurirano ADC in is_adc_channel v handle
  */
HAL_StatusTypeDef BTT6200_ReadChannelCurrent(BTT6200_HandleTypeDef* handle,
                                             BTT6200_Channel_t channel,
                                             uint32_t* current_mA);

/**
  * @brief  Preveri overcurrent status (preko OC pina)
  * @param  handle: Pointer na BTT6200_HandleTypeDef strukturo
  * @retval true = overcurrent detektiran, false = OK
  * @note   Zahteva konfigurirano oc_pin v handle
  */
bool BTT6200_IsOvercurrent(BTT6200_HandleTypeDef* handle);

/**
  * @brief  Pridobi status kanala
  * @param  handle: Pointer na BTT6200_HandleTypeDef strukturo
  * @param  channel: Kanal (BTT6200_CH0 do BTT6200_CH3)
  * @retval BTT6200_Status_t status
  */
BTT6200_Status_t BTT6200_GetChannelStatus(BTT6200_HandleTypeDef* handle,
                                          BTT6200_Channel_t channel);

/**
  * @brief  Onemogoči vse kanale na modulu
  * @param  handle: Pointer na BTT6200_HandleTypeDef strukturo
  * @retval HAL status
  */
HAL_StatusTypeDef BTT6200_DisableAll(BTT6200_HandleTypeDef* handle);

/**
  * @brief  Omogoči vse kanale na modulu
  * @param  handle: Pointer na BTT6200_HandleTypeDef strukturo
  * @retval HAL status
  */
HAL_StatusTypeDef BTT6200_EnableAll(BTT6200_HandleTypeDef* handle);

#ifdef __cplusplus
}
#endif

#endif /* __BTT6200_4ESA_H */
