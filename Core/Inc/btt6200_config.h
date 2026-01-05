/**
  ******************************************************************************
  * @file           : btt6200_config.h
  * @brief          : BTT6200-4ESA Configuration for BMU
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  * @attention
  *
  * Konfiguracija za 6 BTT6200-4ESA modulov (24 kanalov, uporabljamo 20)
  *
  * Module 0: OUT0_0 do OUT3_0
  * Module 1: OUT0_1 do OUT3_1
  * Module 2: OUT0_2 do OUT3_2
  * Module 3: OUT0_3 do OUT3_3
  * Module 4: OUT0_4 do OUT3_4
  * Module 5: OUT0_5 do OUT3_5 (samo 2 kanala uporabljena)
  *
  ******************************************************************************
  */

#ifndef __BTT6200_CONFIG_H
#define __BTT6200_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "btt6200_4esa.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  BMU Output ID (0-19)
  */
typedef enum {
    BMU_OUT0_0 = 0,   // Module 0, Channel 0
    BMU_OUT1_0 = 1,   // Module 0, Channel 1
    BMU_OUT2_0 = 2,   // Module 0, Channel 2
    BMU_OUT3_0 = 3,   // Module 0, Channel 3
    BMU_OUT0_1 = 4,   // Module 1, Channel 0
    BMU_OUT1_1 = 5,   // Module 1, Channel 1
    BMU_OUT2_1 = 6,   // Module 1, Channel 2
    BMU_OUT3_1 = 7,   // Module 1, Channel 3
    BMU_OUT0_2 = 8,   // Module 2, Channel 0
    BMU_OUT1_2 = 9,   // Module 2, Channel 1
    BMU_OUT2_2 = 10,  // Module 2, Channel 2
    BMU_OUT3_2 = 11,  // Module 2, Channel 3
    BMU_OUT0_3 = 12,  // Module 3, Channel 0
    BMU_OUT1_3 = 13,  // Module 3, Channel 1
    BMU_OUT2_3 = 14,  // Module 3, Channel 2
    BMU_OUT3_3 = 15,  // Module 3, Channel 3
    BMU_OUT0_4 = 16,  // Module 4, Channel 0
    BMU_OUT1_4 = 17,  // Module 4, Channel 1
    BMU_OUT2_4 = 18,  // Module 4, Channel 2
    BMU_OUT3_4 = 19,  // Module 4, Channel 3
    BMU_OUT_MAX = 20
} BMU_Output_t;

/* Exported constants --------------------------------------------------------*/

#define BTT6200_NUM_MODULES  6
#define BTT6200_NUM_OUTPUTS  20

/* Exported variables --------------------------------------------------------*/

extern BTT6200_HandleTypeDef btt6200_modules[BTT6200_NUM_MODULES];

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Inicializira vse BTT6200-4ESA module
  * @param  hadc: Pointer na ADC handle za current sensing (opcijsko)
  * @retval HAL status
  */
HAL_StatusTypeDef BTT6200_Config_Init(ADC_HandleTypeDef* hadc);

/**
  * @brief  Omogoči/onemogoči BMU output
  * @param  output: BMU output ID (BMU_OUT0_0 do BMU_OUT3_4)
  * @param  enable: true = ON, false = OFF
  * @retval HAL status
  */
HAL_StatusTypeDef BTT6200_Config_SetOutput(BMU_Output_t output, bool enable);

/**
  * @brief  Preberi tok na BMU outputu
  * @param  output: BMU output ID
  * @param  current_mA: Pointer na spremenljivko za rezultat
  * @retval HAL status
  */
HAL_StatusTypeDef BTT6200_Config_ReadCurrent(BMU_Output_t output, uint32_t* current_mA);

/**
  * @brief  Onemogoči vse outpute
  * @retval HAL status
  */
HAL_StatusTypeDef BTT6200_Config_DisableAll(void);

/**
  * @brief  Pridobi status outputa
  * @param  output: BMU output ID
  * @retval BTT6200_Status_t
  */
BTT6200_Status_t BTT6200_Config_GetStatus(BMU_Output_t output);

#ifdef __cplusplus
}
#endif

#endif /* __BTT6200_CONFIG_H */
