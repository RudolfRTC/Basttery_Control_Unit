/**
  ******************************************************************************
  * @file           : btt6200_config.c
  * @brief          : BTT6200-4ESA Configuration Implementation
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "btt6200_config.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

typedef struct {
    uint8_t module;
    BTT6200_Channel_t channel;
} OutputMapping_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// BTT6200 module handles (6 modulov)
BTT6200_HandleTypeDef btt6200_modules[BTT6200_NUM_MODULES];

// Mapping: BMU output -> (module, channel)
static const OutputMapping_t output_mapping[BTT6200_NUM_OUTPUTS] = {
    {0, BTT6200_CH0},  // BMU_OUT0_0
    {0, BTT6200_CH1},  // BMU_OUT1_0
    {0, BTT6200_CH2},  // BMU_OUT2_0
    {0, BTT6200_CH3},  // BMU_OUT3_0
    {1, BTT6200_CH0},  // BMU_OUT0_1
    {1, BTT6200_CH1},  // BMU_OUT1_1
    {1, BTT6200_CH2},  // BMU_OUT2_1
    {1, BTT6200_CH3},  // BMU_OUT3_1
    {2, BTT6200_CH0},  // BMU_OUT0_2
    {2, BTT6200_CH1},  // BMU_OUT1_2
    {2, BTT6200_CH2},  // BMU_OUT2_2
    {2, BTT6200_CH3},  // BMU_OUT3_2
    {3, BTT6200_CH0},  // BMU_OUT0_3
    {3, BTT6200_CH1},  // BMU_OUT1_3
    {3, BTT6200_CH2},  // BMU_OUT2_3
    {3, BTT6200_CH3},  // BMU_OUT3_3
    {4, BTT6200_CH0},  // BMU_OUT0_4
    {4, BTT6200_CH1},  // BMU_OUT1_4
    {4, BTT6200_CH2},  // BMU_OUT2_4
    {4, BTT6200_CH3},  // BMU_OUT3_4
};

/* Private function prototypes -----------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Inicializira vse BTT6200-4ESA module
  */
HAL_StatusTypeDef BTT6200_Config_Init(ADC_HandleTypeDef* hadc)
{
    HAL_StatusTypeDef status = HAL_OK;

    // ========== MODULE 0 ==========
    btt6200_modules[0].module_id = 0;
    btt6200_modules[0].out[0] = (BTT6200_GPIO_t){OUT0_0_GPIO_Port, OUT0_0_Pin};  // PB10
    btt6200_modules[0].out[1] = (BTT6200_GPIO_t){OUT1_0_GPIO_Port, OUT1_0_Pin};  // PE15
    btt6200_modules[0].out[2] = (BTT6200_GPIO_t){OUT2_0_GPIO_Port, OUT2_0_Pin};  // PE12
    btt6200_modules[0].out[3] = (BTT6200_GPIO_t){OUT3_0_GPIO_Port, OUT3_0_Pin};  // PB11
    btt6200_modules[0].den = (BTT6200_GPIO_t){DEN_0_GPIO_Port, DEN_0_Pin};       // PE14
    btt6200_modules[0].dsel0 = (BTT6200_GPIO_t){DSEL0_0_GPIO_Port, DSEL0_0_Pin}; // PE13
    btt6200_modules[0].dsel1 = (BTT6200_GPIO_t){DSEL1_0_GPIO_Port, DSEL1_0_Pin}; // PE10
    btt6200_modules[0].hadc = hadc;
    btt6200_modules[0].is_adc_channel = ADC_CHANNEL_10; // PC0


    // ========== MODULE 1 ==========
    btt6200_modules[1].module_id = 1;
    btt6200_modules[1].out[0] = (BTT6200_GPIO_t){OUT0_1_GPIO_Port, OUT0_1_Pin};  // PD13
    btt6200_modules[1].out[1] = (BTT6200_GPIO_t){OUT1_1_GPIO_Port, OUT1_1_Pin};  // PD12
    btt6200_modules[1].out[2] = (BTT6200_GPIO_t){OUT2_1_GPIO_Port, OUT2_1_Pin};  // PD9
    btt6200_modules[1].out[3] = (BTT6200_GPIO_t){OUT3_1_GPIO_Port, OUT3_1_Pin};  // PD8
    btt6200_modules[1].den = (BTT6200_GPIO_t){DEN_1_GPIO_Port, DEN_1_Pin};       // PB11
    btt6200_modules[1].dsel0 = (BTT6200_GPIO_t){DSEL0_1_GPIO_Port, DSEL0_1_Pin}; // PD10
    btt6200_modules[1].dsel1 = (BTT6200_GPIO_t){DSEL1_1_GPIO_Port, DSEL1_1_Pin}; // PB15
    btt6200_modules[1].hadc = hadc;
    btt6200_modules[1].is_adc_channel = ADC_CHANNEL_11; // PC1 - FIXED: was [0]

    // ========== MODULE 2 ==========
    btt6200_modules[2].module_id = 2;
    btt6200_modules[2].out[0] = (BTT6200_GPIO_t){OUT0_2_GPIO_Port, OUT0_2_Pin};  // PG6
    btt6200_modules[2].out[1] = (BTT6200_GPIO_t){OUT1_2_GPIO_Port, OUT1_2_Pin};  // PG5
    btt6200_modules[2].out[2] = (BTT6200_GPIO_t){OUT2_2_GPIO_Port, OUT2_2_Pin};  // PG2
    btt6200_modules[2].out[3] = (BTT6200_GPIO_t){OUT3_2_GPIO_Port, OUT3_2_Pin};  // PD15
    btt6200_modules[2].den = (BTT6200_GPIO_t){DEN_2_GPIO_Port, DEN_2_Pin};       // PG4
    btt6200_modules[2].dsel0 = (BTT6200_GPIO_t){DSEL0_2_GPIO_Port, DSEL0_2_Pin}; // PG3
    btt6200_modules[2].dsel1 = (BTT6200_GPIO_t){DSEL1_2_GPIO_Port, DSEL1_2_Pin}; // PD14
    btt6200_modules[2].hadc = hadc;
    btt6200_modules[2].is_adc_channel = ADC_CHANNEL_12; // PC2 - FIXED: was [0]

    // ========== MODULE 3 ==========
    btt6200_modules[3].module_id = 3;
    btt6200_modules[3].out[0] = (BTT6200_GPIO_t){OUT0_3_GPIO_Port, OUT0_3_Pin};  // PA8
    btt6200_modules[3].out[1] = (BTT6200_GPIO_t){OUT1_3_GPIO_Port, OUT1_3_Pin};  // PA9
    btt6200_modules[3].out[2] = (BTT6200_GPIO_t){OUT2_3_GPIO_Port, OUT2_3_Pin};  // PA10
    btt6200_modules[3].out[3] = (BTT6200_GPIO_t){OUT3_3_GPIO_Port, OUT3_3_Pin};  // PA11
    btt6200_modules[3].den = (BTT6200_GPIO_t){DEN_3_GPIO_Port, DEN_3_Pin};       // PC7
    btt6200_modules[3].dsel0 = (BTT6200_GPIO_t){DSEL0_3_GPIO_Port, DSEL0_3_Pin}; // PC8
    btt6200_modules[3].dsel1 = (BTT6200_GPIO_t){DSEL1_3_GPIO_Port, DSEL1_3_Pin}; // PC9
    btt6200_modules[3].hadc = hadc;
    btt6200_modules[3].is_adc_channel = ADC_CHANNEL_13; // PC3 - FIXED: was [0]

    // ========== MODULE 4 ==========
    btt6200_modules[4].module_id = 4;
    btt6200_modules[4].out[0] = (BTT6200_GPIO_t){OUT0_4_GPIO_Port, OUT0_4_Pin};  // PA15
    btt6200_modules[4].out[1] = (BTT6200_GPIO_t){OUT1_4_GPIO_Port, OUT1_4_Pin};  // PC10
    btt6200_modules[4].out[2] = (BTT6200_GPIO_t){OUT2_4_GPIO_Port, OUT2_4_Pin};  // PD0
    btt6200_modules[4].out[3] = (BTT6200_GPIO_t){OUT3_4_GPIO_Port, OUT3_4_Pin};  // PD1
    btt6200_modules[4].den = (BTT6200_GPIO_t){DEN_4_GPIO_Port, DEN_4_Pin};       // PC11
    btt6200_modules[4].dsel0 = (BTT6200_GPIO_t){DSEL0_4_GPIO_Port, DSEL0_4_Pin}; // PC12
    btt6200_modules[4].dsel1 = (BTT6200_GPIO_t){DSEL1_4_GPIO_Port, DSEL1_4_Pin}; // PD2
    btt6200_modules[4].hadc = hadc;
    btt6200_modules[4].is_adc_channel = ADC_CHANNEL_14; // PC4 - FIXED: was [0]

    // Inicializiraj vse module
    for (uint8_t i = 0; i < BTT6200_NUM_MODULES; i++) {
        if (BTT6200_Init(&btt6200_modules[i]) != HAL_OK) {
            status = HAL_ERROR;
        }
    }

    return status;
}

/**
  * @brief  Omogoči/onemogoči BMU output
  */
HAL_StatusTypeDef BTT6200_Config_SetOutput(BMU_Output_t output, bool enable)
{
    if (output >= BMU_OUT_MAX) {
        return HAL_ERROR;
    }

    uint8_t module = output_mapping[output].module;
    BTT6200_Channel_t channel = output_mapping[output].channel;

    return BTT6200_SetChannel(&btt6200_modules[module], channel, enable);
}

/**
  * @brief  Preberi tok na BMU outputu
  */
HAL_StatusTypeDef BTT6200_Config_ReadCurrent(BMU_Output_t output, uint32_t* current_mA)
{
    if (output >= BMU_OUT_MAX || current_mA == NULL) {
        return HAL_ERROR;
    }

    uint8_t module = output_mapping[output].module;
    BTT6200_Channel_t channel = output_mapping[output].channel;

    return BTT6200_ReadChannelCurrent(&btt6200_modules[module], channel, current_mA);
}

/**
  * @brief  Onemogoči vse outpute
  */
HAL_StatusTypeDef BTT6200_Config_DisableAll(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    for (uint8_t i = 0; i < BTT6200_NUM_MODULES; i++) {
        if (BTT6200_DisableAll(&btt6200_modules[i]) != HAL_OK) {
            status = HAL_ERROR;
        }
    }

    return status;
}

/**
  * @brief  Pridobi status outputa
  */
BTT6200_Status_t BTT6200_Config_GetStatus(BMU_Output_t output)
{
    if (output >= BMU_OUT_MAX) {
        return BTT6200_STATUS_ERROR;
    }

    uint8_t module = output_mapping[output].module;
    BTT6200_Channel_t channel = output_mapping[output].channel;

    return BTT6200_GetChannelStatus(&btt6200_modules[module], channel);
}
