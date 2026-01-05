/**
  ******************************************************************************
  * @file           : btt6200_4esa.c
  * @brief          : BTT6200-4ESA Driver Library Implementation
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  * @attention
  *
  * Driver za Infineon BTT6200-4ESA quad-channel smart high-side switch
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "btt6200_4esa.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static HAL_StatusTypeDef BTT6200_SetDiagnosticSelect(BTT6200_HandleTypeDef* handle,
                                                      uint8_t dsel_value);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Inicializira BTT6200-4ESA modul
  */
HAL_StatusTypeDef BTT6200_Init(BTT6200_HandleTypeDef* handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    // Inicializiraj vse kanale kot disabled
    for (uint8_t i = 0; i < 4; i++) {
        handle->channel_enabled[i] = false;
        HAL_GPIO_WritePin(handle->out[i].port, handle->out[i].pin, GPIO_PIN_RESET);
    }

    // Onemogoči diagnostiko na začetku
    HAL_GPIO_WritePin(handle->den.port, handle->den.pin, GPIO_PIN_RESET);

    // Nastavi diagnostic select na kanal 0
    BTT6200_SetDiagnosticSelect(handle, BTT6200_DSEL_CH0);

    return HAL_OK;
}

/**
  * @brief  Omogoči/onemogoči izhodni kanal
  */
HAL_StatusTypeDef BTT6200_SetChannel(BTT6200_HandleTypeDef* handle,
                                     BTT6200_Channel_t channel,
                                     bool enable)
{
    if (handle == NULL || channel > BTT6200_CH3) {
        return HAL_ERROR;
    }

    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(handle->out[channel].port,
                      handle->out[channel].pin,
                      state);

    handle->channel_enabled[channel] = enable;

    return HAL_OK;
}

/**
  * @brief  Omogoči izhodni kanal (ON)
  */
HAL_StatusTypeDef BTT6200_ChannelOn(BTT6200_HandleTypeDef* handle,
                                    BTT6200_Channel_t channel)
{
    return BTT6200_SetChannel(handle, channel, true);
}

/**
  * @brief  Onemogoči izhodni kanal (OFF)
  */
HAL_StatusTypeDef BTT6200_ChannelOff(BTT6200_HandleTypeDef* handle,
                                     BTT6200_Channel_t channel)
{
    return BTT6200_SetChannel(handle, channel, false);
}

/**
  * @brief  Omogoči diagnostiko za izbrani kanal
  */
HAL_StatusTypeDef BTT6200_SelectDiagnosticChannel(BTT6200_HandleTypeDef* handle,
                                                   BTT6200_Channel_t channel)
{
    if (handle == NULL || channel > BTT6200_CH3) {
        return HAL_ERROR;
    }

    uint8_t dsel_value = (uint8_t)channel;
    return BTT6200_SetDiagnosticSelect(handle, dsel_value);
}

/**
  * @brief  Omogoči/onemogoči diagnostiko
  */
HAL_StatusTypeDef BTT6200_EnableDiagnostic(BTT6200_HandleTypeDef* handle,
                                           bool enable)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(handle->den.port, handle->den.pin, state);

    return HAL_OK;
}

/**
  * @brief  Preberi trenutni tok preko IS pina (current sensing)
  */
HAL_StatusTypeDef BTT6200_ReadChannelCurrent(BTT6200_HandleTypeDef* handle,
                                             BTT6200_Channel_t channel,
                                             uint32_t* current_mA)
{
    if (handle == NULL || current_mA == NULL || channel > BTT6200_CH3) {
        return HAL_ERROR;
    }

    if (handle->hadc == NULL) {
        return HAL_ERROR;  // ADC ni konfiguriran
    }

    // Izberi diagnostični kanal
    if (BTT6200_SelectDiagnosticChannel(handle, channel) != HAL_OK) {
        return HAL_ERROR;
    }

    // Omogoči diagnostiko
    if (BTT6200_EnableDiagnostic(handle, true) != HAL_OK) {
        return HAL_ERROR;
    }

    // Počakaj krajši čas za stabilizacijo (priporočeno iz datasheeta)
    HAL_Delay(1);

    // Preberi ADC vrednost
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = handle->is_adc_channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if (HAL_ADC_ConfigChannel(handle->hadc, &sConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_ADC_Start(handle->hadc);
    if (HAL_ADC_PollForConversion(handle->hadc, 100) != HAL_OK) {
        HAL_ADC_Stop(handle->hadc);
        return HAL_ERROR;
    }

    uint32_t adc_value = HAL_ADC_GetValue(handle->hadc);
    HAL_ADC_Stop(handle->hadc);

    // Pretvori ADC vrednost v napetost (mV)
    // Assuming 3.3V reference, 12-bit ADC
    uint32_t voltage_mv = (adc_value * 3300) / 4095;

    // Pretvori napetost v tok
    // IS pin current: IIS = IL / kILIS
    // Voltage across sense resistor (usually Rsense = 1.2kOhm typical)
    // V_IS = IIS * Rsense
    // IL = (V_IS / Rsense) * kILIS

    // Poenostavljena formula (prilagodi glede na hardware):
    // Če uporabimo Rsense = 1.2kOhm in kILIS = 1400:
    // IL(mA) = V_IS(mV) / 1.2 * 1400 / 1000
    // IL(mA) = V_IS(mV) * 1.167

    // Za večjo natančnost uporabi dejanske vrednosti iz hardware sheme
    *current_mA = (voltage_mv * BTT6200_CURRENT_SENSE_RATIO) / 1200;

    return HAL_OK;
}

/**
  * @brief  Preveri overcurrent status (preko OC pina)
  */
bool BTT6200_IsOvercurrent(BTT6200_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->has_oc_pin) {
        return false;
    }

    // OC pin je active high (overcurrent = HIGH)
    GPIO_PinState state = HAL_GPIO_ReadPin(handle->oc_pin.port,
                                            handle->oc_pin.pin);

    return (state == GPIO_PIN_SET);
}

/**
  * @brief  Pridobi status kanala
  */
BTT6200_Status_t BTT6200_GetChannelStatus(BTT6200_HandleTypeDef* handle,
                                          BTT6200_Channel_t channel)
{
    if (handle == NULL || channel > BTT6200_CH3) {
        return BTT6200_STATUS_ERROR;
    }

    // Preveri overcurrent
    if (BTT6200_IsOvercurrent(handle)) {
        return BTT6200_STATUS_OVERCURRENT;
    }

    // Preveri, če je kanal omogočen
    if (!handle->channel_enabled[channel]) {
        return BTT6200_STATUS_DISABLED;
    }

    return BTT6200_STATUS_OK;
}

/**
  * @brief  Onemogoči vse kanale na modulu
  */
HAL_StatusTypeDef BTT6200_DisableAll(BTT6200_HandleTypeDef* handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    for (uint8_t i = 0; i < 4; i++) {
        BTT6200_ChannelOff(handle, (BTT6200_Channel_t)i);
    }

    return HAL_OK;
}

/**
  * @brief  Omogoči vse kanale na modulu
  */
HAL_StatusTypeDef BTT6200_EnableAll(BTT6200_HandleTypeDef* handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    for (uint8_t i = 0; i < 4; i++) {
        BTT6200_ChannelOn(handle, (BTT6200_Channel_t)i);
    }

    return HAL_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Nastavi diagnostic select pine (DSEL0, DSEL1)
  * @param  handle: Pointer na BTT6200_HandleTypeDef
  * @param  dsel_value: Vrednost 0-3 (2-bit)
  */
static HAL_StatusTypeDef BTT6200_SetDiagnosticSelect(BTT6200_HandleTypeDef* handle,
                                                      uint8_t dsel_value)
{
    if (handle == NULL || dsel_value > 3) {
        return HAL_ERROR;
    }

    // DSEL0 = bit 0
    GPIO_PinState dsel0_state = (dsel_value & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(handle->dsel0.port, handle->dsel0.pin, dsel0_state);

    // DSEL1 = bit 1
    GPIO_PinState dsel1_state = (dsel_value & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(handle->dsel1.port, handle->dsel1.pin, dsel1_state);

    return HAL_OK;
}
