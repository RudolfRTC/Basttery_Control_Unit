/**
  ******************************************************************************
  * @file           : bmu_can.c
  * @brief          : BMU CAN Protocol Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bmu_can.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef BMU_CAN_SendMessage(BMU_CAN_HandleTypeDef* handle,
                                             uint32_t msg_id,
                                             uint8_t* data,
                                             uint8_t dlc);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Inicializira BMU CAN protokol
  */
HAL_StatusTypeDef BMU_CAN_Init(BMU_CAN_HandleTypeDef* handle,
                               CAN_HandleTypeDef* hcan1,
                               CAN_HandleTypeDef* hcan2)
{
    if (handle == NULL || hcan1 == NULL) {
        return HAL_ERROR;
    }

    memset(handle, 0, sizeof(BMU_CAN_HandleTypeDef));

    handle->hcan1 = hcan1;
    handle->hcan2 = hcan2;

    // Konfiguriraj CAN1 na 500 kbps
    if (BMU_CAN_Configure500k(hcan1) != HAL_OK) {
        return HAL_ERROR;
    }

    // Konfiguriraj filter
    if (BMU_CAN_ConfigureFilter(hcan1) != HAL_OK) {
        return HAL_ERROR;
    }

    // Zaženi CAN1
    if (HAL_CAN_Start(hcan1) != HAL_OK) {
        return HAL_ERROR;
    }

    // Če je CAN2 prisoten, ga tudi konfiguriraj
    if (hcan2 != NULL) {
        if (BMU_CAN_Configure500k(hcan2) != HAL_OK) {
            return HAL_ERROR;
        }
        if (BMU_CAN_ConfigureFilter(hcan2) != HAL_OK) {
            return HAL_ERROR;
        }
        if (HAL_CAN_Start(hcan2) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    handle->is_initialized = true;

    return HAL_OK;
}

/**
  * @brief  Konfiguriraj CAN na 500 kbps
  * @note   APB1 Clock = 16 MHz (HSI)
  *         Baudrate = APB1_Clock / (Prescaler * (1 + BS1 + BS2))
  *         500 kbps = 16MHz / (2 * (1 + 13 + 2)) = 16MHz / 32 = 500 kHz
  */
HAL_StatusTypeDef BMU_CAN_Configure500k(CAN_HandleTypeDef* hcan)
{
    if (hcan == NULL) {
        return HAL_ERROR;
    }

    // Stop CAN if running
    HAL_CAN_Stop(hcan);

    // Deinitialize
    HAL_CAN_DeInit(hcan);

    // Reconfigure for 500 kbps
    hcan->Init.Prescaler = 2;                       // Prescaler = 2
    hcan->Init.Mode = CAN_MODE_NORMAL;
    hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan->Init.TimeSeg1 = CAN_BS1_13TQ;             // BS1 = 13 TQ
    hcan->Init.TimeSeg2 = CAN_BS2_2TQ;              // BS2 = 2 TQ
    hcan->Init.TimeTriggeredMode = DISABLE;
    hcan->Init.AutoBusOff = ENABLE;                 // Auto recovery
    hcan->Init.AutoWakeUp = DISABLE;
    hcan->Init.AutoRetransmission = ENABLE;
    hcan->Init.ReceiveFifoLocked = DISABLE;
    hcan->Init.TransmitFifoPriority = DISABLE;

    // Initialize CAN
    if (HAL_CAN_Init(hcan) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief  Nastavi CAN filter (accept all messages)
  */
HAL_StatusTypeDef BMU_CAN_ConfigureFilter(CAN_HandleTypeDef* hcan)
{
    if (hcan == NULL) {
        return HAL_ERROR;
    }

    CAN_FilterTypeDef filter;

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief  Pošlji BMU Status sporočilo (0x100)
  */
HAL_StatusTypeDef BMU_CAN_SendStatus(BMU_CAN_HandleTypeDef* handle,
                                     BMU_Status_Msg_t* msg)
{
    if (handle == NULL || msg == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, CAN_ID_BMU_STATUS,
                               (uint8_t*)msg, sizeof(BMU_Status_Msg_t));
}

/**
  * @brief  Pošlji Temperature sporočilo (0x101)
  */
HAL_StatusTypeDef BMU_CAN_SendTemperature(BMU_CAN_HandleTypeDef* handle,
                                          BMU_Temperature_Msg_t* msg)
{
    if (handle == NULL || msg == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, CAN_ID_TEMPERATURE,
                               (uint8_t*)msg, sizeof(BMU_Temperature_Msg_t));
}

/**
  * @brief  Pošlji LEM Current sporočilo (0x110-0x112)
  */
HAL_StatusTypeDef BMU_CAN_SendLEMCurrent(BMU_CAN_HandleTypeDef* handle,
                                         uint32_t msg_id,
                                         BMU_LEM_Current_Msg_t* msg)
{
    if (handle == NULL || msg == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, msg_id,
                               (uint8_t*)msg, sizeof(BMU_LEM_Current_Msg_t));
}

/**
  * @brief  Pošlji BTT6200 Status sporočilo (0x120-0x122)
  */
HAL_StatusTypeDef BMU_CAN_SendBTTStatus(BMU_CAN_HandleTypeDef* handle,
                                        uint32_t msg_id,
                                        BMU_BTT6200_Status_Msg_t* msg)
{
    if (handle == NULL || msg == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, msg_id,
                               (uint8_t*)msg, sizeof(BMU_BTT6200_Status_Msg_t));
}

/**
  * @brief  Pošlji FRAM Statistics sporočilo (0x130)
  */
HAL_StatusTypeDef BMU_CAN_SendFRAMStats(BMU_CAN_HandleTypeDef* handle,
                                       BMU_FRAM_Stats_Msg_t* msg)
{
    if (handle == NULL || msg == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, CAN_ID_FRAM_STATS,
                               (uint8_t*)msg, sizeof(BMU_FRAM_Stats_Msg_t));
}

/**
  * @brief  Pošlji Alarms sporočilo (0x140)
  */
HAL_StatusTypeDef BMU_CAN_SendAlarms(BMU_CAN_HandleTypeDef* handle,
                                     BMU_Alarms_Msg_t* msg)
{
    if (handle == NULL || msg == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, CAN_ID_ALARMS,
                               (uint8_t*)msg, sizeof(BMU_Alarms_Msg_t));
}

/**
  * @brief  Pošlji Heartbeat sporočilo (0x1FF)
  */
HAL_StatusTypeDef BMU_CAN_SendHeartbeat(BMU_CAN_HandleTypeDef* handle,
                                        uint32_t counter)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    uint8_t data[8] = {0};
    data[0] = (counter >> 24) & 0xFF;
    data[1] = (counter >> 16) & 0xFF;
    data[2] = (counter >> 8) & 0xFF;
    data[3] = counter & 0xFF;
    data[4] = 0xBE;  // Magic bytes
    data[5] = 0xEF;

    return BMU_CAN_SendMessage(handle, CAN_ID_HEARTBEAT, data, 8);
}

/**
  * @brief  Pridobi statistiko CAN komunikacije
  */
HAL_StatusTypeDef BMU_CAN_GetStats(BMU_CAN_HandleTypeDef* handle,
                                  uint32_t* tx_count,
                                  uint32_t* rx_count,
                                  uint32_t* error_count)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    if (tx_count != NULL) {
        *tx_count = handle->tx_count;
    }
    if (rx_count != NULL) {
        *rx_count = handle->rx_count;
    }
    if (error_count != NULL) {
        *error_count = handle->error_count;
    }

    return HAL_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Pošlji CAN sporočilo
  */
static HAL_StatusTypeDef BMU_CAN_SendMessage(BMU_CAN_HandleTypeDef* handle,
                                             uint32_t msg_id,
                                             uint8_t* data,
                                             uint8_t dlc)
{
    if (handle == NULL || data == NULL) {
        return HAL_ERROR;
    }

    // Ensure DLC is valid (0-8)
    if (dlc > 8) {
        dlc = 8;
    }

    // Configure TX header
    handle->tx_header.StdId = msg_id;
    handle->tx_header.IDE = CAN_ID_STD;  // Standard 11-bit ID
    handle->tx_header.RTR = CAN_RTR_DATA;
    handle->tx_header.DLC = dlc;
    handle->tx_header.TransmitGlobalTime = DISABLE;

    // Copy data to TX buffer
    memcpy(handle->tx_data, data, dlc);

    // Send message
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(handle->hcan1,
                                                    &handle->tx_header,
                                                    handle->tx_data,
                                                    &handle->tx_mailbox);

    if (status == HAL_OK) {
        handle->tx_count++;
    } else {
        handle->error_count++;
    }

    return status;
}
