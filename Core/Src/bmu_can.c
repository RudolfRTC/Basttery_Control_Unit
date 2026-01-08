/**
  ******************************************************************************
  * @file           : bmu_can.c
  * @brief          : BMU CAN Protocol Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bmu_can.h"
#include "btt6200_config.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define BMU_HEARTBEAT_MAGIC_BYTE1  0xBEU
#define BMU_HEARTBEAT_MAGIC_BYTE2  0xEFU

/* Private variables ---------------------------------------------------------*/
/* MISRA C 2012 Rule 8.9 deviation: Global pointer for interrupt context access */
static volatile BMU_CAN_HandleTypeDef* g_bmu_can_handle = NULL;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef BMU_CAN_SendMessage(BMU_CAN_HandleTypeDef* handle,
                                             uint32_t msg_id,
                                             uint8_t* data,
                                             uint8_t dlc);

static HAL_StatusTypeDef BMU_CAN_PerformRecovery(CAN_HandleTypeDef* hcan);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Inicializira BMU CAN protokol
  */
HAL_StatusTypeDef BMU_CAN_Init(BMU_CAN_HandleTypeDef* handle,
                               CAN_HandleTypeDef* hcan1,
                               CAN_HandleTypeDef* hcan2)
{
    /* MISRA C 2012 Rule 14.4: Use explicit NULL comparison */
    if ((handle == NULL) || (hcan1 == NULL)) {
        return HAL_ERROR;
    }

    (void)memset(handle, 0, sizeof(BMU_CAN_HandleTypeDef));

    handle->hcan1 = hcan1;
    handle->hcan2 = hcan2;

    // POMEMBNO: CAN je že inicializiran v MX_CAN1_Init()
    // Tukaj samo nastavimo filter in zaženemo CAN

    // Konfiguriraj filter za CAN1
    if (BMU_CAN_ConfigureFilter(hcan1) != HAL_OK) {
        return HAL_ERROR;
    }

    // Zaženi CAN1
    if (HAL_CAN_Start(hcan1) != HAL_OK) {
        return HAL_ERROR;
    }

    // Če je CAN2 prisoten, ga tudi konfiguriraj
    if (hcan2 != NULL) {
        if (BMU_CAN_ConfigureFilter(hcan2) != HAL_OK) {
            return HAL_ERROR;
        }
        if (HAL_CAN_Start(hcan2) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    handle->is_initialized = true;

    // Store global handle for RX callback (atomic operation to prevent race condition)
    __disable_irq();
    g_bmu_can_handle = handle;
    __enable_irq();

    // Enable CAN RX interrupt notifications for FIFO0
    if (HAL_CAN_ActivateNotification(hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return HAL_ERROR;
    }

    if (hcan2 != NULL) {
        if (HAL_CAN_ActivateNotification(hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
            return HAL_ERROR;
        }
    }

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
    /* MISRA C 2012 Rule 17.7: Return value intentionally ignored */
    (void)HAL_CAN_Stop(hcan);

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
  * @note   CAN1 uporablja FilterBank 0, CAN2 uporablja FilterBank 14
  */
HAL_StatusTypeDef BMU_CAN_ConfigureFilter(CAN_HandleTypeDef* hcan)
{
    CAN_FilterTypeDef filter;

    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if (hcan == NULL) {
        return HAL_ERROR;
    }

    /* Determine FilterBank based on CAN instance */
    if (hcan->Instance == CAN1) {
        filter.FilterBank = 0U;  /* CAN1 uses banks 0-13 */
    } else if (hcan->Instance == CAN2) {
        filter.FilterBank = 14U;  /* CAN2 uses banks 14-27 */
    } else {
        return HAL_ERROR;
    }

    /* Configure filter to accept all messages */
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000U;
    filter.FilterIdLow = 0x0000U;
    filter.FilterMaskIdHigh = 0x0000U;
    filter.FilterMaskIdLow = 0x0000U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;  /* CAN2 start filter bank */

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
    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (msg == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, CAN_ID_BMU_STATUS,
                               (uint8_t*)msg, (uint8_t)sizeof(BMU_Status_Msg_t));
}

/**
  * @brief  Pošlji Temperature sporočilo (0x101)
  */
HAL_StatusTypeDef BMU_CAN_SendTemperature(BMU_CAN_HandleTypeDef* handle,
                                          BMU_Temperature_Msg_t* msg)
{
    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (msg == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, CAN_ID_TEMPERATURE,
                               (uint8_t*)msg, (uint8_t)sizeof(BMU_Temperature_Msg_t));
}

/**
  * @brief  Pošlji LEM Current sporočilo (0x110-0x112)
  */
HAL_StatusTypeDef BMU_CAN_SendLEMCurrent(BMU_CAN_HandleTypeDef* handle,
                                         uint32_t msg_id,
                                         BMU_LEM_Current_Msg_t* msg)
{
    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (msg == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, msg_id,
                               (uint8_t*)msg, (uint8_t)sizeof(BMU_LEM_Current_Msg_t));
}

/**
  * @brief  Pošlji BTT6200 Status sporočilo (0x120-0x122)
  */
HAL_StatusTypeDef BMU_CAN_SendBTTStatus(BMU_CAN_HandleTypeDef* handle,
                                        uint32_t msg_id,
                                        BMU_BTT6200_Status_Msg_t* msg)
{
    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (msg == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, msg_id,
                               (uint8_t*)msg, (uint8_t)sizeof(BMU_BTT6200_Status_Msg_t));
}

/**
  * @brief  Pošlji FRAM Statistics sporočilo (0x130)
  */
HAL_StatusTypeDef BMU_CAN_SendFRAMStats(BMU_CAN_HandleTypeDef* handle,
                                       BMU_FRAM_Stats_Msg_t* msg)
{
    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (msg == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, CAN_ID_FRAM_STATS,
                               (uint8_t*)msg, (uint8_t)sizeof(BMU_FRAM_Stats_Msg_t));
}

/**
  * @brief  Pošlji Alarms sporočilo (0x140)
  */
HAL_StatusTypeDef BMU_CAN_SendAlarms(BMU_CAN_HandleTypeDef* handle,
                                     BMU_Alarms_Msg_t* msg)
{
    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (msg == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, CAN_ID_ALARMS,
                               (uint8_t*)msg, (uint8_t)sizeof(BMU_Alarms_Msg_t));
}

/**
  * @brief  Pošlji Heartbeat sporočilo (0x1FF)
  */
HAL_StatusTypeDef BMU_CAN_SendHeartbeat(BMU_CAN_HandleTypeDef* handle,
                                        uint32_t counter)
{
    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    uint8_t data[8] = {0U};
    data[0] = (uint8_t)((counter >> 24) & 0xFFU);
    data[1] = (uint8_t)((counter >> 16) & 0xFFU);
    data[2] = (uint8_t)((counter >> 8) & 0xFFU);
    data[3] = (uint8_t)(counter & 0xFFU);
    data[4] = BMU_HEARTBEAT_MAGIC_BYTE1;
    data[5] = BMU_HEARTBEAT_MAGIC_BYTE2;
    data[6] = 0U;
    data[7] = 0U;

    return BMU_CAN_SendMessage(handle, CAN_ID_HEARTBEAT, data, 8U);
}

/**
  * @brief  Pridobi statistiko CAN komunikacije
  */
HAL_StatusTypeDef BMU_CAN_GetStats(BMU_CAN_HandleTypeDef* handle,
                                  uint32_t* tx_count,
                                  uint32_t* rx_count,
                                  uint32_t* error_count)
{
    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
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

/**
  * @brief  Pošlji Power Supply Status sporočilo (0x102)
  */
HAL_StatusTypeDef BMU_CAN_SendPowerSupply(BMU_CAN_HandleTypeDef* handle,
                                          BMU_PowerSupply_Msg_t* msg)
{
    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (msg == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, CAN_ID_POWER_SUPPLY,
                               (uint8_t*)msg, (uint8_t)sizeof(BMU_PowerSupply_Msg_t));
}

/**
  * @brief  Pošlji Input States sporočilo (0x103)
  */
HAL_StatusTypeDef BMU_CAN_SendInputStates(BMU_CAN_HandleTypeDef* handle,
                                          BMU_InputStates_Msg_t* msg)
{
    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (msg == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, CAN_ID_INPUT_STATES,
                               (uint8_t*)msg, (uint8_t)sizeof(BMU_InputStates_Msg_t));
}

/**
  * @brief  Pošlji BTT6200 Detailed Status sporočilo (0x124-0x128)
  */
HAL_StatusTypeDef BMU_CAN_SendBTTDetailed(BMU_CAN_HandleTypeDef* handle,
                                          uint32_t msg_id,
                                          BMU_BTT6200_Detailed_Msg_t* msg)
{
    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (msg == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    // Validate message ID
    if ((msg_id < CAN_ID_BTT6200_DETAIL_1) || (msg_id > CAN_ID_BTT6200_DETAIL_5)) {
        return HAL_ERROR;
    }

    return BMU_CAN_SendMessage(handle, msg_id,
                               (uint8_t*)msg, (uint8_t)sizeof(BMU_BTT6200_Detailed_Msg_t));
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Perform full CAN recovery sequence
  * @note   Stops CAN, resets errors, reconfigures filter, and restarts
  */
static HAL_StatusTypeDef BMU_CAN_PerformRecovery(CAN_HandleTypeDef* hcan)
{
    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if (hcan == NULL) {
        return HAL_ERROR;
    }

    (void)HAL_CAN_Stop(hcan);
    (void)HAL_CAN_ResetError(hcan);
    HAL_Delay(10U);

    if ((BMU_CAN_ConfigureFilter(hcan) != HAL_OK) ||
        (HAL_CAN_Start(hcan) != HAL_OK) ||
        (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief  Wait for TX mailbox to become available
  * @note   Polls CAN peripheral until at least one TX mailbox is free
  */
HAL_StatusTypeDef BMU_CAN_WaitTxMailboxFree(BMU_CAN_HandleTypeDef* handle, uint32_t timeout_ms)
{
    uint32_t start_tick;
    bool mailbox_free;

    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    start_tick = HAL_GetTick();
    mailbox_free = false;

    /* Wait until at least one TX mailbox is free */
    while (mailbox_free == false) {
        if (HAL_CAN_GetTxMailboxesFreeLevel(handle->hcan1) != 0U) {
            mailbox_free = true;
            break;
        }

        if ((HAL_CAN_GetError(handle->hcan1) & HAL_CAN_ERROR_BOF) != 0U) {
            return BMU_CAN_PerformRecovery(handle->hcan1);
        }

        if ((HAL_GetTick() - start_tick) > timeout_ms) {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

/**
  * @brief  Check CAN error state and recover if needed
  */
HAL_StatusTypeDef BMU_CAN_CheckAndRecover(BMU_CAN_HandleTypeDef* handle)
{
    uint32_t can_error;
    HAL_CAN_StateTypeDef can_state;

    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    can_error = HAL_CAN_GetError(handle->hcan1);
    can_state = HAL_CAN_GetState(handle->hcan1);

    /* BUS-OFF, ERROR, or RESET state - full recovery needed */
    if (((can_error & HAL_CAN_ERROR_BOF) != 0U) ||
        (can_state == HAL_CAN_STATE_RESET) ||
        (can_state == HAL_CAN_STATE_ERROR)) {
        return BMU_CAN_PerformRecovery(handle->hcan1);
    }

    /* ERROR-PASSIVE or WARNING - just reset errors */
    if ((can_error & (HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_EWG)) != 0U) {
        (void)HAL_CAN_ResetError(handle->hcan1);
    }

    /* INRQ set - CAN stuck in init mode */
    if ((handle->hcan1->Instance->MCR & CAN_MCR_INRQ) != 0U) {
        if ((HAL_CAN_Start(handle->hcan1) != HAL_OK) ||
            (HAL_CAN_ActivateNotification(handle->hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

/**
  * @brief  Pošlji CAN sporočilo
  */
static HAL_StatusTypeDef BMU_CAN_SendMessage(BMU_CAN_HandleTypeDef* handle,
                                             uint32_t msg_id,
                                             uint8_t* data,
                                             uint8_t dlc)
{
    HAL_StatusTypeDef wait_status;
    HAL_StatusTypeDef status;
    uint8_t dlc_safe;

    /* MISRA C 2012 Rule 14.4: Explicit NULL checks */
    if ((handle == NULL) || (data == NULL)) {
        return HAL_ERROR;
    }

    /* Ensure DLC is valid (0-8) */
    if (dlc > 8U) {
        dlc_safe = 8U;
    } else {
        dlc_safe = dlc;
    }

    /* Wait for TX mailbox to be free (with timeout) */
    wait_status = BMU_CAN_WaitTxMailboxFree(handle, 10U);
    if (wait_status != HAL_OK) {
        handle->error_count++;
        return wait_status;
    }

    /* Configure TX header */
    handle->tx_header.StdId = msg_id;
    handle->tx_header.IDE = CAN_ID_STD;  /* Standard 11-bit ID */
    handle->tx_header.RTR = CAN_RTR_DATA;
    handle->tx_header.DLC = dlc_safe;
    handle->tx_header.TransmitGlobalTime = DISABLE;

    /* Copy data to TX buffer */
    (void)memcpy(handle->tx_data, data, dlc_safe);

    /* Send message */
    status = HAL_CAN_AddTxMessage(handle->hcan1,
                                  &handle->tx_header,
                                  handle->tx_data,
                                  &handle->tx_mailbox);

    if (status == HAL_OK) {
        handle->tx_count++;
    } else {
        handle->error_count++;

        /* DEBUG: Print TX failure (only failures to reduce UART load) */
        #if 1
        {
            extern UART_HandleTypeDef huart1;
            char debug_buf[80];
            (void)snprintf(debug_buf, sizeof(debug_buf), "[TX ERR] 0x%03lX (Errors:%lu)\r\n",
                          msg_id, handle->error_count);
            (void)HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, (uint16_t)strlen(debug_buf), 10);
        }
        #endif
    }

    return status;
}

/**
  * @brief  Procesira prejeto CAN sporočilo (RX)
  */
HAL_StatusTypeDef BMU_CAN_ProcessRxMessage(BMU_CAN_HandleTypeDef* handle,
                                           CAN_RxHeaderTypeDef* rx_header,
                                           uint8_t* rx_data)
{
    uint32_t can_id;
    bool new_state;
    BMU_BTT6200_OutputCmd_t cmd_output;
    BMU_BTT6200_MultiCmd_t cmd_multi;
    BMU_SystemCmd_t cmd_system;
    BTT6200_Status_t current_status;
    uint8_t i;

    /* MISRA C 2012 Rule 14.4: Explicit NULL checks */
    if ((handle == NULL) || (rx_header == NULL) || (rx_data == NULL)) {
        return HAL_ERROR;
    }

    /* Extract CAN ID (standard or extended) */
    if (rx_header->IDE == CAN_ID_STD) {
        can_id = rx_header->StdId;
    } else {
        can_id = rx_header->ExtId;
    }

    /* Increment RX counter */
    handle->rx_count++;

    /* Process based on message ID */
    switch (can_id) {

        /* ========== BTT6200 Output Command (0x200) ========== */
        case CAN_ID_BTT6200_OUTPUT_CMD:
            if (rx_header->DLC < (uint8_t)sizeof(BMU_BTT6200_OutputCmd_t)) {
                handle->error_count++;
                return HAL_ERROR;
            }

            /* MISRA C 2012 Rule 11.5: Use memcpy to avoid unaligned access */
            (void)memcpy(&cmd_output, rx_data, sizeof(cmd_output));

            /* Verify magic number for safety */
            if (cmd_output.magic != BMU_MAGIC_OUTPUT_CMD) {
                handle->error_count++;
                return HAL_ERROR;
            }

            /* Validate output ID */
            if (cmd_output.output_id >= BTT6200_NUM_OUTPUTS) {
                handle->error_count++;
                return HAL_ERROR;
            }

            /* Execute command */
            if (cmd_output.command == BMU_CMD_OUTPUT_OFF) {
                new_state = false;
            } else if (cmd_output.command == BMU_CMD_OUTPUT_ON) {
                new_state = true;
            } else if (cmd_output.command == BMU_CMD_OUTPUT_TOGGLE) {
                /* Read current state and toggle */
                current_status = BTT6200_Config_GetStatus(cmd_output.output_id);
                new_state = (current_status == BTT6200_STATUS_DISABLED);
            } else {
                handle->error_count++;
                return HAL_ERROR;
            }

            /* Set output state */
            BTT6200_Config_SetOutput((BMU_Output_t)cmd_output.output_id, new_state);
            break;

        /* ========== BTT6200 Multi Command (0x201) ========== */
        case CAN_ID_BTT6200_MULTI_CMD:
            if (rx_header->DLC < (uint8_t)sizeof(BMU_BTT6200_MultiCmd_t)) {
                handle->error_count++;
                return HAL_ERROR;
            }

            /* MISRA C 2012 Rule 11.5: Use memcpy to avoid unaligned access */
            (void)memcpy(&cmd_multi, rx_data, sizeof(cmd_multi));

            /* Process each output in mask */
            for (i = 0; i < BTT6200_NUM_OUTPUTS; i++) {
                if ((cmd_multi.output_mask & (1UL << i)) != 0U) {
                    new_state = ((cmd_multi.output_states & (1UL << i)) != 0U);
                    BTT6200_Config_SetOutput((BMU_Output_t)i, new_state);
                }
            }
            break;

        /* ========== System Command (0x202) ========== */
        case CAN_ID_SYSTEM_CMD:
            if (rx_header->DLC < (uint8_t)sizeof(BMU_SystemCmd_t)) {
                handle->error_count++;
                return HAL_ERROR;
            }

            /* MISRA C 2012 Rule 11.5: Use memcpy to avoid unaligned access */
            (void)memcpy(&cmd_system, rx_data, sizeof(cmd_system));

            /* Verify magic number for safety */
            if (cmd_system.magic != BMU_MAGIC_SYSTEM_CMD) {
                handle->error_count++;
                return HAL_ERROR;
            }

            /* Execute system command */
            switch (cmd_system.command) {
                case BMU_CMD_SYSTEM_NOP:
                    /* Do nothing */
                    break;

                case BMU_CMD_SYSTEM_RESET_STATS:
                    /* Reset CAN statistics */
                    handle->tx_count = 0U;
                    handle->rx_count = 0U;
                    handle->error_count = 0U;
                    break;

                case BMU_CMD_SYSTEM_DISABLE_ALL:
                    /* Disable all BTT6200 outputs */
                    BTT6200_Config_DisableAll();
                    break;

                case BMU_CMD_SYSTEM_ENABLE_ALL:
                    /* Enable all BTT6200 outputs */
                    for (i = 0; i < BTT6200_NUM_OUTPUTS; i++) {
                        BTT6200_Config_SetOutput((BMU_Output_t)i, true);
                    }
                    break;

                case BMU_CMD_SYSTEM_REBOOT:
                    /* Software reset */
                    NVIC_SystemReset();
                    break;

                default:
                    handle->error_count++;
                    return HAL_ERROR;
            }
            break;

        default:
            /* Unknown message ID - not a BMU protocol message */
            /* This is acceptable - might be DCDC or other protocol */
            /* Do not increment error_count for unknown IDs */
            break;
    }

    return HAL_OK;
}

/**
  * @brief  CAN RX callback - klicana iz interrupt-a
  * @note   Uporabnik mora to funkcijo poklicati iz HAL_CAN_RxFifo0MsgPendingCallback
  */
void BMU_CAN_RxCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((g_bmu_can_handle == NULL) || (g_bmu_can_handle->is_initialized == false)) {
        return;
    }

    // Read message from FIFO0
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        // Process the received message (do this FIRST for speed)
        // Cast from volatile to non-volatile (safe because pointer is set once during init)
        HAL_StatusTypeDef result = BMU_CAN_ProcessRxMessage((BMU_CAN_HandleTypeDef*)g_bmu_can_handle, &rx_header, rx_data);

        // DEBUG: Lightweight RX output (only show received messages, not all data)
        #if 1
        extern UART_HandleTypeDef huart1;
        if (result == HAL_OK) {
            // Success - just count it, print summary periodically
            static uint32_t rx_count_debug = 0;
            rx_count_debug++;
            if ((rx_count_debug % 10U) == 1U) {  // Print first and every 10th
                char buf[40];
                (void)snprintf(buf, sizeof(buf), "[RX OK] 0x%03lX (count:%lu)\r\n",
                              rx_header.StdId, rx_count_debug);
                (void)HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)strlen(buf), 10);
            }
        } else {
            // Failure - always print with data for debugging
            char buf[120];
            (void)snprintf(buf, sizeof(buf),
                          "[RX ERR] 0x%03lX: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                          rx_header.StdId,
                          rx_data[0], rx_data[1], rx_data[2], rx_data[3],
                          rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
            (void)HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)strlen(buf), 10);
        }
        #endif
    }
}

/**
  * @brief  Process already-read CAN message (for use in interrupt context)
  * @note   Call this from interrupt handler after reading the message
  * @param  hcan: Pointer na CAN_HandleTypeDef
  * @param  rx_header: Pointer na že prebran CAN_RxHeaderTypeDef
  * @param  rx_data: Pointer na že prebrane podatke (8 bajtov)
  * @retval None
  */
void BMU_CAN_ProcessRxMessageISR(CAN_HandleTypeDef* hcan,
                                  CAN_RxHeaderTypeDef* rx_header,
                                  uint8_t* rx_data)
{
    HAL_StatusTypeDef result;

    /* MISRA C 2012 Rule 14.4: Explicit NULL and boolean checks */
    if ((hcan == NULL) || (rx_header == NULL) || (rx_data == NULL)) {
        return;
    }

    if ((g_bmu_can_handle == NULL) || (g_bmu_can_handle->is_initialized == false)) {
        return;
    }

    /* Cast from volatile to non-volatile (safe - pointer set once during init) */
    result = BMU_CAN_ProcessRxMessage((BMU_CAN_HandleTypeDef*)g_bmu_can_handle,
                                      rx_header, rx_data);

    /* Optional: Handle result or update statistics */
    (void)result; /* Explicitly ignore return value in ISR context */
}
