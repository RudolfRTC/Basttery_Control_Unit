/**
 * @file dcdc_diagnostics.c
 * @brief DC/DC Converter CAN1 Diagnostic Interface Implementation
 * @note MISRA C:2012 compliant
 */

#include "dcdc_diagnostics.h"
#include "dcdc_controller.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* External CAN handle - must be defined in main.c */
extern CAN_HandleTypeDef hcan1;

/* Private variables */
static bool g_diag_is_initialized = false;
static uint32_t g_last_diag_tx_tick = 0U;

/* Private function prototypes */
static bool DCDC_Diag_TransmitCAN1(uint32_t can_id, const uint8_t *data, uint8_t dlc);

/* Public function implementations */

/**
 * @brief Initialize diagnostic interface
 */
bool DCDC_Diag_Init(void)
{
    g_diag_is_initialized = true;
    g_last_diag_tx_tick = 0U;

    return true;
}

/**
 * @brief Process received command from CAN1
 */
void DCDC_Diag_ProcessCommand(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    DCDC_Diag_Command_t cmd;
    uint8_t target_converter;
    bool process_both;

    /* Validate parameters */
    if ((data == NULL) || (dlc < 1U) || (g_diag_is_initialized == false)) {
        return;
    }

    /* Parse command */
    (void)memcpy(&cmd, data, sizeof(DCDC_Diag_Command_t));

    /* Determine which converter(s) to control */
    process_both = false;
    target_converter = 0U;

    switch (can_id) {
        case DCDC_DIAG_CMD_CONV1:
            target_converter = 0U;
            process_both = false;
            break;

        case DCDC_DIAG_CMD_CONV2:
            target_converter = 1U;
            process_both = false;
            break;

        case DCDC_DIAG_CMD_BOTH:
            target_converter = 0U;  /* Start with first converter */
            process_both = true;
            break;

        default:
            /* Unknown command ID */
            return;
    }

    /* Execute command */
    if (cmd.command == DCDC_DIAG_CMD_ON) {
        /* Vklop */
        (void)DCDC_Start(target_converter);
        if (process_both == true) {
            (void)DCDC_Start(1U);
        }
    } else if (cmd.command == DCDC_DIAG_CMD_OFF) {
        /* Izklop */
        (void)DCDC_Stop(target_converter);
        if (process_both == true) {
            (void)DCDC_Stop(1U);
        }
    } else {
        /* Unknown command value */
        return;
    }

    /* Send immediate diagnostic update after command */
    if (process_both == true) {
        (void)DCDC_Diag_SendAll();
    } else {
        (void)DCDC_Diag_SendStatus(target_converter);
        (void)DCDC_Diag_SendMeasures(target_converter);
        (void)DCDC_Diag_SendPower(target_converter);
    }
}

/**
 * @brief Periodic task to send diagnostics on CAN1
 */
void DCDC_Diag_PeriodicTask(void)
{
    uint32_t current_tick;
    uint32_t elapsed_ms;

    if (g_diag_is_initialized == false) {
        return;
    }

    current_tick = HAL_GetTick();
    elapsed_ms = current_tick - g_last_diag_tx_tick;

    /* Send diagnostics every 100ms */
    if (elapsed_ms >= DCDC_DIAG_TX_PERIOD_MS) {
        (void)DCDC_Diag_SendAll();
        g_last_diag_tx_tick = current_tick;
    }
}

/**
 * @brief Send status diagnostic for specified converter
 */
bool DCDC_Diag_SendStatus(uint8_t converter_id)
{
    DCDC_Status_t status;
    DCDC_Diag_Status_t diag_msg;
    uint32_t can_id;
    bool result;

    /* Validate parameters */
    if ((converter_id > 1U) || (g_diag_is_initialized == false)) {
        return false;
    }

    /* Get converter status */
    if (DCDC_GetStatus(converter_id, &status) == false) {
        return false;
    }

    /* Pack diagnostic message */
    diag_msg.operating_mode = status.operating_mode;
    diag_msg.control_type = status.control_type;
    diag_msg.is_running = (status.operating_mode != 0U) ? 1U : 0U;
    diag_msg.has_errors = (status.error_flags != 0U) ? 1U : 0U;
    diag_msg.error_flags = status.error_flags;

    /* Determine CAN ID */
    can_id = (converter_id == 0U) ? DCDC_DIAG_STATUS_CONV1 : DCDC_DIAG_STATUS_CONV2;

    /* Transmit message */
    result = DCDC_Diag_TransmitCAN1(can_id, (const uint8_t *)&diag_msg,
                                    (uint8_t)sizeof(DCDC_Diag_Status_t));

    return result;
}

/**
 * @brief Send voltage/current measurements for specified converter
 */
bool DCDC_Diag_SendMeasures(uint8_t converter_id)
{
    DCDC_Status_t status;
    DCDC_Diag_Measures_t diag_msg;
    uint32_t can_id;
    bool result;

    /* Validate parameters */
    if ((converter_id > 1U) || (g_diag_is_initialized == false)) {
        return false;
    }

    /* Get converter status */
    if (DCDC_GetStatus(converter_id, &status) == false) {
        return false;
    }

    /* Pack diagnostic message */
    diag_msg.voltage_bus1 = status.voltage_bus1;
    diag_msg.voltage_bus2 = status.voltage_bus2;
    diag_msg.current_bus1 = status.current_bus1;
    diag_msg.current_bus2 = status.current_bus2;

    /* Determine CAN ID */
    can_id = (converter_id == 0U) ? DCDC_DIAG_MEASURES_CONV1 : DCDC_DIAG_MEASURES_CONV2;

    /* Transmit message */
    result = DCDC_Diag_TransmitCAN1(can_id, (const uint8_t *)&diag_msg,
                                    (uint8_t)sizeof(DCDC_Diag_Measures_t));

    return result;
}

/**
 * @brief Send power measurements for specified converter
 */
bool DCDC_Diag_SendPower(uint8_t converter_id)
{
    DCDC_Status_t status;
    DCDC_Diag_Power_t diag_msg;
    uint32_t can_id;
    bool result;

    /* Validate parameters */
    if ((converter_id > 1U) || (g_diag_is_initialized == false)) {
        return false;
    }

    /* Get converter status */
    if (DCDC_GetStatus(converter_id, &status) == false) {
        return false;
    }

    /* Pack diagnostic message */
    diag_msg.power_bus1 = status.power_bus1;
    diag_msg.power_bus2 = status.power_bus2;

    /* Determine CAN ID */
    can_id = (converter_id == 0U) ? DCDC_DIAG_POWER_CONV1 : DCDC_DIAG_POWER_CONV2;

    /* Transmit message */
    result = DCDC_Diag_TransmitCAN1(can_id, (const uint8_t *)&diag_msg,
                                    (uint8_t)sizeof(DCDC_Diag_Power_t));

    return result;
}

/**
 * @brief Send all diagnostics for both converters
 */
bool DCDC_Diag_SendAll(void)
{
    bool result;
    bool all_success;

    all_success = true;

    /* Send diagnostics for converter 1 */
    result = DCDC_Diag_SendStatus(0U);
    if (result == false) {
        all_success = false;
    }

    result = DCDC_Diag_SendMeasures(0U);
    if (result == false) {
        all_success = false;
    }

    result = DCDC_Diag_SendPower(0U);
    if (result == false) {
        all_success = false;
    }

    /* Send diagnostics for converter 2 */
    result = DCDC_Diag_SendStatus(1U);
    if (result == false) {
        all_success = false;
    }

    result = DCDC_Diag_SendMeasures(1U);
    if (result == false) {
        all_success = false;
    }

    result = DCDC_Diag_SendPower(1U);
    if (result == false) {
        all_success = false;
    }

    return all_success;
}

/* Private function implementations */

/**
 * @brief Transmit CAN message on CAN1 with standard 11-bit ID
 */
static bool DCDC_Diag_TransmitCAN1(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    HAL_StatusTypeDef status;
    uint32_t timeout_start;
    uint32_t free_level;

    /* Validate parameters */
    if ((data == NULL) || (dlc > 8U)) {
        return false;
    }

    /* Wait for free TX mailbox (10ms timeout) */
    timeout_start = HAL_GetTick();
    do {
        free_level = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
        if (free_level > 0U) {
            break;
        }
        if ((HAL_GetTick() - timeout_start) > 10U) {
            return false;  /* Timeout */
        }
    } while (true);

    /* Configure TX header for standard 11-bit ID */
    tx_header.StdId = can_id;
    tx_header.IDE = CAN_ID_STD;  /* Standard identifier */
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    /* Copy data to local buffer */
    (void)memcpy(tx_data, data, dlc);

    /* Add message to TX mailbox */
    status = HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);

    return (status == HAL_OK);
}
