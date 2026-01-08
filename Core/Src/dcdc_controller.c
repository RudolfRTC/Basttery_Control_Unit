/**
 * @file dcdc_controller.c
 * @brief Dual DC/DC Converter Controller Implementation
 * @note MISRA C:2012 compliant
 */

#include "dcdc_controller.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* External CAN handle - must be defined in main.c */
extern CAN_HandleTypeDef hcan1;

/* Private structures */

/**
 * @brief Internal state structure for each converter
 */
typedef struct {
    DCDC_Config_t config;       /* Current configuration */
    DCDC_Status_t status;       /* Current status */
    uint32_t last_tx_tick;      /* Last transmission timestamp */
    bool needs_transmission;    /* Flag for periodic transmission */
} DCDC_Converter_t;

/* Private variables */
static DCDC_Converter_t g_converters[DCDC_COUNT];
static bool g_is_initialized = false;

/* Private function prototypes */
static bool DCDC_SendControlFrame(uint8_t converter_id);
static bool DCDC_SendModeRequest(uint8_t converter_id);
static void DCDC_PackControlFrame(const DCDC_Config_t *config, uint8_t *data);
static void DCDC_PackModeRequest(uint8_t mode, uint8_t *data);
static bool DCDC_TransmitCANMessage(uint32_t can_id, const uint8_t *data, uint8_t dlc);
static void DCDC_ProcessControlInfo(uint8_t converter_id, const uint8_t *data);
static void DCDC_ProcessMeasures(uint8_t converter_id, const uint8_t *data);
static void DCDC_ProcessErrors(uint8_t converter_id, const uint8_t *data);

/* Public function implementations */

/**
 * @brief Initialize DC/DC controller
 */
bool DCDC_Init(void)
{
    /* Initialize all converters */
    (void)memset(g_converters, 0, sizeof(g_converters));

    /* Set default configuration for both converters */
    for (uint8_t i = 0U; i < DCDC_COUNT; i++) {
        g_converters[i].config.can_address = (i == 0U) ? DCDC1_CAN_ADDR : DCDC2_CAN_ADDR;
        g_converters[i].config.converter_mode = DCDC_MODE_BUS1_TO_BUS2;
        g_converters[i].config.control_type = DCDC_CTRL_TYPE_VBUS2_LIM;
        g_converters[i].config.voltage_setpoint = DCDC_SETPOINT_24V;
        g_converters[i].config.current_limit = DCDC_CURRENT_LIMIT_10A;
        g_converters[i].config.start_command = DCDC_CMD_STOP;
        g_converters[i].status.is_initialized = true;
        g_converters[i].needs_transmission = false;
        g_converters[i].last_tx_tick = 0U;
    }

    g_is_initialized = true;
    return true;
}

/**
 * @brief Configure DC/DC converter
 */
bool DCDC_Configure(uint8_t converter_id, const DCDC_Config_t *config)
{
    /* Validate parameters */
    if ((converter_id >= DCDC_COUNT) || (config == NULL) || (g_is_initialized == false)) {
        return false;
    }

    /* Copy configuration */
    (void)memcpy(&g_converters[converter_id].config, config, sizeof(DCDC_Config_t));

    /* Ensure CAN address is correct */
    g_converters[converter_id].config.can_address =
        (converter_id == 0U) ? DCDC1_CAN_ADDR : DCDC2_CAN_ADDR;

    /* Mark for transmission */
    g_converters[converter_id].needs_transmission = true;

    return true;
}

/**
 * @brief Start DC/DC converter
 */
bool DCDC_Start(uint8_t converter_id)
{
    /* Validate parameters */
    if ((converter_id >= DCDC_COUNT) || (g_is_initialized == false)) {
        return false;
    }

    /* Set start command */
    g_converters[converter_id].config.start_command = DCDC_CMD_START;

    /* Send mode request first */
    if (DCDC_SendModeRequest(converter_id) == false) {
        return false;
    }

    /* Send control frame */
    if (DCDC_SendControlFrame(converter_id) == false) {
        return false;
    }

    /* Mark for periodic transmission */
    g_converters[converter_id].needs_transmission = true;
    g_converters[converter_id].last_tx_tick = HAL_GetTick();

    return true;
}

/**
 * @brief Stop DC/DC converter
 */
bool DCDC_Stop(uint8_t converter_id)
{
    /* Validate parameters */
    if ((converter_id >= DCDC_COUNT) || (g_is_initialized == false)) {
        return false;
    }

    /* Set stop command */
    g_converters[converter_id].config.start_command = DCDC_CMD_STOP;

    /* Send control frame to stop */
    if (DCDC_SendControlFrame(converter_id) == false) {
        return false;
    }

    /* Disable periodic transmission */
    g_converters[converter_id].needs_transmission = false;

    return true;
}

/**
 * @brief Update DC/DC converter setpoint
 */
bool DCDC_UpdateSetpoint(uint8_t converter_id, uint16_t voltage_setpoint, uint16_t current_limit)
{
    /* Validate parameters */
    if ((converter_id >= DCDC_COUNT) || (g_is_initialized == false)) {
        return false;
    }

    /* Update setpoint */
    g_converters[converter_id].config.voltage_setpoint = voltage_setpoint;
    g_converters[converter_id].config.current_limit = current_limit;

    /* Configuration will be sent in next periodic task */

    return true;
}

/**
 * @brief Periodic task - must be called every 20ms
 */
void DCDC_PeriodicTask(void)
{
    uint32_t current_tick;
    uint32_t elapsed_ms;

    if (g_is_initialized == false) {
        return;
    }

    current_tick = HAL_GetTick();

    /* Process each converter */
    for (uint8_t i = 0U; i < DCDC_COUNT; i++) {
        /* Check if this converter needs periodic transmission */
        if (g_converters[i].needs_transmission == false) {
            continue;
        }

        /* Calculate elapsed time */
        elapsed_ms = current_tick - g_converters[i].last_tx_tick;

        /* Send control frame every 20ms */
        if (elapsed_ms >= DCDC_CAN_PERIOD_MS) {
            (void)DCDC_SendControlFrame(i);
            g_converters[i].last_tx_tick = current_tick;
        }
    }
}

/**
 * @brief Process received CAN message
 */
void DCDC_ProcessCANMessage(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    uint32_t base_id;
    uint8_t can_addr;
    uint8_t converter_id;

    /* Validate parameters */
    if ((data == NULL) || (g_is_initialized == false)) {
        return;
    }

    /* Extract CAN address from ID (last byte) */
    can_addr = (uint8_t)(can_id & 0xFFU);

    /* Determine which converter this message is for */
    if (can_addr == DCDC1_CAN_ADDR) {
        converter_id = 0U;
    } else if (can_addr == DCDC2_CAN_ADDR) {
        converter_id = 1U;
    } else {
        /* Unknown CAN address */
        return;
    }

    /* Get base ID (without address) */
    base_id = can_id & 0xFFFFFF00U;

    /* Process based on message type */
    switch (base_id) {
        case DCDC_CONTROL_INFO_FRAME_ID:
            if (dlc >= 4U) {
                DCDC_ProcessControlInfo(converter_id, data);
            }
            break;

        case DCDC_MEASURES1_FRAME_ID:
            if (dlc >= 8U) {
                DCDC_ProcessMeasures(converter_id, data);
            }
            break;

        case DCDC_ERRORS_FRAME_ID:
            if (dlc >= 4U) {
                DCDC_ProcessErrors(converter_id, data);
            }
            break;

        default:
            /* Unknown message type */
            break;
    }
}

/**
 * @brief Get converter status
 */
bool DCDC_GetStatus(uint8_t converter_id, DCDC_Status_t *status)
{
    /* Validate parameters */
    if ((converter_id >= DCDC_COUNT) || (status == NULL) || (g_is_initialized == false)) {
        return false;
    }

    /* Copy status */
    (void)memcpy(status, &g_converters[converter_id].status, sizeof(DCDC_Status_t));

    return true;
}

/**
 * @brief Check if converter has errors
 */
bool DCDC_HasErrors(uint8_t converter_id)
{
    bool has_errors = false;

    /* Validate parameters */
    if ((converter_id >= DCDC_COUNT) || (g_is_initialized == false)) {
        return true;  /* Return error state if invalid parameters */
    }

    /* Check error flags */
    if (g_converters[converter_id].status.error_flags != 0U) {
        has_errors = true;
    }

    return has_errors;
}

/* Private function implementations */

/**
 * @brief Send control frame to converter
 */
static bool DCDC_SendControlFrame(uint8_t converter_id)
{
    uint8_t data[8];
    uint32_t can_id;

    /* Validate parameters */
    if (converter_id >= DCDC_COUNT) {
        return false;
    }

    /* Pack control frame data */
    DCDC_PackControlFrame(&g_converters[converter_id].config, data);

    /* Calculate CAN ID with address offset */
    can_id = DCDC_CONTROL_FRAME_ID | (uint32_t)g_converters[converter_id].config.can_address;

    /* Transmit message */
    return DCDC_TransmitCANMessage(can_id, data, DCDC_CAN_DLC_CONTROL);
}

/**
 * @brief Send mode request to converter
 */
static bool DCDC_SendModeRequest(uint8_t converter_id)
{
    uint8_t data[8];
    uint32_t can_id;

    /* Validate parameters */
    if (converter_id >= DCDC_COUNT) {
        return false;
    }

    /* Pack mode request data */
    DCDC_PackModeRequest(g_converters[converter_id].config.converter_mode, data);

    /* Calculate CAN ID with address offset */
    can_id = DCDC_MODE_REQUEST_FRAME_ID | (uint32_t)g_converters[converter_id].config.can_address;

    /* Transmit message */
    return DCDC_TransmitCANMessage(can_id, data, DCDC_CAN_DLC_MODE_REQ);
}

/**
 * @brief Pack control frame data (Little Endian)
 */
static void DCDC_PackControlFrame(const DCDC_Config_t *config, uint8_t *data)
{
    /* Clear data buffer */
    (void)memset(data, 0, 8);

    /* Byte 0: Control type */
    data[0] = config->control_type;

    /* Byte 1: Start/Stop command */
    data[1] = config->start_command;

    /* Bytes 2-3: Voltage setpoint (Little Endian, 0.1V units) */
    data[2] = (uint8_t)(config->voltage_setpoint & 0xFFU);
    data[3] = (uint8_t)((config->voltage_setpoint >> 8) & 0xFFU);

    /* Bytes 4-5: Current limit (Little Endian, 0.1A units) */
    data[4] = (uint8_t)(config->current_limit & 0xFFU);
    data[5] = (uint8_t)((config->current_limit >> 8) & 0xFFU);

    /* Byte 6: Reserved */
    data[6] = 0x00U;
}

/**
 * @brief Pack mode request data
 */
static void DCDC_PackModeRequest(uint8_t mode, uint8_t *data)
{
    /* Clear data buffer */
    (void)memset(data, 0, 8);

    /* Byte 0: Converter mode */
    data[0] = mode;

    /* Byte 1: Reserved */
    data[1] = 0x00U;
}

/**
 * @brief Transmit CAN message with extended 29-bit ID
 */
static bool DCDC_TransmitCANMessage(uint32_t can_id, const uint8_t *data, uint8_t dlc)
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

    /* Configure TX header for extended 29-bit ID */
    tx_header.ExtId = can_id;
    tx_header.IDE = CAN_ID_EXT;  /* Extended identifier */
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    /* Copy data to local buffer */
    (void)memcpy(tx_data, data, dlc);

    /* Add message to TX mailbox */
    status = HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);

    return (status == HAL_OK);
}

/**
 * @brief Process control info frame
 */
static void DCDC_ProcessControlInfo(uint8_t converter_id, const uint8_t *data)
{
    /* Validate parameters */
    if ((converter_id >= DCDC_COUNT) || (data == NULL)) {
        return;
    }

    /* Extract operating mode and control type */
    g_converters[converter_id].status.operating_mode = data[0];
    g_converters[converter_id].status.control_type = data[1];
}

/**
 * @brief Process measures frame (voltages, currents, power)
 */
static void DCDC_ProcessMeasures(uint8_t converter_id, const uint8_t *data)
{
    uint16_t temp_u16;

    /* Validate parameters */
    if ((converter_id >= DCDC_COUNT) || (data == NULL)) {
        return;
    }

    /* Extract Bus1 voltage (bytes 0-1, Little Endian, 0.1V) */
    temp_u16 = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    g_converters[converter_id].status.voltage_bus1 = temp_u16;

    /* Extract Bus2 voltage (bytes 2-3, Little Endian, 0.1V) */
    temp_u16 = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    g_converters[converter_id].status.voltage_bus2 = temp_u16;

    /* Extract Bus1 current (bytes 4-5, Little Endian, 0.1A, signed) */
    g_converters[converter_id].status.current_bus1 =
        (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));

    /* Extract Bus2 current (bytes 6-7, Little Endian, 0.1A, signed) */
    g_converters[converter_id].status.current_bus2 =
        (int16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));

    /* Calculate power (P = V * I / 100, result in Watts) */
    /* Voltage in 0.1V, Current in 0.1A, so multiply and divide by 100 */
    g_converters[converter_id].status.power_bus1 =
        ((uint32_t)g_converters[converter_id].status.voltage_bus1 *
         (uint32_t)((g_converters[converter_id].status.current_bus1 >= 0) ?
                    g_converters[converter_id].status.current_bus1 :
                    -g_converters[converter_id].status.current_bus1)) / 100U;

    g_converters[converter_id].status.power_bus2 =
        ((uint32_t)g_converters[converter_id].status.voltage_bus2 *
         (uint32_t)((g_converters[converter_id].status.current_bus2 >= 0) ?
                    g_converters[converter_id].status.current_bus2 :
                    -g_converters[converter_id].status.current_bus2)) / 100U;
}

/**
 * @brief Process error frame
 */
static void DCDC_ProcessErrors(uint8_t converter_id, const uint8_t *data)
{
    /* Validate parameters */
    if ((converter_id >= DCDC_COUNT) || (data == NULL)) {
        return;
    }

    /* Extract error flags (bytes 0-3, Little Endian) */
    g_converters[converter_id].status.error_flags =
        (uint32_t)data[0] |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);
}
