/**
 * @file dcdc_controller.h
 * @brief Dual DC/DC Converter Controller Header
 * @note MISRA C:2012 compliant
 */

#ifndef DCDC_CONTROLLER_H
#define DCDC_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

/* CAN Frame Identifiers */
#define DCDC_CONTROL_FRAME_ID        0x00100000U
#define DCDC_MODE_REQUEST_FRAME_ID   0x00500000U
#define DCDC_CONTROL_INFO_FRAME_ID   0x07900000U
#define DCDC_SETPOINT_INFO_FRAME_ID  0x06F00000U
#define DCDC_MEASURES1_FRAME_ID      0x03300000U
#define DCDC_ERRORS_FRAME_ID         0x01F00000U

/* Converter Mode Definitions */
#define DCDC_MODE_BUS1_TO_BUS2       0x01U
#define DCDC_MODE_BUS2_TO_BUS1       0x02U

/* Control Type Definitions */
#define DCDC_CTRL_TYPE_VBUS2         0x01U
#define DCDC_CTRL_TYPE_VBUS1         0x02U
#define DCDC_CTRL_TYPE_IBUS1         0x03U
#define DCDC_CTRL_TYPE_IBUS2         0x04U
#define DCDC_CTRL_TYPE_VBUS2_LIM     0x07U  /* Voltage with current limit */
#define DCDC_CTRL_TYPE_VBUS1_LIM     0x08U

/* Command Definitions */
#define DCDC_CMD_STOP                0x00U
#define DCDC_CMD_START               0x01U

/* Operating States */
#define DCDC_STATE_EMERGENCY         0x00U
#define DCDC_STATE_STANDBY           0x01U
#define DCDC_STATE_RUNNING           0x02U
#define DCDC_STATE_POWER_DOWN        0x03U
#define DCDC_STATE_SETTINGS          0x04U

/* Configuration Constants */
#define DCDC_SETPOINT_24V            240U    /* 24V * 10 (unit: 0.1V) */
#define DCDC_CURRENT_LIMIT_10A       100U    /* 10A * 10 (unit: 0.1A) */
#define DCDC_CAN_PERIOD_MS           20U     /* CAN frame period in ms */
#define DCDC_CAN_DLC_CONTROL         7U      /* Control frame data length */
#define DCDC_CAN_DLC_MODE_REQ        2U      /* Mode request frame data length */

/* Number of DC/DC converters */
#define DCDC_COUNT                   2U

/* CAN Address offsets for dual converters */
#define DCDC1_CAN_ADDR               0x00U
#define DCDC2_CAN_ADDR               0x01U

/**
 * @brief DC/DC Converter Status Structure
 */
typedef struct {
    uint8_t operating_mode;      /* Current operating mode */
    uint8_t control_type;        /* Current control type */
    int16_t current_bus1;        /* Bus1 current in 0.1A */
    int16_t current_bus2;        /* Bus2 current in 0.1A */
    uint16_t voltage_bus1;       /* Bus1 voltage in 0.1V */
    uint16_t voltage_bus2;       /* Bus2 voltage in 0.1V */
    uint32_t power_bus1;         /* Bus1 power in W */
    uint32_t power_bus2;         /* Bus2 power in W */
    uint32_t error_flags;        /* Error flags */
    bool is_initialized;         /* Initialization status */
} DCDC_Status_t;

/**
 * @brief DC/DC Converter Configuration Structure
 */
typedef struct {
    uint8_t can_address;         /* CAN address for this converter */
    uint8_t converter_mode;      /* Converter mode (Buck/Boost) */
    uint8_t control_type;        /* Control type */
    uint16_t voltage_setpoint;   /* Voltage setpoint in 0.1V */
    uint16_t current_limit;      /* Current limit in 0.1A */
    uint8_t start_command;       /* Start/Stop command */
} DCDC_Config_t;

/* Function Prototypes */

/**
 * @brief Initialize DC/DC controller
 * @return true if initialization successful, false otherwise
 */
bool DCDC_Init(void);

/**
 * @brief Configure DC/DC converter
 * @param converter_id Converter ID (0 or 1)
 * @param config Pointer to configuration structure
 * @return true if configuration successful, false otherwise
 */
bool DCDC_Configure(uint8_t converter_id, const DCDC_Config_t *config);

/**
 * @brief Start DC/DC converter
 * @param converter_id Converter ID (0 or 1)
 * @return true if start successful, false otherwise
 */
bool DCDC_Start(uint8_t converter_id);

/**
 * @brief Stop DC/DC converter
 * @param converter_id Converter ID (0 or 1)
 * @return true if stop successful, false otherwise
 */
bool DCDC_Stop(uint8_t converter_id);

/**
 * @brief Update DC/DC converter setpoint
 * @param converter_id Converter ID (0 or 1)
 * @param voltage_setpoint Voltage setpoint in 0.1V
 * @param current_limit Current limit in 0.1A
 * @return true if update successful, false otherwise
 */
bool DCDC_UpdateSetpoint(uint8_t converter_id, uint16_t voltage_setpoint, uint16_t current_limit);

/**
 * @brief Periodic task - must be called every 20ms
 */
void DCDC_PeriodicTask(void);

/**
 * @brief Process received CAN message
 * @param can_id CAN identifier
 * @param data Pointer to received data
 * @param dlc Data length code
 */
void DCDC_ProcessCANMessage(uint32_t can_id, const uint8_t *data, uint8_t dlc);

/**
 * @brief Get converter status
 * @param converter_id Converter ID (0 or 1)
 * @param status Pointer to status structure to be filled
 * @return true if successful, false otherwise
 */
bool DCDC_GetStatus(uint8_t converter_id, DCDC_Status_t *status);

/**
 * @brief Check if converter has errors
 * @param converter_id Converter ID (0 or 1)
 * @return true if errors present, false otherwise
 */
bool DCDC_HasErrors(uint8_t converter_id);

#endif /* DCDC_CONTROLLER_H */
