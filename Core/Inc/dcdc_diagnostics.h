/**
 * @file dcdc_diagnostics.h
 * @brief DC/DC Converter CAN1 Diagnostic Interface
 * @note MISRA C:2012 compliant
 *
 * CAN1 vmesnik za upravljanje in diagnostiko DC/DC pretvornikov:
 * - RX (Komande): Vklop/Izklop pretvornikov
 * - TX (Diagnostika): Status, napetosti, tokovi, moči, napake
 */

#ifndef DCDC_DIAGNOSTICS_H
#define DCDC_DIAGNOSTICS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <stdint.h>
#include <stdbool.h>

/* CAN1 Message IDs - Commands (RX) */
#define DCDC_DIAG_CMD_CONV1         0x300U  /* Komanda za pretvornik 1 (ON/OFF) */
#define DCDC_DIAG_CMD_CONV2         0x301U  /* Komanda za pretvornik 2 (ON/OFF) */
#define DCDC_DIAG_CMD_BOTH          0x302U  /* Komanda za oba pretvornika (ON/OFF) */
#define DCDC_DIAG_CFG_CONV1         0x303U  /* Konfiguracijska komanda za pretvornik 1 */
#define DCDC_DIAG_CFG_CONV2         0x304U  /* Konfiguracijska komanda za pretvornik 2 */
#define DCDC_DIAG_CFG_BOTH          0x305U  /* Konfiguracijska komanda za oba */

/* CAN1 Message IDs - Diagnostics (TX) */
#define DCDC_DIAG_STATUS_CONV1      0x400U  /* Status pretvornika 1 */
#define DCDC_DIAG_MEASURES_CONV1    0x401U  /* Napetosti in tokovi pretvornika 1 */
#define DCDC_DIAG_POWER_CONV1       0x402U  /* Moči pretvornika 1 */

#define DCDC_DIAG_STATUS_CONV2      0x410U  /* Status pretvornika 2 */
#define DCDC_DIAG_MEASURES_CONV2    0x411U  /* Napetosti in tokovi pretvornika 2 */
#define DCDC_DIAG_POWER_CONV2       0x412U  /* Moči pretvornika 2 */

#define DCDC_DIAG_CONNECTIVITY      0x500U  /* Connectivity status obe pretvornika */

/* Command values */
#define DCDC_DIAG_CMD_OFF           0x00U   /* Izklop */
#define DCDC_DIAG_CMD_ON            0x01U   /* Vklop */

/* Diagnostic transmission period */
#define DCDC_DIAG_TX_PERIOD_MS      100U    /* Pošiljaj diagnostiko vsakih 100ms */

/* Connectivity timeout - če ni CAN2 sporočil X ms, pretvornik ni povezan */
#define DCDC_CONNECTIVITY_TIMEOUT_MS 500U   /* 500ms timeout */

/**
 * @brief Command message structure (RX on CAN1)
 * DLC: 8 bytes
 */
typedef struct __attribute__((packed)) {
    uint8_t command;            /* 0=OFF, 1=ON */
    uint8_t reserved[7];        /* Rezervirano za prihodnost */
} DCDC_Diag_Command_t;

/**
 * @brief Configuration message structure (RX on CAN1)
 * DLC: 8 bytes
 * Allows dynamic voltage setpoint and current limit adjustment
 */
typedef struct __attribute__((packed)) {
    uint16_t voltage_setpoint;  /* Napetost v 0.1V (e.g., 240 = 24.0V) */
    uint16_t current_limit;     /* Tok limit v 0.1A (e.g., 100 = 10.0A) */
    uint8_t  control_type;      /* 7=vBus2Lim, 8=vBus1Lim */
    uint8_t  converter_mode;    /* 0=Standby, 1=Bus1toBus2, 2=Bus2toBus1 */
    uint16_t reserved;          /* Rezervirano */
} DCDC_Diag_Config_t;

/**
 * @brief Status diagnostic message (TX on CAN1)
 * DLC: 8 bytes
 */
typedef struct __attribute__((packed)) {
    uint8_t operating_mode;     /* Operating mode (0=Standby, 1=Bus1toBus2, etc.) */
    uint8_t control_type;       /* Control type (7=vBus2Lim, 8=vBus1Lim) */
    uint8_t is_running;         /* 0=Stopped, 1=Running */
    uint8_t has_errors;         /* 0=No errors, 1=Has errors */
    uint32_t error_flags;       /* Error flags bitfield */
} DCDC_Diag_Status_t;

/**
 * @brief Voltage and current measurements (TX on CAN1)
 * DLC: 8 bytes
 */
typedef struct __attribute__((packed)) {
    uint16_t voltage_bus1;      /* Bus1 voltage in 0.1V (e.g., 480 = 48.0V) */
    uint16_t voltage_bus2;      /* Bus2 voltage in 0.1V (e.g., 240 = 24.0V) */
    int16_t current_bus1;       /* Bus1 current in 0.1A, signed */
    int16_t current_bus2;       /* Bus2 current in 0.1A, signed */
} DCDC_Diag_Measures_t;

/**
 * @brief Power measurements (TX on CAN1)
 * DLC: 8 bytes
 */
typedef struct __attribute__((packed)) {
    uint32_t power_bus1;        /* Bus1 power in Watts */
    uint32_t power_bus2;        /* Bus2 power in Watts */
} DCDC_Diag_Power_t;

/**
 * @brief Connectivity status (TX on CAN1)
 * DLC: 8 bytes
 * Reports whether DC/DC converters are connected based on CAN2 activity
 */
typedef struct __attribute__((packed)) {
    uint8_t conv1_connected;    /* 0=Not connected, 1=Connected (CAN2 messages received) */
    uint8_t conv2_connected;    /* 0=Not connected, 1=Connected (CAN2 messages received) */
    uint32_t conv1_last_rx_ms;  /* Time since last CAN2 message from Conv1 (milliseconds) */
    uint16_t conv2_last_rx_ms;  /* Time since last CAN2 message from Conv2 (milliseconds) */
} DCDC_Diag_Connectivity_t;

/* Exported functions */

/**
 * @brief Initialize diagnostic interface
 * @return true if successful, false otherwise
 */
bool DCDC_Diag_Init(void);

/**
 * @brief Process received command from CAN1
 * @param can_id CAN message ID
 * @param data Pointer to received data (8 bytes)
 * @param dlc Data length code
 */
void DCDC_Diag_ProcessCommand(uint32_t can_id, const uint8_t *data, uint8_t dlc);

/**
 * @brief Periodic task to send diagnostics on CAN1
 * @note Call this from main loop or timer interrupt every 100ms
 */
void DCDC_Diag_PeriodicTask(void);

/**
 * @brief Send status diagnostic for specified converter
 * @param converter_id Converter ID (0 or 1)
 * @return true if sent successfully, false otherwise
 */
bool DCDC_Diag_SendStatus(uint8_t converter_id);

/**
 * @brief Send voltage/current measurements for specified converter
 * @param converter_id Converter ID (0 or 1)
 * @return true if sent successfully, false otherwise
 */
bool DCDC_Diag_SendMeasures(uint8_t converter_id);

/**
 * @brief Send power measurements for specified converter
 * @param converter_id Converter ID (0 or 1)
 * @return true if sent successfully, false otherwise
 */
bool DCDC_Diag_SendPower(uint8_t converter_id);

/**
 * @brief Send all diagnostics for both converters immediately
 * @return true if all sent successfully, false otherwise
 */
bool DCDC_Diag_SendAll(void);

/**
 * @brief Send connectivity status message on CAN1
 * @return true if sent successfully, false otherwise
 */
bool DCDC_Diag_SendConnectivity(void);

/**
 * @brief Update last RX timestamp for specified converter (called from CAN2 RX)
 * @param converter_id Converter ID (0 or 1)
 */
void DCDC_Diag_UpdateConnectivity(uint8_t converter_id);

#ifdef __cplusplus
}
#endif

#endif /* DCDC_DIAGNOSTICS_H */
