/**
  ******************************************************************************
  * @file           : bmu_can.h
  * @brief          : BMU CAN Protocol Header
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  * @attention
  *
  * CAN protocol za Battery Management Unit (BMU)
  * - CAN1: Internal communication (500 kbps)
  * - CAN2: External BMS communication (500 kbps)
  * - Standard 11-bit identifiers
  * - DBC file compatible
  *
  * CAN Message IDs (Standard 11-bit):
  * 0x100 - BMU Status
  * 0x101 - Temperature Data
  * 0x110-0x119 - LEM Current Data (10 sensors)
  * 0x120-0x123 - BTT6200 Output Status (5 modules)
  * 0x130 - FRAM Statistics
  * 0x140 - Alarms & Errors
  *
  ******************************************************************************
  */

#ifndef __BMU_CAN_H
#define __BMU_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  CAN message IDs
  */
typedef enum {
    // Status Messages (TX) - 0x100-0x1FF
    CAN_ID_BMU_STATUS       = 0x100,    // BMU system status
    CAN_ID_TEMPERATURE      = 0x101,    // Temperature sensor data
    CAN_ID_POWER_SUPPLY     = 0x102,    // Power supply status (5V, 3.3V, 24V)
    CAN_ID_INPUT_STATES     = 0x103,    // Digital input states (IN_1 to IN_20)

    CAN_ID_LEM_CURRENT_1    = 0x110,    // LEM 1-4 currents
    CAN_ID_LEM_CURRENT_2    = 0x111,    // LEM 5-8 currents
    CAN_ID_LEM_CURRENT_3    = 0x112,    // LEM 9-10 currents

    CAN_ID_BTT6200_STATUS_1 = 0x120,    // BTT6200 modules 0-1 (basic status)
    CAN_ID_BTT6200_STATUS_2 = 0x121,    // BTT6200 modules 2-3 (basic status)
    CAN_ID_BTT6200_STATUS_3 = 0x122,    // BTT6200 module 4 (basic status)

    CAN_ID_BTT6200_DETAIL_1 = 0x124,    // BTT6200 OUT 0-3 (detailed with current)
    CAN_ID_BTT6200_DETAIL_2 = 0x125,    // BTT6200 OUT 4-7
    CAN_ID_BTT6200_DETAIL_3 = 0x126,    // BTT6200 OUT 8-11
    CAN_ID_BTT6200_DETAIL_4 = 0x127,    // BTT6200 OUT 12-15
    CAN_ID_BTT6200_DETAIL_5 = 0x128,    // BTT6200 OUT 16-19

    CAN_ID_FRAM_STATS       = 0x130,    // FRAM statistics
    CAN_ID_ALARMS           = 0x140,    // Alarms and errors
    CAN_ID_HEARTBEAT        = 0x1FF,    // Heartbeat message

    // Command Messages (RX) - 0x200-0x2FF
    CAN_ID_BTT6200_OUTPUT_CMD = 0x200,  // Control single BTT6200 output
    CAN_ID_BTT6200_MULTI_CMD  = 0x201,  // Control multiple outputs (bitmap)
    CAN_ID_SYSTEM_CMD         = 0x202,  // System commands (reset, reboot, etc.)
} BMU_CAN_MessageID_t;

/**
  * @brief  BMU Status message (0x100)
  */
typedef struct __attribute__((packed)) {
    uint8_t  system_state;      // 0=Init, 1=Normal, 2=Warning, 3=Error
    uint8_t  power_good_flags;  // Bit0=5V, Bit1=3V3A, Bit2=24V
    uint16_t uptime_seconds;    // System uptime in seconds
    uint8_t  active_outputs;    // Number of active BTT6200 outputs
    uint8_t  active_sensors;    // Number of active LEM sensors
    uint16_t reserved;
} BMU_Status_Msg_t;

/**
  * @brief  Temperature message (0x101)
  */
typedef struct __attribute__((packed)) {
    int16_t temperature_C;      // Temperature × 100 (e.g., 2534 = 25.34°C)
    uint8_t alert_flag;         // 1 = below 0°C alert
    uint8_t sensor_status;      // TMP1075 status
    int16_t min_temp_C;         // Min temperature × 100
    int16_t max_temp_C;         // Max temperature × 100
} BMU_Temperature_Msg_t;

/**
  * @brief  LEM Current message (0x110-0x112)
  */
typedef struct __attribute__((packed)) {
    int16_t current_1_mA;       // Current in mA (signed)
    int16_t current_2_mA;
    int16_t current_3_mA;
    int16_t current_4_mA;
} BMU_LEM_Current_Msg_t;

/**
  * @brief  BTT6200 Status message (0x120-0x122)
  */
typedef struct __attribute__((packed)) {
    uint16_t output_states_1;   // Module 0: bits 0-3 = CH0-CH3
    uint16_t output_states_2;   // Module 1: bits 0-3 = CH0-CH3
    uint16_t oc_flags_1;        // Module 0: overcurrent flags
    uint16_t oc_flags_2;        // Module 1: overcurrent flags
} BMU_BTT6200_Status_Msg_t;

/**
  * @brief  FRAM Statistics message (0x130)
  */
typedef struct __attribute__((packed)) {
    uint32_t sample_count;      // Total temperature samples
    uint32_t alert_count;       // Total alert events
} BMU_FRAM_Stats_Msg_t;

/**
  * @brief  Alarms message (0x140)
  */
typedef struct __attribute__((packed)) {
    uint16_t lem_oc_flags;      // LEM overcurrent flags (bit 0-9)
    uint16_t temp_alerts;       // Temperature alert flags
    uint16_t btt_oc_flags;      // BTT6200 overcurrent flags
    uint16_t system_errors;     // System error flags
} BMU_Alarms_Msg_t;

/**
  * @brief  Power Supply Status message (0x102)
  */
typedef struct __attribute__((packed)) {
    uint8_t pg_5v;              // Power Good 5V (0=Fault, 1=OK)
    uint8_t pg_3v3a;            // Power Good 3.3V Analog
    uint8_t pg_24v;             // Power Good 24V (computed from voltage)
    uint8_t pwr_sleep_state;    // PWR_SLEEP pin state
    uint16_t pwr_voltage_mV;    // Power supply voltage in mV
    uint16_t pwr_current_mA;    // Power supply current in mA
} BMU_PowerSupply_Msg_t;

/**
  * @brief  Input States message (0x103)
  */
typedef struct __attribute__((packed)) {
    uint32_t input_states;      // Bits 0-19: IN_1 to IN_20 (1=HIGH, 0=LOW)
    uint32_t reserved;          // Reserved for future use
} BMU_InputStates_Msg_t;

/**
  * @brief  BTT6200 Detailed Status message (0x124-0x128)
  *         4 outputs per message, with current measurement
  */
typedef struct __attribute__((packed)) {
    uint8_t out0_state;         // Output 0: 0=OFF, 1=ON
    uint8_t out1_state;         // Output 1: 0=OFF, 1=ON
    uint8_t out2_state;         // Output 2: 0=OFF, 1=ON
    uint8_t out3_state;         // Output 3: 0=OFF, 1=ON
    uint16_t out0_current_mA;   // Output 0 current in mA
    uint16_t out1_current_mA;   // Output 1 current in mA
    // Note: message continues in next CAN frame for out2/out3
} BMU_BTT6200_Detailed_Msg_t;

// ==================== COMMAND MESSAGES (RX) ====================

/**
  * @brief  BTT6200 Output Command (0x200)
  *         Control single output ON/OFF
  */
typedef struct __attribute__((packed)) {
    uint8_t output_id;          // Output ID (0-19)
    uint8_t command;            // 0=OFF, 1=ON, 2=TOGGLE
    uint16_t reserved;          // Reserved
    uint32_t magic;             // Safety magic number (0xDEADBEEF)
} BMU_BTT6200_OutputCmd_t;

/**
  * @brief  BTT6200 Multi Command (0x201)
  *         Control multiple outputs using bitmap
  */
typedef struct __attribute__((packed)) {
    uint32_t output_mask;       // Bits 0-19: which outputs to control
    uint32_t output_states;     // Bits 0-19: desired states (1=ON, 0=OFF)
} BMU_BTT6200_MultiCmd_t;

/**
  * @brief  System Command (0x202)
  */
typedef struct __attribute__((packed)) {
    uint8_t command;            // 0=NOP, 1=RESET_STATS, 2=DISABLE_ALL, 3=ENABLE_ALL, 0xFE=REBOOT
    uint8_t reserved[3];
    uint32_t magic;             // Safety magic number (0xCAFEBABE)
} BMU_SystemCmd_t;

/**
  * @brief  CAN handle structure
  */
typedef struct {
    CAN_HandleTypeDef* hcan1;   // CAN1 handle
    CAN_HandleTypeDef* hcan2;   // CAN2 handle (optional)

    // TX/RX buffers
    CAN_TxHeaderTypeDef tx_header;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t tx_data[8];
    uint8_t rx_data[8];
    uint32_t tx_mailbox;

    // Statistics
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;

    // State
    bool is_initialized;

} BMU_CAN_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/

#define BMU_CAN_BAUDRATE            500000      // 500 kbps
#define BMU_CAN_TRANSMIT_TIMEOUT    100         // ms

// System states
#define BMU_STATE_INIT              0
#define BMU_STATE_NORMAL            1
#define BMU_STATE_WARNING           2
#define BMU_STATE_ERROR             3

// Command values
#define BMU_CMD_OUTPUT_OFF          0
#define BMU_CMD_OUTPUT_ON           1
#define BMU_CMD_OUTPUT_TOGGLE       2

#define BMU_CMD_SYSTEM_NOP          0
#define BMU_CMD_SYSTEM_RESET_STATS  1
#define BMU_CMD_SYSTEM_DISABLE_ALL  2
#define BMU_CMD_SYSTEM_ENABLE_ALL   3
#define BMU_CMD_SYSTEM_REBOOT       0xFE

// Safety magic numbers
#define BMU_MAGIC_OUTPUT_CMD        0xDEADBEEF
#define BMU_MAGIC_SYSTEM_CMD        0xCAFEBABE

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Inicializira BMU CAN protokol
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  hcan1: Pointer na CAN1 handle
  * @param  hcan2: Pointer na CAN2 handle (NULL če ni uporabljen)
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_Init(BMU_CAN_HandleTypeDef* handle,
                               CAN_HandleTypeDef* hcan1,
                               CAN_HandleTypeDef* hcan2);

/**
  * @brief  Konfiguriraj CAN na 500 kbps
  * @param  hcan: Pointer na CAN_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_Configure500k(CAN_HandleTypeDef* hcan);

/**
  * @brief  Pošlji BMU Status sporočilo (0x100)
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  msg: Pointer na BMU_Status_Msg_t
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_SendStatus(BMU_CAN_HandleTypeDef* handle,
                                     BMU_Status_Msg_t* msg);

/**
  * @brief  Pošlji Temperature sporočilo (0x101)
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  msg: Pointer na BMU_Temperature_Msg_t
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_SendTemperature(BMU_CAN_HandleTypeDef* handle,
                                          BMU_Temperature_Msg_t* msg);

/**
  * @brief  Pošlji LEM Current sporočilo (0x110-0x112)
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  msg_id: CAN_ID_LEM_CURRENT_1/2/3
  * @param  msg: Pointer na BMU_LEM_Current_Msg_t
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_SendLEMCurrent(BMU_CAN_HandleTypeDef* handle,
                                         uint32_t msg_id,
                                         BMU_LEM_Current_Msg_t* msg);

/**
  * @brief  Pošlji BTT6200 Status sporočilo (0x120-0x122)
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  msg_id: CAN_ID_BTT6200_STATUS_1/2/3
  * @param  msg: Pointer na BMU_BTT6200_Status_Msg_t
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_SendBTTStatus(BMU_CAN_HandleTypeDef* handle,
                                        uint32_t msg_id,
                                        BMU_BTT6200_Status_Msg_t* msg);

/**
  * @brief  Pošlji FRAM Statistics sporočilo (0x130)
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  msg: Pointer na BMU_FRAM_Stats_Msg_t
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_SendFRAMStats(BMU_CAN_HandleTypeDef* handle,
                                       BMU_FRAM_Stats_Msg_t* msg);

/**
  * @brief  Pošlji Alarms sporočilo (0x140)
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  msg: Pointer na BMU_Alarms_Msg_t
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_SendAlarms(BMU_CAN_HandleTypeDef* handle,
                                     BMU_Alarms_Msg_t* msg);

/**
  * @brief  Pošlji Heartbeat sporočilo (0x1FF)
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  counter: Heartbeat counter
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_SendHeartbeat(BMU_CAN_HandleTypeDef* handle,
                                        uint32_t counter);

/**
  * @brief  Nastavi CAN filter
  * @param  hcan: Pointer na CAN_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_ConfigureFilter(CAN_HandleTypeDef* hcan);

/**
  * @brief  Pridobi statistiko CAN komunikacije
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  tx_count: Pointer za TX counter
  * @param  rx_count: Pointer za RX counter
  * @param  error_count: Pointer za Error counter
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_GetStats(BMU_CAN_HandleTypeDef* handle,
                                  uint32_t* tx_count,
                                  uint32_t* rx_count,
                                  uint32_t* error_count);

/**
  * @brief  Pošlji Power Supply Status sporočilo (0x102)
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  msg: Pointer na BMU_PowerSupply_Msg_t
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_SendPowerSupply(BMU_CAN_HandleTypeDef* handle,
                                          BMU_PowerSupply_Msg_t* msg);

/**
  * @brief  Pošlji Input States sporočilo (0x103)
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  msg: Pointer na BMU_InputStates_Msg_t
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_SendInputStates(BMU_CAN_HandleTypeDef* handle,
                                          BMU_InputStates_Msg_t* msg);

/**
  * @brief  Pošlji BTT6200 Detailed Status sporočilo (0x124-0x128)
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  msg_id: CAN_ID_BTT6200_DETAIL_1/2/3/4/5
  * @param  msg: Pointer na BMU_BTT6200_Detailed_Msg_t
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_SendBTTDetailed(BMU_CAN_HandleTypeDef* handle,
                                          uint32_t msg_id,
                                          BMU_BTT6200_Detailed_Msg_t* msg);

/**
  * @brief  Procesira prejeto CAN sporočilo (RX)
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  rx_header: Pointer na CAN_RxHeaderTypeDef
  * @param  rx_data: Pointer na prejete podatke (8 bajtov)
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_ProcessRxMessage(BMU_CAN_HandleTypeDef* handle,
                                           CAN_RxHeaderTypeDef* rx_header,
                                           uint8_t* rx_data);

/**
  * @brief  CAN RX callback - klicana iz interrupt-a
  * @note   To funkcijo mora uporabnik poklicati iz HAL_CAN_RxFifo0MsgPendingCallback
  * @param  hcan: Pointer na CAN_HandleTypeDef
  * @retval None
  */
void BMU_CAN_RxCallback(CAN_HandleTypeDef* hcan);

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
                                  uint8_t* rx_data);

/**
  * @brief  Wait for TX mailbox to become available
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @param  timeout_ms: Timeout v milisekundah
  * @retval HAL status (HAL_OK če je mailbox prosta, HAL_TIMEOUT če timeout)
  */
HAL_StatusTypeDef BMU_CAN_WaitTxMailboxFree(BMU_CAN_HandleTypeDef* handle, uint32_t timeout_ms);

/**
  * @brief  Check CAN error state and recover if needed
  * @param  handle: Pointer na BMU_CAN_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef BMU_CAN_CheckAndRecover(BMU_CAN_HandleTypeDef* handle);

/**
  * @brief  Process CAN RX queue (call from main loop)
  * @retval None
  */
void BMU_CAN_ProcessQueue(void);

#ifdef __cplusplus
}
#endif

#endif /* __BMU_CAN_H */
