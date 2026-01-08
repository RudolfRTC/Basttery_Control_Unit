/**
  ******************************************************************************
  * @file           : safety.h
  * @brief          : ISO 26262 Safety Mechanisms Header
  * @author         : BMU IOC Project
  * @date           : 2026-01-08
  ******************************************************************************
  * @attention
  *
  * ISO 26262 ASIL-B Safety Mechanisms for Battery Management Unit
  * - CRC checking for data integrity
  * - Watchdog support
  * - Range validation
  * - Safe state management
  * - Diagnostic functions
  *
  ******************************************************************************
  */

#ifndef __SAFETY_H
#define __SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  System Safety State
  */
typedef enum {
    SAFETY_STATE_INIT           = 0x00U,  // Initialization state
    SAFETY_STATE_NORMAL         = 0x01U,  // Normal operation
    SAFETY_STATE_WARNING        = 0x02U,  // Warning - degraded operation
    SAFETY_STATE_ERROR          = 0x03U,  // Error - limited operation
    SAFETY_STATE_SAFE           = 0x04U,  // Safe state - all outputs disabled
    SAFETY_STATE_FAULT          = 0x05U   // Fatal fault - system halt
} Safety_State_t;

/**
  * @brief  Safety Error Codes
  */
typedef enum {
    SAFETY_ERROR_NONE           = 0x0000U,
    SAFETY_ERROR_CRC            = 0x0001U,  // CRC check failed
    SAFETY_ERROR_RANGE          = 0x0002U,  // Value out of range
    SAFETY_ERROR_TIMEOUT        = 0x0004U,  // Operation timeout
    SAFETY_ERROR_SEQUENCE       = 0x0008U,  // Invalid call sequence
    SAFETY_ERROR_PARAM          = 0x0010U,  // Invalid parameter
    SAFETY_ERROR_STACK          = 0x0020U,  // Stack overflow detected
    SAFETY_ERROR_MEMORY         = 0x0040U,  // Memory corruption detected
    SAFETY_ERROR_WATCHDOG       = 0x0080U,  // Watchdog timeout
    SAFETY_ERROR_CAN_BUSOFF     = 0x0100U,  // CAN bus-off state
    SAFETY_ERROR_OVERCURRENT    = 0x0200U,  // Overcurrent detected
    SAFETY_ERROR_TEMP_HIGH      = 0x0400U,  // Temperature too high
    SAFETY_ERROR_TEMP_LOW       = 0x0800U,  // Temperature too low
    SAFETY_ERROR_VOLTAGE_HIGH   = 0x1000U,  // Voltage too high
    SAFETY_ERROR_VOLTAGE_LOW    = 0x2000U   // Voltage too low
} Safety_ErrorCode_t;

/**
  * @brief  Safety Statistics
  */
typedef struct {
    uint32_t total_errors;              // Total error count
    uint32_t crc_errors;                // CRC check failures
    uint32_t range_errors;              // Range validation failures
    uint32_t timeout_errors;            // Timeout errors
    uint32_t watchdog_refreshes;        // Watchdog refresh count
    uint32_t safe_state_entries;        // Number of times safe state entered
    uint32_t self_test_runs;            // Self-test execution count
    uint32_t self_test_failures;        // Self-test failure count
} Safety_Statistics_t;

/**
  * @brief  Safety Configuration
  */
typedef struct {
    bool enable_crc_check;              // Enable CRC checking
    bool enable_range_check;            // Enable range validation
    bool enable_watchdog;               // Enable watchdog monitoring
    bool enable_stack_check;            // Enable stack overflow detection
    uint32_t watchdog_timeout_ms;       // Watchdog timeout in milliseconds
    uint32_t safe_state_timeout_ms;     // Timeout before entering safe state
} Safety_Config_t;

/**
  * @brief  Safety Handle
  */
typedef struct {
    Safety_State_t current_state;       // Current safety state
    Safety_State_t previous_state;      // Previous safety state
    Safety_ErrorCode_t active_errors;   // Active error flags (bitfield)
    Safety_Statistics_t statistics;     // Safety statistics
    Safety_Config_t config;             // Safety configuration
    uint32_t last_watchdog_refresh;     // Last watchdog refresh timestamp
    uint32_t error_entry_time;          // Timestamp when error state entered
    bool is_initialized;                // Initialization flag
} Safety_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/

/* ISO 26262 ASIL Classification */
#define SAFETY_ASIL_LEVEL           'B'  // ASIL-B classification

/* CRC-8 Polynomial (SAE J1850) */
#define SAFETY_CRC8_POLY            0x1DU
#define SAFETY_CRC8_INIT            0xFFU

/* Range Limits */
#define SAFETY_TEMP_MIN_C           (-40)    // Minimum temperature (°C)
#define SAFETY_TEMP_MAX_C           (85)     // Maximum temperature (°C)
#define SAFETY_VOLTAGE_MIN_MV       (18000U) // Minimum voltage (18V)
#define SAFETY_VOLTAGE_MAX_MV       (30000U) // Maximum voltage (30V)
#define SAFETY_CURRENT_MAX_MA       (100000U)// Maximum current (100A)

/* Error Thresholds */
#define SAFETY_MAX_CRC_ERRORS       (10U)    // Max CRC errors before safe state
#define SAFETY_MAX_RANGE_ERRORS     (20U)    // Max range errors before safe state
#define SAFETY_MAX_TIMEOUT_ERRORS   (5U)     // Max timeout errors before safe state

/* Watchdog Settings */
#define SAFETY_WATCHDOG_TIMEOUT_MS  (1000U)  // 1 second watchdog timeout
#define SAFETY_WATCHDOG_REFRESH_MIN (100U)   // Minimum refresh interval (ms)

/* Exported macro ------------------------------------------------------------*/

/**
  * @brief  Check if value is in valid range
  */
#define SAFETY_IS_IN_RANGE(val, min, max)  (((val) >= (min)) && ((val) <= (max)))

/**
  * @brief  Set error flag
  */
#define SAFETY_SET_ERROR(handle, error)     ((handle)->active_errors |= (error))

/**
  * @brief  Clear error flag
  */
#define SAFETY_CLEAR_ERROR(handle, error)   ((handle)->active_errors &= ~(error))

/**
  * @brief  Check if error is active
  */
#define SAFETY_HAS_ERROR(handle, error)     (((handle)->active_errors & (error)) != 0U)

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Initialize safety mechanisms
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @param  config: Pointer to Safety_Config_t
  * @retval HAL status
  */
HAL_StatusTypeDef Safety_Init(Safety_HandleTypeDef* handle,
                               const Safety_Config_t* config);

/**
  * @brief  Execute periodic safety checks
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef Safety_PeriodicCheck(Safety_HandleTypeDef* handle);

/**
  * @brief  Calculate CRC-8 checksum
  * @param  data: Pointer to data buffer
  * @param  length: Length of data in bytes
  * @retval CRC-8 checksum
  */
uint8_t Safety_CalculateCRC8(const uint8_t* data, uint16_t length);

/**
  * @brief  Verify CRC-8 checksum
  * @param  data: Pointer to data buffer (including CRC byte at end)
  * @param  length: Total length including CRC byte
  * @retval true if CRC valid, false otherwise
  */
bool Safety_VerifyCRC8(const uint8_t* data, uint16_t length);

/**
  * @brief  Validate temperature reading
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @param  temp_C: Temperature in Celsius
  * @retval true if valid, false if out of range
  */
bool Safety_ValidateTemperature(Safety_HandleTypeDef* handle, int16_t temp_C);

/**
  * @brief  Validate voltage reading
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @param  voltage_mV: Voltage in millivolts
  * @retval true if valid, false if out of range
  */
bool Safety_ValidateVoltage(Safety_HandleTypeDef* handle, uint16_t voltage_mV);

/**
  * @brief  Validate current reading
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @param  current_mA: Current in milliamps
  * @retval true if valid, false if out of range
  */
bool Safety_ValidateCurrent(Safety_HandleTypeDef* handle, int32_t current_mA);

/**
  * @brief  Refresh watchdog timer
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef Safety_RefreshWatchdog(Safety_HandleTypeDef* handle);

/**
  * @brief  Enter safe state
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @param  error_code: Error code causing safe state entry
  * @retval HAL status
  */
HAL_StatusTypeDef Safety_EnterSafeState(Safety_HandleTypeDef* handle,
                                        Safety_ErrorCode_t error_code);

/**
  * @brief  Attempt recovery from error state
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef Safety_AttemptRecovery(Safety_HandleTypeDef* handle);

/**
  * @brief  Get current safety state
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @retval Current safety state
  */
Safety_State_t Safety_GetState(const Safety_HandleTypeDef* handle);

/**
  * @brief  Get active error codes
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @retval Active error flags (bitfield)
  */
Safety_ErrorCode_t Safety_GetActiveErrors(const Safety_HandleTypeDef* handle);

/**
  * @brief  Get safety statistics
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @param  stats: Pointer to Safety_Statistics_t to receive statistics
  * @retval HAL status
  */
HAL_StatusTypeDef Safety_GetStatistics(const Safety_HandleTypeDef* handle,
                                       Safety_Statistics_t* stats);

/**
  * @brief  Reset safety statistics
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef Safety_ResetStatistics(Safety_HandleTypeDef* handle);

/**
  * @brief  Execute self-test
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef Safety_SelfTest(Safety_HandleTypeDef* handle);

/**
  * @brief  Check stack integrity
  * @retval true if stack OK, false if overflow detected
  */
bool Safety_CheckStackIntegrity(void);

/**
  * @brief  Report error to safety system
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @param  error_code: Error code to report
  * @retval HAL status
  */
HAL_StatusTypeDef Safety_ReportError(Safety_HandleTypeDef* handle,
                                     Safety_ErrorCode_t error_code);

/**
  * @brief  Clear specific error
  * @param  handle: Pointer to Safety_HandleTypeDef
  * @param  error_code: Error code to clear
  * @retval HAL status
  */
HAL_StatusTypeDef Safety_ClearError(Safety_HandleTypeDef* handle,
                                    Safety_ErrorCode_t error_code);

#ifdef __cplusplus
}
#endif

#endif /* __SAFETY_H */
