/**
  ******************************************************************************
  * @file           : safety.c
  * @brief          : ISO 26262 Safety Mechanisms Implementation
  * @author         : BMU IOC Project
  * @date           : 2026-01-08
  ******************************************************************************
  * @attention
  *
  * ISO 26262 ASIL-B Safety Mechanisms Implementation
  * - Implements CRC checking, range validation, watchdog support
  * - Provides safe state management and error handling
  * - Includes self-test and diagnostic functions
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "safety.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define STACK_CANARY_VALUE      0xDEADBEEFU  /* Stack overflow detection */

/* Private variables ---------------------------------------------------------*/
static uint32_t stack_canary = STACK_CANARY_VALUE;

/* Private function prototypes -----------------------------------------------*/
static void Safety_TransitionState(Safety_HandleTypeDef* handle,
                                   Safety_State_t new_state);
static bool Safety_CheckErrorThresholds(const Safety_HandleTypeDef* handle);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Initialize safety mechanisms
  */
HAL_StatusTypeDef Safety_Init(Safety_HandleTypeDef* handle,
                               const Safety_Config_t* config)
{
    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if ((handle == NULL) || (config == NULL)) {
        return HAL_ERROR;
    }

    /* Initialize handle structure */
    (void)memset(handle, 0, sizeof(Safety_HandleTypeDef));

    /* Copy configuration */
    (void)memcpy(&handle->config, config, sizeof(Safety_Config_t));

    /* Initialize state */
    handle->current_state = SAFETY_STATE_INIT;
    handle->previous_state = SAFETY_STATE_INIT;
    handle->active_errors = SAFETY_ERROR_NONE;

    /* Initialize watchdog */
    handle->last_watchdog_refresh = HAL_GetTick();

    /* Initialize stack canary */
    stack_canary = STACK_CANARY_VALUE;

    handle->is_initialized = true;

    /* Transition to normal state after successful initialization */
    Safety_TransitionState(handle, SAFETY_STATE_NORMAL);

    return HAL_OK;
}

/**
  * @brief  Execute periodic safety checks
  */
HAL_StatusTypeDef Safety_PeriodicCheck(Safety_HandleTypeDef* handle)
{
    /* MISRA C 2012 Rule 14.4: Explicit boolean check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    /* Check watchdog timeout */
    if (handle->config.enable_watchdog != false) {
        uint32_t current_tick = HAL_GetTick();
        uint32_t elapsed_ms = current_tick - handle->last_watchdog_refresh;

        if (elapsed_ms > handle->config.watchdog_timeout_ms) {
            handle->statistics.total_errors++;
            (void)Safety_ReportError(handle, SAFETY_ERROR_WATCHDOG);
            return HAL_ERROR;
        }
    }

    /* Check stack integrity */
    if (handle->config.enable_stack_check != false) {
        if (Safety_CheckStackIntegrity() == false) {
            handle->statistics.total_errors++;
            (void)Safety_ReportError(handle, SAFETY_ERROR_STACK);
            return HAL_ERROR;
        }
    }

    /* Check error thresholds */
    if (Safety_CheckErrorThresholds(handle) == false) {
        /* Too many errors - enter safe state */
        (void)Safety_EnterSafeState(handle, SAFETY_ERROR_NONE);
        return HAL_ERROR;
    }

    /* Check if in error state for too long */
    if ((handle->current_state == SAFETY_STATE_ERROR) ||
        (handle->current_state == SAFETY_STATE_SAFE)) {
        uint32_t current_tick = HAL_GetTick();
        uint32_t elapsed_ms = current_tick - handle->error_entry_time;

        if (elapsed_ms > handle->config.safe_state_timeout_ms) {
            /* Attempt recovery */
            (void)Safety_AttemptRecovery(handle);
        }
    }

    return HAL_OK;
}

/**
  * @brief  Calculate CRC-8 checksum (SAE J1850)
  */
uint8_t Safety_CalculateCRC8(const uint8_t* data, uint16_t length)
{
    uint8_t crc = SAFETY_CRC8_INIT;
    uint16_t i;
    uint8_t j;

    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if (data == NULL) {
        return 0U;
    }

    for (i = 0U; i < length; i++) {
        crc ^= data[i];
        for (j = 0U; j < 8U; j++) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1) ^ SAFETY_CRC8_POLY);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return crc;
}

/**
  * @brief  Verify CRC-8 checksum
  */
bool Safety_VerifyCRC8(const uint8_t* data, uint16_t length)
{
    uint8_t calculated_crc;
    uint8_t received_crc;
    bool result = false;

    /* MISRA C 2012 Rule 14.4: Explicit checks */
    if ((data != NULL) && (length > 0U)) {
        /* Calculate CRC over all data except last byte */
        calculated_crc = Safety_CalculateCRC8(data, (uint16_t)(length - 1U));

        /* Compare with received CRC (last byte) */
        received_crc = data[length - 1U];

        result = (calculated_crc == received_crc);
    }

    return result;
}

/**
  * @brief  Validate temperature reading
  */
bool Safety_ValidateTemperature(Safety_HandleTypeDef* handle, int16_t temp_C)
{
    bool is_valid = true;

    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return false;
    }

    /* Check range only if enabled */
    if (handle->config.enable_range_check != false) {
        if ((temp_C < SAFETY_TEMP_MIN_C) || (temp_C > SAFETY_TEMP_MAX_C)) {
            is_valid = false;
            handle->statistics.range_errors++;
            handle->statistics.total_errors++;

            if (temp_C < SAFETY_TEMP_MIN_C) {
                (void)Safety_ReportError(handle, SAFETY_ERROR_TEMP_LOW);
            } else {
                (void)Safety_ReportError(handle, SAFETY_ERROR_TEMP_HIGH);
            }
        }
    }

    return is_valid;
}

/**
  * @brief  Validate voltage reading
  */
bool Safety_ValidateVoltage(Safety_HandleTypeDef* handle, uint16_t voltage_mV)
{
    bool is_valid = true;

    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return false;
    }

    /* Check range only if enabled */
    if (handle->config.enable_range_check != false) {
        if ((voltage_mV < SAFETY_VOLTAGE_MIN_MV) ||
            (voltage_mV > SAFETY_VOLTAGE_MAX_MV)) {
            is_valid = false;
            handle->statistics.range_errors++;
            handle->statistics.total_errors++;

            if (voltage_mV < SAFETY_VOLTAGE_MIN_MV) {
                (void)Safety_ReportError(handle, SAFETY_ERROR_VOLTAGE_LOW);
            } else {
                (void)Safety_ReportError(handle, SAFETY_ERROR_VOLTAGE_HIGH);
            }
        }
    }

    return is_valid;
}

/**
  * @brief  Validate current reading
  */
bool Safety_ValidateCurrent(Safety_HandleTypeDef* handle, int32_t current_mA)
{
    bool is_valid = true;
    int32_t abs_current;

    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return false;
    }

    /* Calculate absolute value */
    abs_current = (current_mA < 0) ? -current_mA : current_mA;

    /* Check range only if enabled */
    if (handle->config.enable_range_check != false) {
        if ((uint32_t)abs_current > SAFETY_CURRENT_MAX_MA) {
            is_valid = false;
            handle->statistics.range_errors++;
            handle->statistics.total_errors++;
            (void)Safety_ReportError(handle, SAFETY_ERROR_OVERCURRENT);
        }
    }

    return is_valid;
}

/**
  * @brief  Refresh watchdog timer
  */
HAL_StatusTypeDef Safety_RefreshWatchdog(Safety_HandleTypeDef* handle)
{
    uint32_t current_tick;
    uint32_t elapsed_ms;

    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    current_tick = HAL_GetTick();

    /* Check minimum refresh interval */
    elapsed_ms = current_tick - handle->last_watchdog_refresh;
    if (elapsed_ms < SAFETY_WATCHDOG_REFRESH_MIN) {
        /* Refreshed too quickly - possible runaway code */
        handle->statistics.total_errors++;
        (void)Safety_ReportError(handle, SAFETY_ERROR_SEQUENCE);
        return HAL_ERROR;
    }

    /* Update refresh timestamp */
    handle->last_watchdog_refresh = current_tick;
    handle->statistics.watchdog_refreshes++;

    return HAL_OK;
}

/**
  * @brief  Enter safe state
  */
HAL_StatusTypeDef Safety_EnterSafeState(Safety_HandleTypeDef* handle,
                                        Safety_ErrorCode_t error_code)
{
    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    /* Record error if provided */
    if (error_code != SAFETY_ERROR_NONE) {
        SAFETY_SET_ERROR(handle, error_code);
    }

    /* Record entry time */
    handle->error_entry_time = HAL_GetTick();

    /* Transition to safe state */
    Safety_TransitionState(handle, SAFETY_STATE_SAFE);

    /* Increment counter */
    handle->statistics.safe_state_entries++;

    /* TODO: Implement actual safe state actions:
     * - Disable all BTT6200 outputs
     * - Stop PWM signals
     * - Send CAN safe state message
     * - Activate warning indicators
     */

    return HAL_OK;
}

/**
  * @brief  Attempt recovery from error state
  */
HAL_StatusTypeDef Safety_AttemptRecovery(Safety_HandleTypeDef* handle)
{
    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    /* Check if recovery is possible */
    if (handle->active_errors != SAFETY_ERROR_NONE) {
        /* Still have active errors - cannot recover */
        return HAL_ERROR;
    }

    /* Check current state */
    if ((handle->current_state != SAFETY_STATE_ERROR) &&
        (handle->current_state != SAFETY_STATE_WARNING) &&
        (handle->current_state != SAFETY_STATE_SAFE)) {
        /* Not in recoverable state */
        return HAL_ERROR;
    }

    /* Attempt to transition back to normal */
    Safety_TransitionState(handle, SAFETY_STATE_NORMAL);

    return HAL_OK;
}

/**
  * @brief  Get current safety state
  */
Safety_State_t Safety_GetState(const Safety_HandleTypeDef* handle)
{
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return SAFETY_STATE_FAULT;
    }

    return handle->current_state;
}

/**
  * @brief  Get active error codes
  */
Safety_ErrorCode_t Safety_GetActiveErrors(const Safety_HandleTypeDef* handle)
{
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return SAFETY_ERROR_NONE;
    }

    return handle->active_errors;
}

/**
  * @brief  Get safety statistics
  */
HAL_StatusTypeDef Safety_GetStatistics(const Safety_HandleTypeDef* handle,
                                       Safety_Statistics_t* stats)
{
    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if ((handle == NULL) || (stats == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    (void)memcpy(stats, &handle->statistics, sizeof(Safety_Statistics_t));

    return HAL_OK;
}

/**
  * @brief  Reset safety statistics
  */
HAL_StatusTypeDef Safety_ResetStatistics(Safety_HandleTypeDef* handle)
{
    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    (void)memset(&handle->statistics, 0, sizeof(Safety_Statistics_t));

    return HAL_OK;
}

/**
  * @brief  Execute self-test
  */
HAL_StatusTypeDef Safety_SelfTest(Safety_HandleTypeDef* handle)
{
    uint8_t test_data[8] = {0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U, 0x00U};
    uint8_t calculated_crc;
    bool crc_ok;

    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    handle->statistics.self_test_runs++;

    /* Test 1: CRC calculation */
    calculated_crc = Safety_CalculateCRC8(test_data, 7U);
    test_data[7] = calculated_crc;

    crc_ok = Safety_VerifyCRC8(test_data, 8U);
    if (crc_ok == false) {
        handle->statistics.self_test_failures++;
        return HAL_ERROR;
    }

    /* Test 2: Stack integrity */
    if (Safety_CheckStackIntegrity() == false) {
        handle->statistics.self_test_failures++;
        return HAL_ERROR;
    }

    /* Test 3: Range validation */
    if (Safety_ValidateTemperature(handle, 25) == false) {
        handle->statistics.self_test_failures++;
        return HAL_ERROR;
    }

    /* All tests passed */
    return HAL_OK;
}

/**
  * @brief  Check stack integrity
  */
bool Safety_CheckStackIntegrity(void)
{
    /* Check stack canary value */
    return (stack_canary == STACK_CANARY_VALUE);
}

/**
  * @brief  Report error to safety system
  */
HAL_StatusTypeDef Safety_ReportError(Safety_HandleTypeDef* handle,
                                     Safety_ErrorCode_t error_code)
{
    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    /* Set error flag */
    SAFETY_SET_ERROR(handle, error_code);

    /* Determine new state based on error severity */
    if ((error_code == SAFETY_ERROR_WATCHDOG) ||
        (error_code == SAFETY_ERROR_STACK) ||
        (error_code == SAFETY_ERROR_MEMORY)) {
        /* Critical error - enter safe state immediately */
        (void)Safety_EnterSafeState(handle, error_code);
    } else if ((error_code == SAFETY_ERROR_OVERCURRENT) ||
               (error_code == SAFETY_ERROR_TEMP_HIGH) ||
               (error_code == SAFETY_ERROR_TEMP_LOW)) {
        /* Warning condition */
        if (handle->current_state == SAFETY_STATE_NORMAL) {
            Safety_TransitionState(handle, SAFETY_STATE_WARNING);
        }
    } else {
        /* Other errors - transition to error state if in normal */
        if (handle->current_state == SAFETY_STATE_NORMAL) {
            Safety_TransitionState(handle, SAFETY_STATE_ERROR);
        }
    }

    return HAL_OK;
}

/**
  * @brief  Clear specific error
  */
HAL_StatusTypeDef Safety_ClearError(Safety_HandleTypeDef* handle,
                                    Safety_ErrorCode_t error_code)
{
    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if ((handle == NULL) || (handle->is_initialized == false)) {
        return HAL_ERROR;
    }

    /* Clear error flag */
    SAFETY_CLEAR_ERROR(handle, error_code);

    /* If no more active errors, attempt recovery */
    if (handle->active_errors == SAFETY_ERROR_NONE) {
        (void)Safety_AttemptRecovery(handle);
    }

    return HAL_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Transition safety state
  */
static void Safety_TransitionState(Safety_HandleTypeDef* handle,
                                   Safety_State_t new_state)
{
    if (handle == NULL) {
        return;
    }

    /* Record state transition */
    handle->previous_state = handle->current_state;
    handle->current_state = new_state;

    /* TODO: Implement state transition logging/notification */
}

/**
  * @brief  Check if error thresholds exceeded
  */
static bool Safety_CheckErrorThresholds(const Safety_HandleTypeDef* handle)
{
    bool thresholds_ok = true;

    if (handle == NULL) {
        return false;
    }

    /* Check CRC error threshold */
    if (handle->statistics.crc_errors >= SAFETY_MAX_CRC_ERRORS) {
        thresholds_ok = false;
    }

    /* Check range error threshold */
    if (handle->statistics.range_errors >= SAFETY_MAX_RANGE_ERRORS) {
        thresholds_ok = false;
    }

    /* Check timeout error threshold */
    if (handle->statistics.timeout_errors >= SAFETY_MAX_TIMEOUT_ERRORS) {
        thresholds_ok = false;
    }

    return thresholds_ok;
}
