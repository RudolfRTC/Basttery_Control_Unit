/**
  ******************************************************************************
  * @file           : temp_logger.c
  * @brief          : Temperature Logger Implementation (FRAM storage)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "temp_logger.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define TEMP_LOGGER_MAGIC_VALUE     0x54454D50  // "TEMP" signature

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef TempLogger_WriteStats(TempLogger_HandleTypeDef* handle);
static HAL_StatusTypeDef TempLogger_ReadStats(TempLogger_HandleTypeDef* handle,
                                              TempLog_Stats_t* stats);
static HAL_StatusTypeDef TempLogger_WriteEntry(TempLogger_HandleTypeDef* handle,
                                               TempLog_Entry_t* entry);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Inicializira temperature logger
  */
HAL_StatusTypeDef TempLogger_Init(TempLogger_HandleTypeDef* handle,
                                  CY15B256J_HandleTypeDef* hfram,
                                  bool reset_stats)
{
    if (handle == NULL || hfram == NULL || !hfram->is_initialized) {
        return HAL_ERROR;
    }

    handle->hfram = hfram;
    handle->log_start_addr = TEMP_LOGGER_LOG_START_ADDR;
    handle->stats_addr = TEMP_LOGGER_STATS_ADDR;
    handle->log_capacity = TEMP_LOGGER_LOG_CAPACITY;
    handle->log_write_index = 0;

    if (reset_stats) {
        // Reset statistike na default vrednosti
        TempLog_Stats_t stats = {0};
        stats.min_temp = 32767;   // MAX int16_t
        stats.max_temp = -32768;  // MIN int16_t

        // Zapiši v FRAM
        HAL_StatusTypeDef status = CY15B256J_Write(hfram,
                                                   handle->stats_addr,
                                                   (uint8_t*)&stats,
                                                   sizeof(TempLog_Stats_t));
        if (status != HAL_OK) return status;

        // Počisti log buffer (opcijsko - FRAM je že prazn po produkciji)
        // CY15B256J_Fill(hfram, handle->log_start_addr, 0xFF,
        //                handle->log_capacity * sizeof(TempLog_Entry_t));
    } else {
        // Preberi obstoječe statistike iz FRAM
        TempLogger_ReadStats(handle);
    }

    handle->is_initialized = true;
    return HAL_OK;
}

/**
  * @brief  Zapiši novo temperaturo v log
  */
HAL_StatusTypeDef TempLogger_LogTemperature(TempLogger_HandleTypeDef* handle,
                                            int16_t temperature,
                                            bool is_alert)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    // Pripravi log entry
    TempLog_Entry_t entry;
    entry.timestamp = HAL_GetTick();
    entry.temperature = temperature;
    entry.alert_flag = is_alert ? 1 : 0;
    entry.reserved = 0;

    // Zapiši entry v FRAM (circular buffer)
    HAL_StatusTypeDef status = TempLogger_WriteEntry(handle, &entry);
    if (status != HAL_OK) return status;

    // Posodobi statistike - preberi trenutne vrednosti iz FRAM
    TempLog_Stats_t stats;
    status = TempLogger_ReadStats(handle, &stats);
    if (status != HAL_OK) return status;

    stats.current_temp = temperature;
    stats.sample_count++;
    stats.last_update = entry.timestamp;

    if (temperature < stats.min_temp) {
        stats.min_temp = temperature;
    }
    if (temperature > stats.max_temp) {
        stats.max_temp = temperature;
    }
    if (is_alert) {
        stats.alert_count++;
    }

    // Zapiši posodobljene statistike v FRAM
    status = CY15B256J_Write(handle->hfram,
                            handle->stats_addr,
                            (uint8_t*)&stats,
                            sizeof(TempLog_Stats_t));

    // Increment circular buffer write index
    handle->log_write_index++;
    if (handle->log_write_index >= handle->log_capacity) {
        handle->log_write_index = 0;
    }

    return status;
}

/**
  * @brief  Preberi statistike iz FRAM
  */
HAL_StatusTypeDef TempLogger_GetStats(TempLogger_HandleTypeDef* handle,
                                     TempLog_Stats_t* stats)
{
    if (handle == NULL || stats == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    return CY15B256J_Read(handle->hfram,
                         handle->stats_addr,
                         (uint8_t*)stats,
                         sizeof(TempLog_Stats_t));
}

/**
  * @brief  Preberi log entry iz FRAM
  */
HAL_StatusTypeDef TempLogger_GetEntry(TempLogger_HandleTypeDef* handle,
                                     uint16_t index,
                                     TempLog_Entry_t* entry)
{
    if (handle == NULL || entry == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    if (index >= handle->log_capacity) {
        return HAL_ERROR;
    }

    uint16_t address = handle->log_start_addr + (index * sizeof(TempLog_Entry_t));

    return CY15B256J_Read(handle->hfram,
                         address,
                         (uint8_t*)entry,
                         sizeof(TempLog_Entry_t));
}

/**
  * @brief  Reset statistike
  */
HAL_StatusTypeDef TempLogger_ResetStats(TempLogger_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return HAL_ERROR;
    }

    TempLog_Stats_t stats = {0};
    stats.min_temp = 32767;   // MAX int16_t
    stats.max_temp = -32768;  // MIN int16_t

    return CY15B256J_Write(handle->hfram,
                          handle->stats_addr,
                          (uint8_t*)&stats,
                          sizeof(TempLog_Stats_t));
}

/**
  * @brief  Pridobi število shranjenih log entries
  */
uint16_t TempLogger_GetLogCount(TempLogger_HandleTypeDef* handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return 0;
    }

    TempLog_Stats_t stats;
    if (TempLogger_GetStats(handle, &stats) != HAL_OK) {
        return 0;
    }

    // Vrni min(sample_count, capacity)
    return (stats.sample_count < handle->log_capacity) ?
           stats.sample_count : handle->log_capacity;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Zapiši entry v circular buffer
  */
static HAL_StatusTypeDef TempLogger_WriteEntry(TempLogger_HandleTypeDef* handle,
                                               TempLog_Entry_t* entry)
{
    uint16_t address = handle->log_start_addr +
                      (handle->log_write_index * sizeof(TempLog_Entry_t));

    return CY15B256J_Write(handle->hfram,
                          address,
                          (uint8_t*)entry,
                          sizeof(TempLog_Entry_t));
}

/**
  * @brief  Preberi statistike (internal)
  */
static HAL_StatusTypeDef TempLogger_ReadStats(TempLogger_HandleTypeDef* handle,
                                              TempLog_Stats_t* stats)
{
    if (stats == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = CY15B256J_Read(handle->hfram,
                                             handle->stats_addr,
                                             (uint8_t*)stats,
                                             sizeof(TempLog_Stats_t));
    return status;
}

/**
  * @brief  Zapiši statistike (internal)
  */
static HAL_StatusTypeDef TempLogger_WriteStats(TempLogger_HandleTypeDef* handle)
{
    // Ta funkcija je rezervirana za bodočo uporabo
    return HAL_OK;
}
