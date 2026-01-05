/**
  ******************************************************************************
  * @file           : temp_logger.h
  * @brief          : Temperature Logger Header (FRAM storage)
  * @author         : BMU IOC Project
  * @date           : 2025
  ******************************************************************************
  * @attention
  *
  * Shranjuje temperature podatke v FRAM (CY15B256J):
  * - Trenutna, min, max temperatura
  * - Circular log buffer (zadnjih 100 meritev)
  * - Število alarmov (temp < 0°C)
  * - Timestamp (HAL_GetTick())
  *
  ******************************************************************************
  */

#ifndef __TEMP_LOGGER_H
#define __TEMP_LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cy15b256j.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Temperature log entry struktura
  */
typedef struct {
    uint32_t timestamp;      // HAL_GetTick() v ms
    int16_t temperature;     // Temperatura × 100 (npr. 2534 = 25.34°C)
    uint8_t alert_flag;      // 1 = alarm (<0°C), 0 = normalno
    uint8_t reserved;        // Rezervirano za alignment
} __attribute__((packed)) TempLog_Entry_t;

/**
  * @brief  Temperature logger statistics
  */
typedef struct {
    int16_t current_temp;    // Trenutna temperatura × 100
    int16_t min_temp;        // Min temperatura × 100
    int16_t max_temp;        // Max temperatura × 100
    uint32_t alert_count;    // Število alarmov
    uint32_t sample_count;   // Število vseh meritev
    uint32_t last_update;    // Zadnji timestamp
} __attribute__((packed)) TempLog_Stats_t;

/**
  * @brief  Temperature logger handle
  */
typedef struct {
    CY15B256J_HandleTypeDef* hfram;     // FRAM handle
    uint16_t log_start_addr;            // Start address za log buffer
    uint16_t stats_addr;                // Address za statistics
    uint16_t log_capacity;              // Max število log entries
    uint16_t log_write_index;           // Trenutni write index (circular)
    bool is_initialized;                // Initialization flag
} TempLogger_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/

// FRAM memory map
#define TEMP_LOGGER_STATS_ADDR      0x0000      // Statistics at address 0
#define TEMP_LOGGER_LOG_START_ADDR  0x0100      // Log buffer starts at 0x0100
#define TEMP_LOGGER_LOG_CAPACITY    100         // 100 entries

// Velikost struktur
#define TEMP_LOGGER_ENTRY_SIZE      sizeof(TempLog_Entry_t)   // 8 bytes
#define TEMP_LOGGER_STATS_SIZE      sizeof(TempLog_Stats_t)   // 20 bytes

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Inicializira temperature logger
  * @param  handle: Pointer na TempLogger_HandleTypeDef
  * @param  hfram: Pointer na CY15B256J_HandleTypeDef
  * @param  reset_stats: true = reset statistike, false = preberi iz FRAM
  * @retval HAL status
  */
HAL_StatusTypeDef TempLogger_Init(TempLogger_HandleTypeDef* handle,
                                  CY15B256J_HandleTypeDef* hfram,
                                  bool reset_stats);

/**
  * @brief  Zapiši novo temperaturo v log
  * @param  handle: Pointer na TempLogger_HandleTypeDef
  * @param  temperature: Temperatura × 100 (npr. 2534 = 25.34°C)
  * @param  is_alert: true = alarm (<0°C), false = normalno
  * @retval HAL status
  */
HAL_StatusTypeDef TempLogger_LogTemperature(TempLogger_HandleTypeDef* handle,
                                            int16_t temperature,
                                            bool is_alert);

/**
  * @brief  Preberi statistike iz FRAM
  * @param  handle: Pointer na TempLogger_HandleTypeDef
  * @param  stats: Pointer na TempLog_Stats_t za rezultat
  * @retval HAL status
  */
HAL_StatusTypeDef TempLogger_GetStats(TempLogger_HandleTypeDef* handle,
                                     TempLog_Stats_t* stats);

/**
  * @brief  Preberi log entry iz FRAM
  * @param  handle: Pointer na TempLogger_HandleTypeDef
  * @param  index: Index (0 = najstarejši, capacity-1 = najnovejši)
  * @param  entry: Pointer na TempLog_Entry_t za rezultat
  * @retval HAL status
  */
HAL_StatusTypeDef TempLogger_GetEntry(TempLogger_HandleTypeDef* handle,
                                     uint16_t index,
                                     TempLog_Entry_t* entry);

/**
  * @brief  Reset statistike (min/max/counters)
  * @param  handle: Pointer na TempLogger_HandleTypeDef
  * @retval HAL status
  */
HAL_StatusTypeDef TempLogger_ResetStats(TempLogger_HandleTypeDef* handle);

/**
  * @brief  Pridobi število shranjenih log entries
  * @param  handle: Pointer na TempLogger_HandleTypeDef
  * @retval Število entries (0-capacity)
  */
uint16_t TempLogger_GetLogCount(TempLogger_HandleTypeDef* handle);

#ifdef __cplusplus
}
#endif

#endif /* __TEMP_LOGGER_H */
