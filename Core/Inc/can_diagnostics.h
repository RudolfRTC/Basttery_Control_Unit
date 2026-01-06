/**
  ******************************************************************************
  * @file           : can_diagnostics.h
  * @brief          : CAN Bus Diagnostic Tools
  ******************************************************************************
  */

#ifndef __CAN_DIAGNOSTICS_H
#define __CAN_DIAGNOSTICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/**
  * @brief  Run complete CAN diagnostics
  * @param  hcan: CAN handle
  * @param  huart: UART handle for debug output
  * @retval None
  */
void CAN_RunDiagnostics(CAN_HandleTypeDef* hcan, UART_HandleTypeDef* huart);

/**
  * @brief  Test CAN loopback mode
  * @param  hcan: CAN handle
  * @param  huart: UART handle for debug output
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_TestLoopback(CAN_HandleTypeDef* hcan, UART_HandleTypeDef* huart);

/**
  * @brief  Check NVIC interrupt status
  * @param  huart: UART handle for debug output
  * @retval None
  */
void CAN_CheckNVIC(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_DIAGNOSTICS_H */
