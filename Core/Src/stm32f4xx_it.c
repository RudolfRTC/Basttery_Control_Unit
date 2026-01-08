/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dcdc_controller.h"
#include "dcdc_diagnostics.h"
#include "bmu_can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Forward declaration of BMU CAN message processor for ISR */
extern void BMU_CAN_ProcessRxMessageISR(CAN_HandleTypeDef* hcan,
                                         CAN_RxHeaderTypeDef* rx_header,
                                         uint8_t* rx_data);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  /* MISRA C 2012 Rule 15.6: Infinite loop with explicit condition */
  for (;;)
  {
    /* Intentional infinite loop - NMI fault condition */
    (void)0;
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  /* MISRA C 2012 Rule 15.6: Infinite loop with explicit condition */
  for (;;)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* Intentional infinite loop - Hard Fault condition */
    (void)0;
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  /* MISRA C 2012 Rule 15.6: Infinite loop with explicit condition */
  for (;;)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* Intentional infinite loop - Memory Management Fault condition */
    (void)0;
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  /* MISRA C 2012 Rule 15.6: Infinite loop with explicit condition */
  for (;;)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* Intentional infinite loop - Bus Fault condition */
    (void)0;
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  /* MISRA C 2012 Rule 15.6: Infinite loop with explicit condition */
  for (;;)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* Intentional infinite loop - Usage Fault condition */
    (void)0;
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/**
  * @brief This function handles DMA2 stream0 global interrupt (ADC1 DMA).
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 TX interrupt.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupt.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN2 TX interrupt.
  */
void CAN2_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_TX_IRQn 0 */

  /* USER CODE END CAN2_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_TX_IRQn 1 */

  /* USER CODE END CAN2_TX_IRQn 1 */
}

/**
  * @brief  CAN RX FIFO 0 message pending callback
  * @param  hcan: Pointer to CAN_HandleTypeDef structure
  * @retval None
  * @note   MISRA C 2012 compliant implementation
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint32_t can_id;
    HAL_StatusTypeDef status;

    /* Debug: UART diagnostika za CAN RX */
    extern UART_HandleTypeDef huart1;
    char debug_buf[80];

    /* MISRA C 2012 Rule 14.4: Explicit NULL check */
    if (hcan == NULL) {
        return;
    }

    /* Get received message */
    status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if (status != HAL_OK) {
        return;
    }

    /* Extract CAN ID (standard or extended) */
    /* MISRA C 2012 Rule 14.4: Explicit comparison */
    if (rx_header.IDE == CAN_ID_STD) {
        can_id = rx_header.StdId;
    } else {
        can_id = rx_header.ExtId;
    }

    /* DEBUG: Print CAN RX message za diagnostiko */
    if (hcan == &hcan1) {
        (void)snprintf(debug_buf, sizeof(debug_buf),
            "[CAN1 RX] ID:0x%03lX DLC:%u Data:%02X %02X %02X %02X\r\n",
            can_id, rx_header.DLC, rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
    } else {
        (void)snprintf(debug_buf, sizeof(debug_buf),
            "[CAN2 RX] ID:0x%08lX DLC:%u Data:%02X %02X\r\n",
            can_id, rx_header.DLC, rx_data[0], rx_data[1]);
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, (uint16_t)strlen(debug_buf), 10);

    /* Route message to appropriate handler */
    if (hcan == &hcan1) {
        /* CAN1 - Diagnostic interface (commands from external controller) */
        DCDC_Diag_ProcessCommand(can_id, rx_data, rx_header.DLC);
    }
    else if (hcan == &hcan2) {
        /* CAN2 - DC/DC converter feedback messages */
        DCDC_ProcessCANMessage(can_id, rx_data, rx_header.DLC);
    }
    else {
        /* MISRA C 2012 Rule 15.7: Non-empty else required */
        /* Unknown CAN interface - no action required */
        (void)0; /* Explicit NOP */
    }

    /* Process BMU protocol messages for both CAN1 and CAN2 */
    /* Pass already-read message to avoid double-read from FIFO */
    BMU_CAN_ProcessRxMessageISR(hcan, &rx_header, rx_data);
}

/* USER CODE END 1 */
