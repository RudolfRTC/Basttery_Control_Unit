/**
  ******************************************************************************
  * @file           : can_diagnostics.c
  * @brief          : CAN Bus Diagnostic Tools Implementation
  ******************************************************************************
  */

#include "can_diagnostics.h"
#include <string.h>
#include <stdio.h>

/* Private function to send UART message */
static void DIAG_Print(UART_HandleTypeDef* huart, const char* msg)
{
    HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 1000);
}

/**
  * @brief  Run complete CAN diagnostics
  */
void CAN_RunDiagnostics(CAN_HandleTypeDef* hcan, UART_HandleTypeDef* huart)
{
    char buf[150];

    DIAG_Print(huart, "\r\n");
    DIAG_Print(huart, "===============================================\r\n");
    DIAG_Print(huart, "    CAN BUS DIAGNOSTICS - FULL TEST\r\n");
    DIAG_Print(huart, "===============================================\r\n\r\n");

    /* ===== TEST 1: CAN Peripheral State ===== */
    DIAG_Print(huart, "TEST 1: CAN Peripheral State\r\n");
    DIAG_Print(huart, "---------------------------------------------\r\n");

    HAL_CAN_StateTypeDef state = HAL_CAN_GetState(hcan);
    snprintf(buf, sizeof(buf), "  CAN State: 0x%02X ", state);
    DIAG_Print(huart, buf);

    if (state == HAL_CAN_STATE_READY) {
        DIAG_Print(huart, "(READY - OK)\r\n");
    } else if (state == HAL_CAN_STATE_LISTENING) {
        DIAG_Print(huart, "(LISTENING - OK)\r\n");
    } else {
        DIAG_Print(huart, "(NOT READY - ERROR!)\r\n");
    }

    uint32_t error = HAL_CAN_GetError(hcan);
    snprintf(buf, sizeof(buf), "  CAN Error Code: 0x%08lX\r\n", error);
    DIAG_Print(huart, buf);

    if (error == 0) {
        DIAG_Print(huart, "  ✓ No errors\r\n");
    } else {
        DIAG_Print(huart, "  ✗ ERROR DETECTED!\r\n");
        if (error & HAL_CAN_ERROR_EWG) DIAG_Print(huart, "    - Error Warning\r\n");
        if (error & HAL_CAN_ERROR_EPV) DIAG_Print(huart, "    - Error Passive\r\n");
        if (error & HAL_CAN_ERROR_BOF) DIAG_Print(huart, "    - Bus-Off\r\n");
        if (error & HAL_CAN_ERROR_STF) DIAG_Print(huart, "    - Stuff Error\r\n");
        if (error & HAL_CAN_ERROR_FOR) DIAG_Print(huart, "    - Form Error\r\n");
        if (error & HAL_CAN_ERROR_ACK) DIAG_Print(huart, "    - Ack Error (no other node!)\r\n");
        if (error & HAL_CAN_ERROR_BR)  DIAG_Print(huart, "    - Bit Recessive Error\r\n");
        if (error & HAL_CAN_ERROR_BD)  DIAG_Print(huart, "    - Bit Dominant Error\r\n");
        if (error & HAL_CAN_ERROR_CRC) DIAG_Print(huart, "    - CRC Error\r\n");
    }

    DIAG_Print(huart, "\r\n");

    /* ===== TEST 2: NVIC Interrupt Status ===== */
    DIAG_Print(huart, "TEST 2: NVIC Interrupt Configuration\r\n");
    DIAG_Print(huart, "---------------------------------------------\r\n");

    CAN_CheckNVIC(huart);

    DIAG_Print(huart, "\r\n");

    /* ===== TEST 3: CAN Registers ===== */
    DIAG_Print(huart, "TEST 3: CAN Hardware Registers\r\n");
    DIAG_Print(huart, "---------------------------------------------\r\n");

    if (hcan->Instance == CAN1) {
        snprintf(buf, sizeof(buf), "  CAN1->MCR:  0x%08lX\r\n", CAN1->MCR);
        DIAG_Print(huart, buf);
        snprintf(buf, sizeof(buf), "  CAN1->MSR:  0x%08lX\r\n", CAN1->MSR);
        DIAG_Print(huart, buf);
        snprintf(buf, sizeof(buf), "  CAN1->TSR:  0x%08lX\r\n", CAN1->TSR);
        DIAG_Print(huart, buf);
        snprintf(buf, sizeof(buf), "  CAN1->RF0R: 0x%08lX ", CAN1->RF0R);
        DIAG_Print(huart, buf);

        uint8_t fmp = CAN1->RF0R & 0x03;
        snprintf(buf, sizeof(buf), "(FMP=%d messages)\r\n", fmp);
        DIAG_Print(huart, buf);

        snprintf(buf, sizeof(buf), "  CAN1->IER:  0x%08lX\r\n", CAN1->IER);
        DIAG_Print(huart, buf);

        if (CAN1->IER & CAN_IER_FMPIE0) {
            DIAG_Print(huart, "  ✓ FIFO0 interrupt ENABLED in CAN\r\n");
        } else {
            DIAG_Print(huart, "  ✗ FIFO0 interrupt DISABLED in CAN!\r\n");
        }
    }

    DIAG_Print(huart, "\r\n");

    /* ===== TEST 4: Loopback Test ===== */
    DIAG_Print(huart, "TEST 4: CAN Loopback Test\r\n");
    DIAG_Print(huart, "---------------------------------------------\r\n");
    DIAG_Print(huart, "  Testing internal loopback mode...\r\n");

    HAL_StatusTypeDef result = CAN_TestLoopback(hcan, huart);

    if (result == HAL_OK) {
        DIAG_Print(huart, "  ✓ Loopback test PASSED\r\n");
        DIAG_Print(huart, "  → CAN peripheral working correctly!\r\n");
    } else {
        DIAG_Print(huart, "  ✗ Loopback test FAILED\r\n");
        DIAG_Print(huart, "  → CAN peripheral hardware problem!\r\n");
    }

    DIAG_Print(huart, "\r\n");
    DIAG_Print(huart, "===============================================\r\n");
    DIAG_Print(huart, "  DIAGNOSTICS COMPLETE\r\n");
    DIAG_Print(huart, "===============================================\r\n\r\n");
}

/**
  * @brief  Check NVIC interrupt status
  */
void CAN_CheckNVIC(UART_HandleTypeDef* huart)
{
    char buf[100];

    // Check CAN1 RX0 interrupt
    if (NVIC_GetEnableIRQ(CAN1_RX0_IRQn)) {
        DIAG_Print(huart, "  ✓ CAN1_RX0 interrupt ENABLED in NVIC\r\n");
    } else {
        DIAG_Print(huart, "  ✗ CAN1_RX0 interrupt DISABLED in NVIC!\r\n");
    }

    uint32_t priority = NVIC_GetPriority(CAN1_RX0_IRQn);
    snprintf(buf, sizeof(buf), "    Priority: %lu\r\n", priority);
    DIAG_Print(huart, buf);

    // Check CAN1 TX interrupt
    if (NVIC_GetEnableIRQ(CAN1_TX_IRQn)) {
        DIAG_Print(huart, "  ✓ CAN1_TX interrupt ENABLED in NVIC\r\n");
    } else {
        DIAG_Print(huart, "  ✗ CAN1_TX interrupt DISABLED in NVIC!\r\n");
    }

    // Check pending interrupts
    if (NVIC_GetPendingIRQ(CAN1_RX0_IRQn)) {
        DIAG_Print(huart, "  ! CAN1_RX0 interrupt PENDING (waiting)\r\n");
    }

    if (NVIC_GetPendingIRQ(CAN1_TX_IRQn)) {
        DIAG_Print(huart, "  ! CAN1_TX interrupt PENDING (waiting)\r\n");
    }
}

/**
  * @brief  Test CAN loopback mode
  */
HAL_StatusTypeDef CAN_TestLoopback(CAN_HandleTypeDef* hcan, UART_HandleTypeDef* huart)
{
    char buf[100];
    HAL_StatusTypeDef status;

    // Save original mode
    uint32_t original_mode = hcan->Init.Mode;

    // Stop CAN
    HAL_CAN_Stop(hcan);

    // Switch to loopback mode
    hcan->Init.Mode = CAN_MODE_LOOPBACK;
    if (HAL_CAN_Init(hcan) != HAL_OK) {
        DIAG_Print(huart, "  ✗ Failed to init loopback mode\r\n");
        return HAL_ERROR;
    }

    // Start CAN
    if (HAL_CAN_Start(hcan) != HAL_OK) {
        DIAG_Print(huart, "  ✗ Failed to start CAN in loopback\r\n");
        return HAL_ERROR;
    }

    // Prepare TX message
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};
    uint32_t tx_mailbox;

    tx_header.StdId = 0x123;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    // Send message
    status = HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
    if (status != HAL_OK) {
        DIAG_Print(huart, "  ✗ Failed to send test message\r\n");
        goto cleanup;
    }

    snprintf(buf, sizeof(buf), "  → Sent test message ID:0x123 in loopback\r\n");
    DIAG_Print(huart, buf);

    // Wait for message to arrive in RX FIFO
    HAL_Delay(10);

    // Check if message received
    if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            DIAG_Print(huart, "  → Received message in loopback:\r\n");
            snprintf(buf, sizeof(buf), "    ID: 0x%03lX, DLC: %d\r\n",
                    rx_header.StdId, rx_header.DLC);
            DIAG_Print(huart, buf);

            snprintf(buf, sizeof(buf), "    Data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                    rx_data[0], rx_data[1], rx_data[2], rx_data[3],
                    rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
            DIAG_Print(huart, buf);

            // Verify data
            if (memcmp(tx_data, rx_data, 8) == 0) {
                status = HAL_OK;
            } else {
                DIAG_Print(huart, "  ✗ Data mismatch!\r\n");
                status = HAL_ERROR;
            }
        } else {
            DIAG_Print(huart, "  ✗ Failed to read RX message\r\n");
            status = HAL_ERROR;
        }
    } else {
        DIAG_Print(huart, "  ✗ No message received in loopback\r\n");
        status = HAL_ERROR;
    }

cleanup:
    // Restore original mode
    HAL_CAN_Stop(hcan);
    hcan->Init.Mode = original_mode;

    if (HAL_CAN_Init(hcan) != HAL_OK) {
        DIAG_Print(huart, "  ✗ Failed to re-init CAN!\r\n");
        return HAL_ERROR;
    }

    // CRITICAL: Re-configure filter (HAL_CAN_Init may have reset it!)
    // Forward declare the filter config function
    extern HAL_StatusTypeDef BMU_CAN_ConfigureFilter(CAN_HandleTypeDef* hcan);
    if (BMU_CAN_ConfigureFilter(hcan) != HAL_OK) {
        DIAG_Print(huart, "  ✗ Failed to re-configure filter!\r\n");
        return HAL_ERROR;
    }

    // CRITICAL: Restart CAN after test!
    if (HAL_CAN_Start(hcan) != HAL_OK) {
        DIAG_Print(huart, "  ✗ Failed to restart CAN after loopback!\r\n");
        return HAL_ERROR;
    }

    // CRITICAL: Re-enable RX notifications (HAL_CAN_Stop disabled them!)
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        DIAG_Print(huart, "  ✗ Failed to re-activate RX notifications!\r\n");
        return HAL_ERROR;
    }

    DIAG_Print(huart, "  → CAN restored to normal mode and restarted\r\n");

    return status;
}
