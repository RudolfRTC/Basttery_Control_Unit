#!/bin/bash
echo "=== Checking CAN interrupt handlers ==="
if grep -q "CAN1_RX0_IRQHandler" Core/Src/stm32f4xx_it.c; then
    echo "✓ CAN1_RX0_IRQHandler found"
else
    echo "✗ CAN1_RX0_IRQHandler MISSING!"
fi

if grep -q "HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn)" Core/Src/stm32f4xx_hal_msp.c; then
    echo "✓ NVIC EnableIRQ found"
else
    echo "✗ NVIC EnableIRQ MISSING!"
fi

if grep -q "HAL_CAN_RxFifo0MsgPendingCallback" Core/Src/main.c; then
    echo "✓ RX callback found"
else
    echo "✗ RX callback MISSING!"
fi

echo ""
echo "=== Checking debug output ==="
if grep -q "#if 1" Core/Src/bmu_can.c; then
    echo "✓ Debug output ENABLED"
else
    echo "⚠ Debug output DISABLED"
fi

echo ""
echo "=== File modification times ==="
stat -c "%n: %y" Core/Src/stm32f4xx_it.c
stat -c "%n: %y" Core/Src/stm32f4xx_hal_msp.c
stat -c "%n: %y" Core/Src/bmu_can.c
