################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc_dma.c \
../Core/Src/bmu_can.c \
../Core/Src/btt6200_4esa.c \
../Core/Src/btt6200_config.c \
../Core/Src/can_diagnostics.c \
../Core/Src/cy15b256j.c \
../Core/Src/lem_config.c \
../Core/Src/lem_hoys.c \
../Core/Src/main.c \
../Core/Src/safety.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/temp_logger.c \
../Core/Src/tmp1075.c 

OBJS += \
./Core/Src/adc_dma.o \
./Core/Src/bmu_can.o \
./Core/Src/btt6200_4esa.o \
./Core/Src/btt6200_config.o \
./Core/Src/can_diagnostics.o \
./Core/Src/cy15b256j.o \
./Core/Src/lem_config.o \
./Core/Src/lem_hoys.o \
./Core/Src/main.o \
./Core/Src/safety.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/temp_logger.o \
./Core/Src/tmp1075.o 

C_DEPS += \
./Core/Src/adc_dma.d \
./Core/Src/bmu_can.d \
./Core/Src/btt6200_4esa.d \
./Core/Src/btt6200_config.d \
./Core/Src/can_diagnostics.d \
./Core/Src/cy15b256j.d \
./Core/Src/lem_config.d \
./Core/Src/lem_hoys.d \
./Core/Src/main.d \
./Core/Src/safety.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/temp_logger.d \
./Core/Src/tmp1075.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F413xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc_dma.cyclo ./Core/Src/adc_dma.d ./Core/Src/adc_dma.o ./Core/Src/adc_dma.su ./Core/Src/bmu_can.cyclo ./Core/Src/bmu_can.d ./Core/Src/bmu_can.o ./Core/Src/bmu_can.su ./Core/Src/btt6200_4esa.cyclo ./Core/Src/btt6200_4esa.d ./Core/Src/btt6200_4esa.o ./Core/Src/btt6200_4esa.su ./Core/Src/btt6200_config.cyclo ./Core/Src/btt6200_config.d ./Core/Src/btt6200_config.o ./Core/Src/btt6200_config.su ./Core/Src/can_diagnostics.cyclo ./Core/Src/can_diagnostics.d ./Core/Src/can_diagnostics.o ./Core/Src/can_diagnostics.su ./Core/Src/cy15b256j.cyclo ./Core/Src/cy15b256j.d ./Core/Src/cy15b256j.o ./Core/Src/cy15b256j.su ./Core/Src/lem_config.cyclo ./Core/Src/lem_config.d ./Core/Src/lem_config.o ./Core/Src/lem_config.su ./Core/Src/lem_hoys.cyclo ./Core/Src/lem_hoys.d ./Core/Src/lem_hoys.o ./Core/Src/lem_hoys.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/safety.cyclo ./Core/Src/safety.d ./Core/Src/safety.o ./Core/Src/safety.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/temp_logger.cyclo ./Core/Src/temp_logger.d ./Core/Src/temp_logger.o ./Core/Src/temp_logger.su ./Core/Src/tmp1075.cyclo ./Core/Src/tmp1075.d ./Core/Src/tmp1075.o ./Core/Src/tmp1075.su

.PHONY: clean-Core-2f-Src

