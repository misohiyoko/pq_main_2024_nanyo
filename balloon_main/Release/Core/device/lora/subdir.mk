################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/device/lora/lora.c 

C_DEPS += \
./Core/device/lora/lora.d 

OBJS += \
./Core/device/lora/lora.o 


# Each subdirectory must supply rules for building sources it contributes
Core/device/lora/%.o Core/device/lora/%.su Core/device/lora/%.cyclo: ../Core/device/lora/%.c Core/device/lora/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Core/Inc -I"/Users/genta/pq_main_2024_nanyo/balloon_main/Core/support" -I"/Users/genta/pq_main_2024_nanyo/balloon_main/Core/device" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/ -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-device-2f-lora

clean-Core-2f-device-2f-lora:
	-$(RM) ./Core/device/lora/lora.cyclo ./Core/device/lora/lora.d ./Core/device/lora/lora.o ./Core/device/lora/lora.su

.PHONY: clean-Core-2f-device-2f-lora

