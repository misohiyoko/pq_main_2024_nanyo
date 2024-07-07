################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/device/env/bme280.c \
../Core/device/env/bme280_support.c \
../Core/device/env/env.c 

C_DEPS += \
./Core/device/env/bme280.d \
./Core/device/env/bme280_support.d \
./Core/device/env/env.d 

OBJS += \
./Core/device/env/bme280.o \
./Core/device/env/bme280_support.o \
./Core/device/env/env.o 


# Each subdirectory must supply rules for building sources it contributes
Core/device/env/%.o Core/device/env/%.su Core/device/env/%.cyclo: ../Core/device/env/%.c Core/device/env/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Core/Inc -I"/Users/genta/pq_main_2024_nanyo/balloon_main/Core/support" -I"/Users/genta/pq_main_2024_nanyo/balloon_main/Core/device" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/ -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-device-2f-env

clean-Core-2f-device-2f-env:
	-$(RM) ./Core/device/env/bme280.cyclo ./Core/device/env/bme280.d ./Core/device/env/bme280.o ./Core/device/env/bme280.su ./Core/device/env/bme280_support.cyclo ./Core/device/env/bme280_support.d ./Core/device/env/bme280_support.o ./Core/device/env/bme280_support.su ./Core/device/env/env.cyclo ./Core/device/env/env.d ./Core/device/env/env.o ./Core/device/env/env.su

.PHONY: clean-Core-2f-device-2f-env

