################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/device/power/power.c 

OBJS += \
./Core/device/power/power.o 

C_DEPS += \
./Core/device/power/power.d 


# Each subdirectory must supply rules for building sources it contributes
Core/device/power/%.o Core/device/power/%.su Core/device/power/%.cyclo: ../Core/device/power/%.c Core/device/power/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L431xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I"C:/Users/genta/Desktop/ws2/balloon_main/Core/support" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-device-2f-power

clean-Core-2f-device-2f-power:
	-$(RM) ./Core/device/power/power.cyclo ./Core/device/power/power.d ./Core/device/power/power.o ./Core/device/power/power.su

.PHONY: clean-Core-2f-device-2f-power

