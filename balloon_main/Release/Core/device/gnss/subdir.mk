################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/device/gnss/gnss.c \
../Core/device/gnss/minmea.c 

C_DEPS += \
./Core/device/gnss/gnss.d \
./Core/device/gnss/minmea.d 

OBJS += \
./Core/device/gnss/gnss.o \
./Core/device/gnss/minmea.o 


# Each subdirectory must supply rules for building sources it contributes
Core/device/gnss/%.o Core/device/gnss/%.su Core/device/gnss/%.cyclo: ../Core/device/gnss/%.c Core/device/gnss/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Core/Inc -I"/Users/genta/pq_main_2024_nanyo/balloon_main/Core/support" -I"/Users/genta/pq_main_2024_nanyo/balloon_main/Core/device" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/ -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-device-2f-gnss

clean-Core-2f-device-2f-gnss:
	-$(RM) ./Core/device/gnss/gnss.cyclo ./Core/device/gnss/gnss.d ./Core/device/gnss/gnss.o ./Core/device/gnss/gnss.su ./Core/device/gnss/minmea.cyclo ./Core/device/gnss/minmea.d ./Core/device/gnss/minmea.o ./Core/device/gnss/minmea.su

.PHONY: clean-Core-2f-device-2f-gnss

