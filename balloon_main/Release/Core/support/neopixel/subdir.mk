################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/support/neopixel/ARGB.c \
../Core/support/neopixel/example.c 

C_DEPS += \
./Core/support/neopixel/ARGB.d \
./Core/support/neopixel/example.d 

OBJS += \
./Core/support/neopixel/ARGB.o \
./Core/support/neopixel/example.o 


# Each subdirectory must supply rules for building sources it contributes
Core/support/neopixel/%.o Core/support/neopixel/%.su Core/support/neopixel/%.cyclo: ../Core/support/neopixel/%.c Core/support/neopixel/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Core/Inc -I"/Users/genta/pq_main_2024_nanyo/balloon_main/Core/support" -I"/Users/genta/pq_main_2024_nanyo/balloon_main/Core/device" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/ -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-support-2f-neopixel

clean-Core-2f-support-2f-neopixel:
	-$(RM) ./Core/support/neopixel/ARGB.cyclo ./Core/support/neopixel/ARGB.d ./Core/support/neopixel/ARGB.o ./Core/support/neopixel/ARGB.su ./Core/support/neopixel/example.cyclo ./Core/support/neopixel/example.d ./Core/support/neopixel/example.o ./Core/support/neopixel/example.su

.PHONY: clean-Core-2f-support-2f-neopixel
