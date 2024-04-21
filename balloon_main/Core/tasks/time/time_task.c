//
// Created by genta on 2024/04/20.
//


#include "time_task.h"
#include "stm32l4xx_hal_gpio.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_1)
    {
        last_updated_gps = HAL_GetTick();
        is_gps_time_valid = 1;
        unixtime_gps_s++;
    }
}

void vTaskTimeCheck(void *pvParameters ){
    if(!is_gps_time_valid || (HAL_GetTick()-last_updated_gps) > 1200){
        is_gps_time_valid = 0;
        last_updated_rtc = HAL_GetTick();
    }
}

time_t getTimeSensor(){

}