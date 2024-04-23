//
// Created by genta on 2024/04/20.
//

#ifndef BALLOON_MAIN_TIME_TASK_H
#define BALLOON_MAIN_TIME_TASK_H
#include "time.h"

time_t unixtime_gps_s;


uint32_t last_updated_gps;
uint32_t last_updated_rtc;

int is_gps_time_valid;

void vTaskTimeCheck(void *pvParameters );

time_t getTimeSensor();

#endif //BALLOON_MAIN_TIME_TASK_H
