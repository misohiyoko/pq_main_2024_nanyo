//
// Created by genta on 2024/05/19.
//

#ifndef BALLOON_MAIN_POWER_H
#define BALLOON_MAIN_POWER_H

#include <stdint-gcc.h>

typedef enum {
   ON = 1,
   OFF = 0
}PowerState;

typedef struct {
    PowerState sensorPower;
    PowerState communicationPower;
}PowerSupplyState;

void ComSetSensorPower(PowerState state);
void ComSetCommunicationPower(PowerState state);


#endif //BALLOON_MAIN_POWER_H
