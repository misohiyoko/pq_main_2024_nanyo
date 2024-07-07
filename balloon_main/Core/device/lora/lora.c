//
// Created by genta on 2024/05/29.
//

#include "lora.h"

#include <sys/types.h>


extern UART_HandleTypeDef huart1;
uint8_t buffer[512];
int lora_init(){
    reset_comm_power();
	HAL_GPIO_WritePin(RM92A_RESET_GPIO_Port, RM92A_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    uint8_t config[] = "\r\r1\ra32\ra32\rc0\rd65535\re0\ri0\rl1\r0\rp1\r0\rp2\r0\rt0\rxs";
    uint8_t down[1];
    for (size_t i = 0; i < sizeof(config); i++) {
     		down[0] = config[i];
     		HAL_UART_Transmit(&huart1, down, 1,10);
     		printf("%c", down[0]);
     		HAL_Delay(100);
     }
    printf("\r\n");
    HAL_Delay(11000);
	return 0;
}
int lora_send(const char *format, ...){
	va_list va;
	va_start(va, format);
	int size = sprintf(buffer,format, va);
	uint8_t down[1];
	for(int i = 0; i < size; i++)
	{
		down[0] = buffer[i];
		HAL_UART_Transmit(&huart1, down, 1,10);
		printf("%c", down[0]);
	}
	va_end(va);
	return 0;
}
