#ifndef SMS_H
#define SMS_H

#include "stm32f4xx_hal.h"

void SMS_Init(UART_HandleTypeDef *huart);
uint8_t SMS_Send(char *phoneNumber, char *message);

#endif // SMS_H
