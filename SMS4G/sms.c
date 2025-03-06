#include "sms.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *smsUart;

void SMS_Init(UART_HandleTypeDef *huart)
{
    smsUart = huart;
}

uint8_t SMS_Send(char *phoneNumber, char *message)
{
    char ATcommand[100];
    uint8_t buffer[50] = {0};
    uint8_t ATisOK = 0;

    while (!ATisOK)
    {
        sprintf(ATcommand, "AT\r\n");
        HAL_UART_Transmit(smsUart, (uint8_t *)ATcommand, strlen(ATcommand), 1000);
        HAL_UART_Receive(smsUart, buffer, sizeof(buffer), 100);
        HAL_Delay(1000);
        if (strstr((char *)buffer, "OK"))
        {
            ATisOK = 1;
        }
        HAL_Delay(1000);
        memset(buffer, 0, sizeof(buffer));
    }

    sprintf(ATcommand, "AT+CMGF=1\r\n");
    HAL_UART_Transmit(smsUart, (uint8_t *)ATcommand, strlen(ATcommand), 1000);
    HAL_UART_Receive(smsUart, buffer, sizeof(buffer), 100);
    HAL_Delay(1000);
    memset(buffer, 0, sizeof(buffer));

    sprintf(ATcommand, "AT+CMGS=\"%s\"\r\n", phoneNumber);
    HAL_UART_Transmit(smsUart, (uint8_t *)ATcommand, strlen(ATcommand), 1000);
    HAL_Delay(100);

    sprintf(ATcommand, "%s%c", message, 0x1A);
    HAL_UART_Transmit(smsUart, (uint8_t *)ATcommand, strlen(ATcommand), 1000);
    HAL_UART_Receive(smsUart, buffer, sizeof(buffer), 100);
    memset(buffer, 0, sizeof(buffer));
    HAL_Delay(4000);

    return 1;
}
