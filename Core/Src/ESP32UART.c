#include "ESP32UART.h"
#include <string.h>
#include <stdio.h>

/* Biến lưu trữ UART sử dụng để giao tiếp với ESP32 */
static UART_HandleTypeDef *esp_uart = NULL;

/* Hàm tính checksum: XOR các byte của mảng dữ liệu rồi lấy bù 1 */
static uint8_t ESP32UART_CalculateChecksum(uint8_t *data, uint8_t len)
{
    uint8_t checksum = 0;
    uint8_t i; // Khai báo bên ngoài vòng lặp để tương thích với một số toolchain
    for (i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }
    return (uint8_t)(~checksum);
}

/* Hàm khởi tạo giao tiếp với ESP32 */
void ESP32UART_Init(UART_HandleTypeDef *huart)
{
    esp_uart = huart;
}

/* Hàm gửi dữ liệu sang ESP32 theo giao thức đóng gói:
   | STX (1 byte) | LEN (1 byte) | PAYLOAD (LEN bytes) | CHECKSUM (1 byte) | ETX (1 byte) |
*/
void ESP32UART_SendData(const char *data)
{
    if (esp_uart == NULL)
        return;

    uint8_t payloadLen = (uint8_t)strlen(data);
    uint8_t packet[128];
    uint8_t idx = 0;

    packet[idx++] = ESP32UART_STX;
    packet[idx++] = payloadLen;

    /* Sao chép payload vào gói tin */
    memcpy(&packet[idx], data, payloadLen);
    idx += payloadLen;

    /* Tính checksum trên các byte từ LEN đến payload */
    uint8_t checksum = ESP32UART_CalculateChecksum(&packet[1], 1 + payloadLen);
    packet[idx++] = checksum;
    packet[idx++] = ESP32UART_ETX;

    HAL_UART_Transmit(esp_uart, packet, idx, HAL_MAX_DELAY);
}
