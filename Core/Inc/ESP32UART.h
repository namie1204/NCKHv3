#ifndef ESP32UART_H
#define ESP32UART_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>

/* --- Các hằng số giao thức đóng gói --- */
#define ESP32UART_STX 0x02 // Byte bắt đầu
#define ESP32UART_ETX 0x03 // Byte kết thúc

    /* --- Khai báo hàm --- */
    /**
     * @brief Khởi tạo giao tiếp với ESP32 qua UART.
     *
     * @param huart Con trỏ tới UART_HandleTypeDef của UART dùng để giao tiếp với ESP32.
     */
    void ESP32UART_Init(UART_HandleTypeDef *huart);

    /**
     * @brief Gửi dữ liệu sang ESP32 theo giao thức đóng gói:
     * | STX (1 byte) | LEN (1 byte) | PAYLOAD (LEN bytes) | CHECKSUM (1 byte) | ETX (1 byte) |
     *
     * @param data Chuỗi dữ liệu cần gửi.
     */
    void ESP32UART_SendData(const char *data);

#ifdef __cplusplus
}
#endif

#endif /* ESP32UART_H */
