#ifndef HLK_LD6002_H
#define HLK_LD6002_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>

// Khai báo các hằng số giao tiếp
#define SOF 0x01

// Loại frame
#define ALL_DATA 0x0A13        // Composite frame: includes 3 float (total phase, breath phase, heart phase)
#define BREATH_RATE 0x0A14     // Frame reports breathing rate results (1 float)
#define HEART_RATE 0x0A15      // Frame reports heart rate results (1 float)
#define DETECT_DISTANCE 0x0A16 // Frame indicates detection distance: 4 byte flag + 1 float

// Position of bytes in header (8 bytes)
#define SOF_BYTE 0
#define ID_BYTE1 1
#define ID_BYTE2 2
#define LEN_BYTE_H 3
#define LEN_BYTE_L 4
#define TYPE_BYTE_H 5
#define TYPE_BYTE_L 6
#define HEAD_CKS_BYTE 7

// Data length of each frame
#define ALL_DATA_LEN 12     // 3 x 4 byte float
#define ONE_DATA_LEN 4      // 1 x 4 byte float
#define DISTANCE_DATA_LEN 8 // 4 byte flag + 4 byte float

    // Structure to store measured data from radar
    typedef struct
    {
        // Frame ALL_DATA (0x0A13)
        float total_phase;
        float breath_phase;
        float heart_phase;

        // Frame
        float breath_rate; // (0x0A14)
        float heart_rate;  // (0x0A15)

        // (0x0A16)
        uint32_t distance_flag; // Flag: if = 1 then there is result
        float distance;         // Distance (cm)
    } RadarData;

    void HLK_LD6002_Init(UART_HandleTypeDef *huart);

    void HLK_LD6002_ProcessData(void);

    void HLK_LD6002_ParseFrame(uint8_t *buffer, uint16_t length);

    // Function to get measured data
    RadarData HLK_LD6002_GetRadarData(void);

    void HLK_LD6002_OutputResult(const char *msg);
    void HLK_LD6002_LCD_Print(const char *msg);

#ifdef __cplusplus
}
		
#endif

#endif /* HLK_LD6002_H */
