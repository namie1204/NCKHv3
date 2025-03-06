/*******************************************************************************************************************************
using in RADAR HLKLD6002 comunication with in STM32
EX:
#include "HLK_LD6002.h"
// Suppose you have configured UART1 for the radar
extern UART_HandleTypeDef huart1;

int main(void)
{
    // config uart

    HLK_LD6002_Init(&huart1);

    while (1)
    {
        HLK_LD6002_ProcessData();

        // RadarData data = HLK_LD6002_GetRadarData();
    }
}

 *******************************************************************************************************************************/
#include "HLK_LD6002.h"
#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
int flag;
static UART_HandleTypeDef *radar_huart;
static RadarData radarData;

void HLK_LD6002_Init(UART_HandleTypeDef *huart)
{
    radar_huart = huart;
}

static uint8_t HLK_LD6002_CalculateChecksum(uint8_t *data, uint8_t len)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }
    return ~checksum;
}

static float HLK_LD6002_BytesToFloat(uint8_t *data)
{
    float value;
    memcpy(&value, data, sizeof(float));
    return value;
}

void HLK_LD6002_OutputResult(const char *msg)
{
    HAL_UART_Transmit(radar_huart, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void HLK_LD6002_ParseFrame(uint8_t *buffer, uint16_t length)
{
    if (length < 8)
        return;

    if (buffer[SOF_BYTE] != SOF)
        return;

    uint16_t type = (buffer[TYPE_BYTE_H] << 8) | buffer[TYPE_BYTE_L];
    uint8_t headCks = buffer[HEAD_CKS_BYTE];

    if (HLK_LD6002_CalculateChecksum(buffer, 7) != headCks)
    {
        HLK_LD6002_OutputResult("Header checksum error\r\n");
        return;
    }

    char msg[64];

    switch (type)
    {
    case ALL_DATA: // Frame 0x0A13: Repor the phase test result
                   /*
                    | form       | Byte number | fundamental type | frame structure     | Example frames | The frame meaning                                 |
                    |------------|-------------|------------------|---------------------|----------------|---------------------------------------------------|
                    | SOF        | 1 byte      | uint8            | Start frame         | 01             |                                                   |
                    | ID         | 2 byte      | uint16           | frame ID            | 00 00          |                                                   |
                    | LEN        | 2 byte      | uint16           | Data frame length   | 00 04          |                                                   |
                    | TYPE       | 2 byte      | uint16           | frame type          | 0A 13          | To report the total phase,                        |
                    | HEAD_CKSUM | 1 byte      | uint8            | Head checksum       | \\             | heartbeat phase,    and respiratory phase results.|
                    | DATA       | 4 byte      | float            | [total phase]       | \\             |                                                   |
                    | DATA       | 4 byte      | float            | [breath phase]      | \\             |                                                   |
                    | DATA       | 4 byte      | float            | [heart phase]       | \\             |                                                   |
                    | DATA_CKSUM | 1 byte      | uint8            | Data checksum       | \\             |                                                   |
                    */

        /*
         Message Type: Report the phase test result of 0x0A13
         Message type is 0x0A13, supporting only one-way data transfer mode.
        */

        if (length < 8 + ALL_DATA_LEN + 1)
            return;
        if (HLK_LD6002_CalculateChecksum(&buffer[8], ALL_DATA_LEN) != buffer[8 + ALL_DATA_LEN])
        {
            HLK_LD6002_OutputResult("Data checksum error (All Data)\r\n");
            return;
        }
        radarData.breath_rate = HLK_LD6002_BytesToFloat(&buffer[8]);
        radarData.heart_rate = HLK_LD6002_BytesToFloat(&buffer[12]);
        snprintf(msg, sizeof(msg), "Breath: %.2f bpm, Heart: %.2f bpm\r\n", radarData.breath_rate, radarData.heart_rate);
        HLK_LD6002_OutputResult(msg);
        break;

    case BREATH_RATE: // Frame 0x0A14: respiratory rate
                      /*
                       Message Type: Report the breath rate test result of 0x0A14
                       Message type is 0x0A14, supporting only one-way data transfer mode.
              
                       | form       | Byte number | fundamental type | frame structure  | Example frames | The frame meaning                           |
                       | SOF        | 1 byte      | uint8            | Start frame      | 01             |                                             |
                       | ID         | 2 byte      | uint16           | frame ID         | 00 00          |                                             |
                       |------------|-------------|------------------|------------------|----------------|------------------------------------------   |
                       | LEN        | 2 byte      | uint16           | Data frame length| 00 04          |                                             |
                       | TYPE       | 2 byte      | uint16           | frame type       | 0A 14          | To report the respiratory rate test results.|
                       | HEAD_CKSUM | 1 byte      | uint8            | Head checksum    | \\             |                                             |
                       | DATA       | 4 byte      | float            | [rate]           | \\             |                                             |
                       | DATA_CKSUM | 1 byte      | uint8            | Data verification| \\             |                                             |
                       */

        if (length < 8 + ONE_DATA_LEN + 1)
            return;
        if (HLK_LD6002_CalculateChecksum(&buffer[8], ONE_DATA_LEN) != buffer[8 + ONE_DATA_LEN])
        {
            HLK_LD6002_OutputResult("Data checksum error (Breath Rate)\r\n");
            return;
        }
        radarData.breath_rate = HLK_LD6002_BytesToFloat(&buffer[8]);
        snprintf(msg, sizeof(msg), "Breath Rate: %.2f bpm\r\n", radarData.breath_rate);
        HLK_LD6002_OutputResult(msg);
        break;

    case HEART_RATE: // Frame 0x0A15: Heartbeat
                     /*
                      | form         | Byte number | fundamental type | frame structure        | Example frames | The frame meaning                                   |
                      |--------------|-------------|------------------|------------------------|----------------|-----------------------------------------------------|
                      | SOF          | 1 byte      | uint8            | Start frame            | 01             |                                                     |
                      | ID           | 2 byte      | uint16           | frame ID               | 00 00          |                                                     |
                      | LEN          | 2 byte      | uint16           | Data frame length      | 00 04          |                                                     |
                      | TYPE         | 2 byte      | uint16           | frame type             | 0A 15          | Used to report the heartbeat phase test results.    |
                      | HEAD_CKSUM   | 1 byte      | uint8            | Head checksum          | \\             |                                                     |
                      | DATA         | 4 byte      | float            | [rate]                 | \\             |                                                     |
                      | DATA_CKSUM   | 1 byte      | uint8            | Data checksum          | \\             |                                                     |
                      */

        /*
         Message Type: Report the heartbeat phase test result of 0x0A15
         Message type is 0x0A15, supporting only one-way data transfer mode.
        */

        if (length < 8 + ONE_DATA_LEN + 1)
            return;
        if (HLK_LD6002_CalculateChecksum(&buffer[8], ONE_DATA_LEN) != buffer[8 + ONE_DATA_LEN])
        {
            HLK_LD6002_OutputResult("Data checksum error (Heart Rate)\r\n");
            return;
        }
        radarData.heart_rate = HLK_LD6002_BytesToFloat(&buffer[8]);
        snprintf(msg, sizeof(msg), "Heart Rate: %.2f bpm\r\n", radarData.heart_rate);
        HLK_LD6002_OutputResult(msg);
        break;

    case DETECT_DISTANCE: // Frame 0x0A16: Detection
                          /*
                           | form       | Byte number | fundamental type | frame structure   | Example frames | The frame meaning                    |
                           |------------|-------------|------------------|-----------------  |----------------|--------------------------------------|
                           | SOF        | 1 byte      | uint8            | Start frame       | 01             |                                      |
                           | ID         | 2 byte      | uint16           | frame ID          | 00 00          |                                      |
                           | LEN        | 2 byte      | uint16           | Data frame length | 00 04          |                                      |
                           | TYPE       | 2 byte      | uint16           | frame type        | 0A 16          | To report the detection distances.   |
                           | HEAD_CKSUM | 1 byte      | uint8            | Head checksum     | \\             |                                      |
                           | DATA       | 4 byte      | Uint32           | [flag ]           | \\             |                                      |
                           | DATA       | 4 byte      | float            | [range ]          | \\             |                                      |
                           | DATA_CKSUM | 1 byte      | uint8            | Data checksum     | \\             |                                      |
                           */

        /*
         Message Type: Report the detection target distance of 0x0A16
         Message type is 0x0A16, supporting only one-way data transfer mode.

         Note: When flag is 1, the output distance (unit: cm)
               If the flag is 0, no distance is output
        */

        if (length < 8 + DISTANCE_DATA_LEN + 1)
            return;
        if (HLK_LD6002_CalculateChecksum(&buffer[8], DISTANCE_DATA_LEN) != buffer[8 + DISTANCE_DATA_LEN])
        {
            HLK_LD6002_OutputResult("Data checksum error (Distance)\r\n");
            return;
        }

        radarData.distance = HLK_LD6002_BytesToFloat(&buffer[12]);

        if (flag == 1)
        {
            snprintf(msg, sizeof(msg), "Target Distance: %.2f cm\r\n", radarData.distance);
        }
        else
        {
            snprintf(msg, sizeof(msg), "No target detected.\r\n");
        }
        HLK_LD6002_OutputResult(msg);
        break;

    default:
        snprintf(msg, sizeof(msg), "Unknown frame type: 0x%04X\r\n", type);
        HLK_LD6002_OutputResult(msg);
        break;
    }
}
RadarData HLK_LD6002_GetRadarData(void)
{
    return radarData;
}

void HLK_LD6002_ProcessData(void)
{
    uint8_t frameBuffer[64] = {0};
    if (HAL_UART_Receive(radar_huart, frameBuffer, 8, 100) == HAL_OK)
    {
        uint16_t type = (frameBuffer[TYPE_BYTE_H] << 8) | frameBuffer[TYPE_BYTE_L];
        uint16_t dataLen = (type == ALL_DATA) ? ALL_DATA_LEN : (type == DETECT_DISTANCE) ? DISTANCE_DATA_LEN
                                                                                         : ONE_DATA_LEN;
        uint16_t totalFrameSize = 8 + dataLen + 1;

        if (HAL_UART_Receive(radar_huart, &frameBuffer[8], dataLen + 1, 100) == HAL_OK)
        {
            HLK_LD6002_ParseFrame(frameBuffer, totalFrameSize);
        }
    }
}
