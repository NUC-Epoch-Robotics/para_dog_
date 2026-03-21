#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usb_device.h"
#include "ReadData.h"
#define FRAME_HEADER_1 0xAA
#define FRAME_HEADER_2 0x55
#define FRAME_TOTAL_LENGTH 10 // AA 55 + 4字节x + 4字节y
#define TMP_BUF_SIZE 64
#define LINE_BUF_SIZE 128
#define RING_BUFFER_SIZE 512

static uint8_t RingBuffer[RING_BUFFER_SIZE] = {0};
static volatile uint16_t WriteIndex = 0;
static volatile uint16_t ReadIndex = 0;

typedef enum
{
    FRAME_STATE_SEARCHING,
    FRAME_STATE_RECEIVING
} frame_state_t;
vcp_message_t msg;

void VCP_ResetRxBuffer(void)
{
    WriteIndex = 0;
    ReadIndex = 0;
}

uint16_t VCP_GetRxCount(void)
{
    uint16_t write = WriteIndex;
    uint16_t read = ReadIndex;

    if (write >= read)
    {
        return write - read;
    }
    return (uint16_t)(RING_BUFFER_SIZE - read + write);
}

uint16_t VCP_WriteRxData(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
    {
        return 0;
    }

    uint16_t write = WriteIndex;
    uint16_t read = ReadIndex;
    uint16_t free_space;

    if (write >= read)
    {
        free_space = (uint16_t)(RING_BUFFER_SIZE - (write - read) - 1);
    }
    else
    {
        free_space = (uint16_t)(read - write - 1);
    }

    if (len > free_space)
    {
        len = free_space;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        RingBuffer[WriteIndex] = data[i];
        WriteIndex = (WriteIndex + 1U) & (RING_BUFFER_SIZE - 1U);
    }

    return len;
}

uint16_t VCP_Read(uint8_t *buf, uint16_t max_len)
{
    uint16_t available = VCP_GetRxCount();

    if (available == 0 || buf == NULL || max_len == 0)
    {
        return 0;
    }

    uint16_t to_read = (max_len < available) ? max_len : available;
    uint16_t read = ReadIndex;

    if ((read + to_read) <= RING_BUFFER_SIZE)
    {
        memcpy(buf, &RingBuffer[read], to_read);
    }
    else
    {
        uint16_t first_part = (uint16_t)(RING_BUFFER_SIZE - read);
        memcpy(buf, &RingBuffer[read], first_part);
        memcpy(&buf[first_part], RingBuffer, to_read - first_part);
    }

    ReadIndex = (ReadIndex + to_read) & (RING_BUFFER_SIZE - 1U);
    return to_read;
}

void VCP_FlushRx(void)
{
    ReadIndex = WriteIndex;
}

void VCP_ReadTask(void)
{

    uint8_t tmp[TMP_BUF_SIZE];
    uint8_t linebuf[LINE_BUF_SIZE];
    uint16_t line_len = 0;
    uint8_t frame_buf[FRAME_TOTAL_LENGTH];
    uint8_t frame_len = 0;
    frame_state_t frame_state = FRAME_STATE_SEARCHING;

    for (;;)
    {
        uint16_t avail = VCP_GetRxCount();
        if (avail == 0)
        {
            vTaskDelay(5);
            continue;
        }

        uint16_t to_read = (avail > TMP_BUF_SIZE) ? TMP_BUF_SIZE : avail;
        uint16_t n = VCP_Read(tmp, to_read);

        if (n == 0)
        {
            continue;
        }

        for (uint16_t i = 0; i < n; ++i)
        {
            uint8_t b = tmp[i];
            if (line_len < (LINE_BUF_SIZE - 1))
            {
                linebuf[line_len++] = b;
            }
            else
            {
                line_len = 0;
                linebuf[line_len++] = b;
            }
            switch (frame_state)
            {
            case FRAME_STATE_SEARCHING:
                if (b == FRAME_HEADER_2)
                {
                    if (frame_len == 1 && frame_buf[0] == FRAME_HEADER_1)
                    {
                        frame_buf[1] = b;
                        frame_len = 2;
                        frame_state = FRAME_STATE_RECEIVING;
                    }
                    else
                    {
                        frame_len = 0;
                        if (b == FRAME_HEADER_1)
                        {
                            frame_buf[frame_len++] = b;
                        }
                    }
                }
                else
                {

                    frame_len = 0;
                    if (b == FRAME_HEADER_1)
                    {
                        frame_buf[frame_len++] = b;
                    }
                }
                break;

            case FRAME_STATE_RECEIVING:
                if (frame_len < FRAME_TOTAL_LENGTH)
                {
                    frame_buf[frame_len++] = b;
                }
                if (frame_len >= FRAME_TOTAL_LENGTH)
                {
                    if (frame_buf[0] == FRAME_HEADER_1 && frame_buf[1] == FRAME_HEADER_2)
                    {
                        uint32_t x_uint = ((uint32_t)frame_buf[5] << 24) | ((uint32_t)frame_buf[4] << 16) | ((uint32_t)frame_buf[3] << 8) | (uint32_t)frame_buf[2];
                        uint32_t y_uint = ((uint32_t)frame_buf[9] << 24) | ((uint32_t)frame_buf[8] << 16) | ((uint32_t)frame_buf[7] << 8) | (uint32_t)frame_buf[6];
                        memcpy(&msg.xdata, &x_uint, 4);
                        memcpy(&msg.ydata, &y_uint, 4);
                    }
                    else
                    {
                    }
                    frame_len = 0;
                    frame_state = FRAME_STATE_SEARCHING;
                }
                break;

            default:
                frame_state = FRAME_STATE_SEARCHING;
                break;
            }
        }
    }
}
