#ifndef READDATA_H
#define READDATA_H

#include <stdint.h>

void VCP_ReadTask(void);
typedef struct
{
    float xdata; // 4字节浮点数
    float ydata; // 4字节浮点数
} vcp_message_t;

uint16_t VCP_Read(uint8_t *buf, uint16_t max_len);
uint16_t VCP_GetRxCount(void);
void VCP_FlushRx(void);
void VCP_ResetRxBuffer(void);
uint16_t VCP_WriteRxData(const uint8_t *data, uint16_t len);

#endif
