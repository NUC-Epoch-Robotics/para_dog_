#ifndef READDATA_H
#define READDATA_H

#include <stdint.h>
#include "cmsis_os.h"

#define VCP_SIGNAL (1U<<0)
extern osThreadId vcp_read_thread_id;

void VCP_ReadTask(void);
typedef struct
{
    float xdata; // 单位: mm
    float ydata; // 单位: mm
} vcp_message_t;
void VCP_ReadTask_Init(osThreadId thread_id);
uint16_t VCP_Read(uint8_t *buf, uint16_t max_len);
uint16_t VCP_GetRxCount(void);
void VCP_FlushRx(void);
void VCP_ResetRxBuffer(void);
uint16_t VCP_WriteRxData(const uint8_t *data, uint16_t len);

#endif
