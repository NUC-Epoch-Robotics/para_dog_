#ifndef READDATA_H
#define READDATA_H


void VCP_ReadTask(void);
typedef struct {
    float xdata;    // 4字节浮点数
    float ydata;    // 4字节浮点数
} vcp_message_t;
#endif
