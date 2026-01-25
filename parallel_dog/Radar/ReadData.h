#ifndef READDATA_H
#define READDATA_H
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"   
#include "FreeRTOS.h"
#include "task.h"
#include "usb_device.h"    
#include "usbd_cdc_if.h" 
void VCP_ReadTask(void);
typedef struct {
    float xdata;    // 4字节浮点数
    float ydata;    // 4字节浮点数
} vcp_message_t;
#endif
