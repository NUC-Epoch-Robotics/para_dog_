#ifndef VCP_HOTPLUG_H
#define VCP_HOTPLUG_H

#include <stdint.h>

typedef struct
{
    uint8_t dtr_ready;
    uint8_t connected;
} VcpHotplugState;

void VcpHotplug_Init(VcpHotplugState *state);

void VcpHotplug_SetDtrReady(VcpHotplugState *state, uint8_t dtr_ready);

uint8_t VcpHotplug_UpdateConnected(VcpHotplugState *state,
                                   uint8_t usb_configured,
                                   uint8_t use_dtr_gating);

uint8_t VcpHotplug_IsConnected(const VcpHotplugState *state);

#endif
