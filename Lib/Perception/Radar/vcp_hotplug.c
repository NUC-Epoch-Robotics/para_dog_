#include "vcp_hotplug.h"

#include <stddef.h>

void VcpHotplug_Init(VcpHotplugState *state)
{
    if (state == NULL)
    {
        return;
    }

    state->dtr_ready = 0U;
    state->connected = 0U;
}

void VcpHotplug_SetDtrReady(VcpHotplugState *state, uint8_t dtr_ready)
{
    if (state == NULL)
    {
        return;
    }

    state->dtr_ready = (dtr_ready != 0U) ? 1U : 0U;
}

uint8_t VcpHotplug_UpdateConnected(VcpHotplugState *state,
                                   uint8_t usb_configured,
                                   uint8_t use_dtr_gating)
{
    if (state == NULL)
    {
        return 0U;
    }

    if (usb_configured == 0U)
    {
        state->connected = 0U;
        return state->connected;
    }

    if (use_dtr_gating != 0U)
    {
        state->connected = state->dtr_ready;
    }
    else
    {
        state->connected = 1U;
    }

    return state->connected;
}

uint8_t VcpHotplug_IsConnected(const VcpHotplugState *state)
{
    if (state == NULL)
    {
        return 0U;
    }

    return state->connected;
}
