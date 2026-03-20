#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H

#include <stdint.h>

float vofa_GetData(uint8_t *data);
void vofa_data_write(uint8_t index, float *receiver);
#endif
