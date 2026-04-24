#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H

#include <stdint.h>
#include "dog.h"
extern uint8_t vofa_rx_frame[6];
float vofa_GetData(uint8_t *data);
void vofa_data_write(uint8_t index, float *receiver);
void vofa_rc(Dog *dog);
void vofa_init(UART_HandleTypeDef *huart);
#endif
