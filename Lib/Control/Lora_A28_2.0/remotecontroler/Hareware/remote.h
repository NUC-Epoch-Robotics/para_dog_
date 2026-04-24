#ifndef __REMOTE_H_
#define __REMOTE_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define JOY_ADC_NUM     4
#define JOY_DEDA_ZONE   80
#define joy_max_out     3000


typedef struct
{
    int16_t mid;
    int16_t pos_span;
    int16_t neg_span;
	
	int16_t mid_left;
    int16_t pos_span_left;
    int16_t neg_span_left;
} cal_t;

void joystick_calibrate(void);
int16_t joy_get_x(void);
int16_t joy_get_y(void);

int16_t joy_get_x_left(void);
int16_t joy_get_y_left(void);

#endif

