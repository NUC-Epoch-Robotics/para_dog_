#include "remote.h"
#include "adc.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

uint16_t joy_raw[JOY_ADC_NUM];

extern ADC_HandleTypeDef hadc1;

cal_t joy_x_cal = {0};
cal_t joy_y_cal = {0};

int16_t joy_x = 0;
int16_t joy_y = 0;

int16_t joy_x_left = 0;
int16_t joy_y_left = 0;

static int16_t map_axis(int16_t raw, cal_t *cal);

void joystick_calibrate(void)
{
    const uint16_t sample_cnt = 200;
    uint32_t sum_x = 0;
    uint32_t sum_y = 0;
	uint32_t sum_x_left = 0;
    uint32_t sum_y_left = 0;

    HAL_Delay(50);

    for (uint16_t i = 0; i< sample_cnt ; i++)
    {
    sum_x += joy_raw[0];
    sum_y += joy_raw[1];
	sum_x_left += joy_raw[2];
    sum_y_left += joy_raw[3];
    HAL_Delay(2);
    }
    joy_x_cal.mid = (int16_t)(sum_x / sample_cnt);
    joy_y_cal.mid = (int16_t)(sum_y / sample_cnt);
	
	joy_x_cal.mid_left = (int16_t)(sum_x_left / sample_cnt);
    joy_y_cal.mid_left = (int16_t)(sum_y_left / sample_cnt);
	
    joy_x_cal.pos_span = 2080;
    joy_x_cal.neg_span = 2080;
    joy_y_cal.pos_span = 2000;
    joy_y_cal.neg_span = 2000;
	
	joy_x_cal.pos_span_left = 2080;
    joy_x_cal.neg_span_left = 2080;
    joy_y_cal.pos_span_left = 2000;
    joy_y_cal.neg_span_left = 2000;

}
static int16_t map_axis(int16_t raw, cal_t *cal)
{
    int32_t v = (int32_t)raw - cal->mid;

    if(v > -JOY_DEDA_ZONE && v < JOY_DEDA_ZONE)return 0;

    int32_t span;
    int32_t v_abs;
    int32_t sign;

    if(v > 0)
    {
        sign = 1;
        span = (int32_t)cal->pos_span;
        v_abs = v - JOY_DEDA_ZONE;
    }
    else{
        sign = -1;
        span = (int32_t)cal->neg_span;
        v_abs = -v - JOY_DEDA_ZONE;
    }

    if(span <= JOY_DEDA_ZONE + 1)
    {
        span = JOY_DEDA_ZONE + 1;
    }

    int32_t span_eff = span - JOY_DEDA_ZONE;

    //做线性映射
    int32_t x_lin = v_abs * joy_max_out / span_eff;

    if(x_lin > joy_max_out)
    {
        x_lin = joy_max_out;
    }

    //在做二次映射
    int32_t x_curve = (x_lin * x_lin) / joy_max_out;
        if(x_curve > joy_max_out)
    {
        x_curve = joy_max_out;
    }

    int32_t val = sign * x_curve;

    if(val > joy_max_out)val = joy_max_out;
    if(val < -joy_max_out)val = -joy_max_out;

    return (int16_t)val;
}
int16_t joy_get_x(void)
{
    return map_axis((int16_t)joy_raw[0], &joy_x_cal);
}
int16_t joy_get_y(void)
{
    return map_axis((int16_t)joy_raw[1], &joy_y_cal);
}

static int16_t map_axis_left(int16_t raw, cal_t *cal)
{
    int32_t v = raw - cal->mid_left;

    if(v > -JOY_DEDA_ZONE && v < JOY_DEDA_ZONE)return 0;

    if( v > 0)
    {
        int32_t val = (v - JOY_DEDA_ZONE) * 1000 / (cal->pos_span_left - JOY_DEDA_ZONE);
        if(val > 1000) val = 1000;
        return (int16_t)val;
    }
    else {
        int32_t val = (v + JOY_DEDA_ZONE) * 1000 / (cal->neg_span_left - JOY_DEDA_ZONE);
        if(val <  -1000) val = -1000;
        return (int16_t)val;
    }
}
int16_t joy_get_x_left(void)
{
    return map_axis((int16_t)joy_raw[2], &joy_x_cal);
}
int16_t joy_get_y_left(void)
{
    return map_axis_left((int16_t)joy_raw[3], &joy_y_cal);
}


