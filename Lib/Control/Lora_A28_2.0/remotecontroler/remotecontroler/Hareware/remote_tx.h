#ifndef __RENOTE_H__
#define __RENOTE_H__

#include "main.h"

#define mode_external_rotation     (1u << 0)
#define mode_internal_rotation     (1u << 1)
#define mode_turn_on_air_pump      (1u << 2)
#define mode_turn_off_air_pump	   (1u << 3)

int joy_send(int16_t joy_x , int16_t joy_y ,int16_t joy_z);
int mode_send(uint8_t mode_bits);


#endif
