#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

#define KEY_COUNT				13

#define KEY_1					0
#define KEY_2					1
#define KEY_3					2
#define KEY_4					3
#define KEY_5					4
#define KEY_6					5
#define KEY_7					6
#define KEY_8					7
#define KEY_9					8
#define KEY_10					9
#define KEY_11					10
#define KEY_12					11
#define KEY_13					12


#define KEY_PRESSED_LEVEL       GPIO_PIN_RESET

void Keys_Init(void);

void Keys_Tick(void);

uint8_t Key_WasPressed(uint8_t key_id);

#endif

