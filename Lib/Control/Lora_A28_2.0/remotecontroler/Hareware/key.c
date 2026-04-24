#include "Key.h"
#include "main.h"
#include "tim.h"
#include "gpio.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} KeyPin;

static const KeyPin key_pins[KEY_COUNT] = {
    [KEY_1] = { GPIOD, GPIO_PIN_15 },
    [KEY_2] = { GPIOA, GPIO_PIN_12 },
    [KEY_3] = { GPIOA, GPIO_PIN_15 },
    [KEY_4] = { GPIOA, GPIO_PIN_11 },
	[KEY_5] = { GPIOC, GPIO_PIN_3  },
    [KEY_6] = { GPIOE, GPIO_PIN_2  },
    [KEY_7] = { GPIOE, GPIO_PIN_1  },
    [KEY_8] = { GPIOC, GPIO_PIN_2  },
	[KEY_9] = { GPIOD, GPIO_PIN_9  },
	[KEY_10] = { GPIOD , GPIO_PIN_10},
	[KEY_11] = { GPIOD , GPIO_PIN_11},
	[KEY_12] = { GPIOD , GPIO_PIN_12},
	[KEY_13] = { GPIOD , GPIO_PIN_13},
};

static uint8_t  stable_state[KEY_COUNT];
static uint8_t  counter[KEY_COUNT];
static uint8_t  click_flag[KEY_COUNT];

#define DEBOUNCE_TICKS   3

void Keys_Init(void)
{
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        stable_state[i] = 0;
        counter[i]      = 0;
        click_flag[i]   = 0;
    }
}


void Keys_Tick(void)
{
    for (uint8_t i = 0; i < KEY_COUNT; i++) {

        GPIO_PinState pin = HAL_GPIO_ReadPin(key_pins[i].port, key_pins[i].pin);
        uint8_t raw = (pin == KEY_PRESSED_LEVEL) ? 1 : 0;

        if (raw != stable_state[i]) {
            if (counter[i] < 0xFF) {
                counter[i]++;
            }
            if (counter[i] >= DEBOUNCE_TICKS) {

                uint8_t old_state = stable_state[i];
                stable_state[i] = raw;
                counter[i] = 0;


                if (old_state == 0 && stable_state[i] == 1) {
                    click_flag[i] = 1;
                }
            }
        } else {

            counter[i] = 0;
        }
    }
}

uint8_t Key_WasPressed(uint8_t key_id)
{
    if (key_id >= KEY_COUNT) {
        return 0;
    }

    if (click_flag[key_id]) {
        click_flag[key_id] = 0; 
        return 1;
    }
    return 0;
}