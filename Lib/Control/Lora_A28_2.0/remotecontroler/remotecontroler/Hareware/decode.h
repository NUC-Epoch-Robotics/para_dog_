#ifndef __DECODE_H
#define __DECODE_H
#include "main.h"

#define data_max 64

  typedef enum
{
    frame_header_FF = 0,
    frame_header_55,
    command_H,
    command_L,
    len_t,
    data,
}state_t;
typedef  struct
{
    state_t st;
    uint16_t cmd;
    uint8_t len;
    uint8_t idx;
    uint8_t data_buf[data_max];
} frame_parser_t;



void usart_start_rx_dma(void);
void parser_init(frame_parser_t *ps);
void parser(frame_parser_t *ps, const uint8_t *buf , uint16_t len);
void usart2_rx_check(void);



#endif

