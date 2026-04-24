#include "decode.h"
#include <string.h>
extern UART_HandleTypeDef huart2;

static void frame_on(uint16_t cmd , const uint8_t *data , uint8_t len);
#define UART_RX_BUF_SIZE 512

uint8_t uart2_rx_buf[UART_RX_BUF_SIZE];
static uint16_t uart_old_pos = 0;

frame_parser_t parser_t;

volatile float g_last_value;

void usart_start_rx_dma(void)
{
    HAL_UART_Receive_DMA(&huart2, uart2_rx_buf, UART_RX_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart2 , UART_IT_IDLE);
}
void usart2_rx_check(void)
{
    uint16_t pos_t = UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    if( pos_t != uart_old_pos)
    {
    if(pos_t > uart_old_pos)
    {
       parser(&parser_t , &uart2_rx_buf[uart_old_pos] , pos_t - uart_old_pos);
    }else {
    parser(&parser_t, &uart2_rx_buf[uart_old_pos], UART_RX_BUF_SIZE - uart_old_pos);
    if(pos_t > 0)parser(&parser_t,&uart2_rx_buf[0], pos_t);
    }
    uart_old_pos = pos_t;
    }
}
static void frame_on(uint16_t cmd , const uint8_t *data , uint8_t len)
{
    if(len == 4)
    {
        float f;
        uint8_t le[4] = {data[3],data[2],data[1],data[0]};
        memcpy(&f ,le , 4);
		g_last_value = f;
    } 
}
void parser_init(frame_parser_t *ps)
{
    memset(ps , 0 , sizeof(*ps));
    ps->st = frame_header_FF;
}
void parser(frame_parser_t *ps, const uint8_t *buf , uint16_t len1)
{
    for(uint16_t i = 0 ; i < len1 ; i++)
    {
        uint8_t b = buf[i];

        switch(ps->st)
        {
            case frame_header_FF:
            if(b == 0xFF)ps->st = frame_header_55;
            break;

            case frame_header_55:
            if(b == 0x55)ps->st = command_H;
            else if(b == 0xFF)ps->st = frame_header_55;
            else ps->st = frame_header_55;
            break;

            case command_H:
            ps->cmd = ((uint16_t)b) << 8;
            ps->st = command_L;
            break;

            case command_L:
            ps->cmd |= b;
            ps->st = len_t;
            break;

            case len_t:
            ps->len = b;
            if(ps->len > data_max)
            {
                ps->st = frame_header_FF;
                break;
            }
            ps->idx = 0;
            ps->st = (ps->len == 0 ) ? frame_header_FF : data;
            if(ps->len == 0)frame_on(ps->cmd , ps->data_buf , ps->len);
            break;

            case data:
            ps->data_buf[ps->idx++] = b;
            if(ps->idx >= ps->len)
            {
                frame_on(ps->cmd , ps->data_buf , ps->len);
                ps->st = frame_header_FF;
            }
            break;

            default:
            ps->st = frame_header_FF;
            break;
        }

    }

}
