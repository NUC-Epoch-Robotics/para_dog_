#include "remote_tx.h"
#include "stdio.h"
#include "usart.h"



static volatile uint8_t uart1_tx_busy = 0;
static uint8_t uart1_tx_buf[16];
static uint8_t uart1_mote_tx_buf[6];

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else             crc = (crc << 1);
        }
    }
    return crc;
}
int joy_send(int16_t joy_x , int16_t joy_y ,int16_t joy_z)
{
    static uint8_t seq = 0;
    
    if(uart1_tx_busy)return -1;

    uart1_tx_buf[0] = 0xAA;
    uart1_tx_buf[1] = 0x55;
    uart1_tx_buf[2] = seq++;

    uart1_tx_buf[3] = (uint8_t)(joy_x & 0xFF);
    uart1_tx_buf[4] = (uint8_t)((joy_x >> 8)&0xFF);

    uart1_tx_buf[5] = (uint8_t)(joy_y & 0xFF);
    uart1_tx_buf[6] = (uint8_t)((joy_y >> 8)&0xFF);
	
	uart1_tx_buf[7] = (uint8_t)(joy_z & 0xFF);
	uart1_tx_buf[8] = (uint8_t)((joy_z >> 8)&0xFF);
	
	

    uart1_tx_buf[9] = 0;

    uint16_t crc = crc16_ccitt(uart1_tx_buf, 10);
    uart1_tx_buf[10] = (uint8_t)(crc & 0xFF);
    uart1_tx_buf[11] = (uint8_t)((crc >> 8) & 0xFF);

    uart1_tx_busy = 1;
    if(HAL_UART_Transmit_DMA(&huart1, uart1_tx_buf, 12) != HAL_OK)
    {
        uart1_tx_busy = 0;
        return -3;
    }
    return 0;
}
int mode_send(uint8_t mode_bits)
{
    static uint8_t seq = 0;
    
    if(uart1_tx_busy)return -1;

    uart1_mote_tx_buf[0] = 0xAA;
    uart1_mote_tx_buf[1] = 0x56;
    uart1_mote_tx_buf[2] = seq++;

	uart1_mote_tx_buf[3] = mode_bits;
    uint16_t crc = crc16_ccitt(uart1_mote_tx_buf, 4);
    uart1_mote_tx_buf[4] = (uint8_t)(crc & 0xFF);
    uart1_mote_tx_buf[5] = (uint8_t)((crc >> 8) & 0xFF);

    uart1_tx_busy = 1;
    if(HAL_UART_Transmit_DMA(&huart1, uart1_mote_tx_buf, 6) != HAL_OK)
    {
        uart1_tx_busy = 0;
        return -3;
    }
    return 0;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        uart1_tx_busy = 0;
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        uart1_tx_busy = 0;
    }
}

