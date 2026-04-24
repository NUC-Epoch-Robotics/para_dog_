#ifndef __APP_REMOTE_RX_H
#define __APP_REMOTE_RX_H

#include "main.h"
#include "cmsis_os.h"

#define SOF0 0xAA
#define SOF1 0x55
#define SOF1_MODE 0x56
#define data_len_1 12
#define mode_len 6

// 环形缓冲区大小
#define RX_BUF_SIZE 64
#define rx_flag_data (1u << 0)

// 遥控器解析数据结构体
typedef struct
{
	uint8_t flag;
	uint8_t seq;
	uint8_t link_ok;
	uint8_t new_data;
	uint8_t mode;
	float x;
	float y;
	float z;
	uint32_t last_rx_tick;
	uint32_t ok_cnt;
	uint32_t crc_err_cnt;
	uint32_t lost_cnt;
} remote_rx_data;

typedef struct
{
	int16_t vx;
	int16_t vy;
	int16_t vw;

	float vx_out;
	float vx_out_last;

	float vy_out;
	float vy_out_last;

	float vw_out;
	float vw_out_last;
} speed_t;

void app_remote_rx_init(UART_HandleTypeDef *huart);
void app_remote_rx_start(void);
void UART_IDLE_Callback(UART_HandleTypeDef *huart);
void app_remote_thread(osThreadId id);
void remote_rx_on(const uint8_t *data, uint16_t len);

#endif
