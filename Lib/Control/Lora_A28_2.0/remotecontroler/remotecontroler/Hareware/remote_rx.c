//#include "remote_rx.h"
//#include "usart.h"
//#include <string.h>

//#define SOF0         0xAA
//#define SOF1         0x55
//#define data_len       10


//static UART_HandleTypeDef *s_huart = NULL;

//static uint8_t rx_buf[128];

//static struct
//{
//    uint8_t buf[data_len];
//    uint8_t idx;
//}rx_cache = {0};

//volatile remote_rx_data rx_data = {0};

//static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
//{
//    uint16_t crc = 0xFFFF;
//    for(uint16_t i = 0; i<len ; i++)
//    {
//        crc ^= (uint16_t)data[i] << 8;
//        for(uint8_t b = 0; b < 8; b++)
//        {
//            if(crc & 0x8000)crc = (crc << 1) ^ 0x1021;
//            else            crc = (crc << 1);
//        }
//    }
//    return crc;
//}
//static void handle_frame(const uint8_t f[data_len])
//{
//    uint16_t crc_calc = crc16_ccitt(f, 8);
//    uint16_t crc_rc = (uint16_t)f[8] | ((uint16_t)f[9] << 8);  

//    if(crc_calc != crc_rc)
//    {
//        rx_data.crc_err_cnt++;
//        return;
//    }

//    uint8_t seq = f[2];

//    int16_t joy_x = (int16_t)((int16_t)f[3] | ((int16_t)f[4] << 8));
//    int16_t joy_y = (int16_t)((int16_t)f[5] | ((int16_t)f[6] << 8));
//    uint8_t flags = f[7];

//    if(rx_data.ok_cnt > 0)
//    {
//        uint8_t expected = (uint8_t)(rx_data.seq + 1);

//        if(seq != expected)
//        {
//            uint8_t diff = (uint8_t)(seq - expected);
//            rx_data.lost_cnt += diff;
//        }
//    }
//    rx_data.seq = seq;
//    rx_data.x = joy_x;
//    rx_data.y = joy_y;
//    rx_data.flag = flags;

//    rx_data.last_rx_tick = HAL_GetTick();
//    rx_data.ok_cnt++;
//    rx_data.link_ok = 1;
//    rx_data.new_data = 1;
//    
//}
//void remoterx_init(UART_HandleTypeDef *huart)
//{
//    s_huart = huart;
//    memset((void *)&rx_data, 0, sizeof(rx_data));
//    memset(&rx_cache, 0, sizeof(rx_cache));
//}
//void remote_start(void)
//{
//    if(!s_huart)return;

//    HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(s_huart, rx_buf, sizeof(rx_buf));
//	    if (ret != HAL_OK) {
//        while(1);
//    }
//    __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);
//}
//void remote_rx_on(const uint8_t *data, uint16_t len)
//{
//    for(uint16_t i = 0; i < len; i++)
//    {
//        uint8_t c = data[i];

//        switch(rx_cache.idx)
//        {
//            case 0:
//                if(c == SOF0)
//                {
//                    rx_cache.buf[0] = SOF0;
//                    rx_cache.idx = 1;
//                }
//                break;

//            case 1:
//                if(c == SOF1)
//                {
//                    rx_cache.buf[1] = SOF1;
//                    rx_cache.idx = 2;
//                }else if(c == SOF0)
//                {
//                    rx_cache.buf[0] = SOF0;
//                    rx_cache.idx = 1;
//                }else {
//                    rx_cache.idx = 0;
//                }
//                break;

//            default:
//                
//            rx_cache.buf[rx_cache.idx++] = c;

//              if(rx_cache.idx >= data_len)
//              {
//                handle_frame(rx_cache.buf);
//                rx_cache.idx = 0;
//              }
//              break;
//        }
//    }
//}
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//    if(huart->Instance == USART1)
//    {
//        remote_rx_on(rx_buf, Size);
//        HAL_UARTEx_ReceiveToIdle_DMA(s_huart, rx_buf, sizeof(rx_buf));
//        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
//    }
//}


