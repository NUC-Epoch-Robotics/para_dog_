#include "usart.h"
#include <string.h>
#include "A28_RC.h"

#define FRAME_HEAD 0x7E
#define FRAME_TAIL 0x7F
uint8_t lora_rx_byte=0;
uint8_t lora_rx_buf[256];
uint16_t lora_rx_len = 0;
int16_t channel[4];   // 4个通道数据
uint8_t key_state[8]; // 8个按键
volatile uint32_t lora_uart_error_code = 0;

/********************************************************
函数名：  	Protocol_ReceiveHandler （协议接收数据包）
日期：    	2025.11.12
功能：    	通过A28接收一个带帧头帧尾的数据包
输入参数：		接收到的字节，输出缓冲区指针和长度指针
返回值： 		0=未收到完整帧，1=收到完整帧
修改记录：  2025.11.20优化
**********************************************************/
uint8_t Protocol_ReceiveHandler(uint8_t byte, uint8_t *out_buf, uint16_t *out_len)
{
    static uint8_t rx_buf[256];
    static uint16_t rx_len = 0;
    static uint8_t in_frame = 0;

    if (byte == FRAME_HEAD)
    {
        // 新帧开始，丢弃前面所有内容
        in_frame = 1;
        rx_len = 0;
        return 0;
    }
    if (in_frame)
    {
        if (byte == FRAME_TAIL)
        {
            // 收到帧尾，输出数据
            if (rx_len > 0 && out_buf && out_len)
            {
                memcpy(out_buf, rx_buf, rx_len);
                *out_len = rx_len;
            }
            in_frame = 0;
            rx_len = 0;
            return 1;
        }
        else
        {
            if (rx_len < 256)
            {
                rx_buf[rx_len++] = byte;
            }
            else
            {
                // 超长，丢弃本帧
                in_frame = 0;
                rx_len = 0;
            }
        }
    }
    return 0;
}

/********************************************************
函数名：  	PackChannels
日期：    	2025.11.16
功能：    	将接收到的8字节原始通道数据打包成16位通道值
输入参数：		原始数据数组，输出通道数组指针
返回值： 		无
修改记录：
**********************************************************/
void PackChannels(const uint8_t raw[8], uint16_t channel[4])
{
    channel[0] = (raw[0] << 8) | raw[1];
    channel[1] = (raw[2] << 8) | raw[3];
    channel[2] = (raw[4] << 8) | raw[5];
    channel[3] = (raw[6] << 8) | raw[7];
}

/********************************************************
函数名：  	UnpackRockerAndKeys
日期：    	2025.11.20
功能：    	解析摇杆(将接收到的8字节原始通道数据打包成16位通道值)和按键数据
输入参数：		原始数据数组，输出通道数组指针，按键状态数组指针
返回值： 		无
修改记录：
**********************************************************/
void UnpackRockerAndKeys(const uint8_t raw[16], uint16_t channel[4], uint8_t keys[8])
{
    channel[0] = (raw[0] << 8) | raw[1];
    channel[1] = (raw[2] << 8) | raw[3];
    channel[2] = (raw[4] << 8) | raw[5];
    channel[3] = (raw[6] << 8) | raw[7];
    for (int i = 0; i < 8; i++)
    {
        keys[i] = raw[8 + i];
    }
}
void A28_RC(Dog *dog)
{
    if (key_state[0] == 1)
    {
        dog->state = WALK_FORWARD;
    }
    else if (key_state[1] == 1)
    {
        dog->state = TURN_RIGHT;
    }
    else if (key_state[2] == 1)
    {
        dog->state = WALK_BACK;
    }
    else if (key_state[3] == 1)
    {
        dog->state = TURN_LEFT;
    }
    else
    {
        dog->state = STAND_UP_;
    }
}
// /********************************************************
// 函数名：  	HAL_UART_RxCpltCallback
// 日期：    	2025.11.12
// 功能：    	UART接收完成中断回调函数
// 输入参数：		UART句柄指针
// 返回值： 		无
// 修改记录：
// **********************************************************/
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == USART3)
//     {
//         if (Protocol_ReceiveHandler(lora_rx_byte, lora_rx_buf, &lora_rx_len))
//         {
//             UnpackRockerAndKeys(lora_rx_buf, (uint16_t *)channel, key_state);
//         }
//         HAL_UART_Receive_IT(&huart3, &lora_rx_byte, 1);
//     }
// }
// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
// {
//     if (huart != &huart3)
//     {
//         return;
//     }

//     lora_uart_error_code = huart->ErrorCode;
//     __HAL_UART_CLEAR_OREFLAG(huart);
//     __HAL_UART_CLEAR_FEFLAG(huart);
//     __HAL_UART_CLEAR_NEFLAG(huart);
//     __HAL_UART_CLEAR_PEFLAG(huart);
//     HAL_UART_AbortReceive(huart);
//     HAL_UART_Receive_IT(&huart3, &lora_rx_byte, 1);
// }
/* USER CODE END 4 */
