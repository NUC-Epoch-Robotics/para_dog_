#include "stdlib.h"
#include <stdint.h>
#include <string.h>
#include "vofa_Debug.h"
#include "usart.h"
#define VOFA_DATA_SIZE 10
float vofa_data[VOFA_DATA_SIZE];
float last_vofa_data[VOFA_DATA_SIZE];
uint8_t vofa_rx_frame[6];

void vofa_init(UART_HandleTypeDef *huart){
  HAL_UART_Receive_IT(huart, vofa_rx_frame, 6);
}
float vofa_GetData(uint8_t *data)
{
  static float num = 0.0f;

  if (data == NULL)
  {
    return num;
  }
  // Frame format: 0x5A + 4-byte payload + '\n' (0x0A).
  if ((data[0] == 0x5A || data[0] == 0x5B || data[0] == 0x5C) && (data[5] == 0x0A))
  {
    uint8_t is_ascii_payload =
        ((data[1] >= 0x20) && (data[1] <= 0x7E)) &&
        ((data[2] >= 0x20) && (data[2] <= 0x7E)) &&
        ((data[3] >= 0x20) && (data[3] <= 0x7E)) &&
        ((data[4] >= 0x20) && (data[4] <= 0x7E));

    if (is_ascii_payload)
    {
      char buf[5];
      char *endptr;
      float parsed;
      memcpy(buf, &data[1], 4);
      buf[4] = '\0';
      parsed = strtof(buf, &endptr);
      if (endptr == buf)
      {
        return num;
      }
      num = parsed;
    }
    else
    {
      uint32_t raw = ((uint32_t)data[1]) |
                     ((uint32_t)data[2] << 8) |
                     ((uint32_t)data[3] << 16) |
                     ((uint32_t)data[4] << 24);
      memcpy(&num, &raw, sizeof(num));
    }
    switch (data[0])
    {
    case 0x5A:
      vofa_data[0] = num;
      break;
    case 0x5B:
      vofa_data[1] = num;
      break;
    case 0x5C:
      vofa_data[2] = num;
      break;
    }
  }
  return num;
}

void vofa_rc(Dog *dog)
{
  if (vofa_data[0] == 0)
  {
    dog->state = STAND_UP_;
  }
  else if (vofa_data[0] == 1)
  {
    dog->state = WALK_FORWARD;
  }
  else if (vofa_data[0] == 2)
  {
    dog->state = WALK_BACK;
  }
  else if (vofa_data[0] == 3)
  {
    dog->state = TURN_RIGHT;
  }
  else if (vofa_data[0] == 4)
  {
    dog->state = TURN_LEFT;
  }
  else if (vofa_data[0] == 5)
  {
    dog->state = LOWWALK_FORWARD;
  }
  else if (vofa_data[0] == 6)
  {
    dog->state = JUMP_FORWARD;
  }
  else if (vofa_data[0] == 7)
  {
    dog->state = INJUMP;
  }
  else if (vofa_data[0] == 8)
  {
    dog->state = DAMPING_MODE;
  }
  else if (vofa_data[0] == 9)
  {
    dog->state = TROT_ROTATEJUMP;
  }
  else
  {
    dog->state = STAND_UP_;
  }
}
void vofa_data_write(uint8_t index, float *receiver)
{
  if (receiver == NULL)
  {
    return;
  }

  if (index >= VOFA_DATA_SIZE)
  {
    *receiver = 0.0f;
    return;
  }

  if (last_vofa_data[index] != vofa_data[index])
  {
    last_vofa_data[index] = vofa_data[index];
    *receiver = last_vofa_data[index];
  }

  return;
}

/********************************************************
函数名：  	HAL_UART_RxCpltCallback
日期：    	2025.11.12
功能：    	UART接收完成中断回调函数
输入参数：		UART句柄指针
返回值： 		无
修改记录：
**********************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    vofa_GetData(vofa_rx_frame);
    
    HAL_UART_Receive_IT(&huart2, vofa_rx_frame, 6);
  }
}
