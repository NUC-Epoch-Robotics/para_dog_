#ifndef __HWT605_H
#define __HWT605_H

#include "stdint.h"
#include "can.h"

//extern yaw_t yaw;

typedef struct wit
{
	float fRoll;
	float fPitch;
	float fYaw;
	float total_angle;
	float last_Yaw;
	float Gx;
	float Gy;
	float Gz;
}wit_t;

void OnRegUpdate(uint32_t reg, uint32_t len);
void MyCanWrite(uint8_t ucId, uint8_t *p_ucData, uint32_t ucLen);
void hwt605_Init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
//void update_yaw_target(yaw_t *yaw, float w, float dt);





#endif


