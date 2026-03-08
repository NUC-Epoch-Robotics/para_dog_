#include "hwt605.h"
#include "wit_c_sdk.h"
#include <string.h>
#include <stdio.h>
#include "can.h"
//#include "resolution.h"  

wit_t hwt_angle;

// 角度更新回调
void OnRegUpdate(uint32_t reg, uint32_t len)
{
    if  (reg == GX)
    {
        int16_t gx = sReg[GX];
        int16_t gy = sReg[GY];
        int16_t gz = sReg[GZ];

        /* Raw gyro is scaled to deg/s per datasheet (±2000dps full scale). */
        hwt_angle.Gx = ((float)gx / 32768.0f * 2000.0f);
        hwt_angle.Gy = ((float)gy / 32768.0f * 2000.0f);
        hwt_angle.Gz = ((float)gz / 32768.0f * 2000.0f);
    }
	
    if (reg == Roll)
    { 
        int16_t roll  = sReg[Roll]; 
        int16_t pitch = sReg[Pitch];   
        int16_t yaw   = sReg[Yaw]; 

        hwt_angle.fRoll  = ((float)roll  / 32768.0f * 180.0f); 
        hwt_angle.fPitch = ((float)pitch / 32768.0f * 180.0f);
        hwt_angle.fYaw   = (float)yaw / 32768.0f * 180.0f;
    }
}

// CAN2发送回调（WIT SDK使用）
void MyCanWrite(uint8_t ucId, uint8_t *p_ucData, uint32_t ucLen)
{
    CAN_TxHeaderTypeDef CANTX;
    uint32_t TXBOX;

    CANTX.DLC = ucLen;
    CANTX.ExtId = 0;
    CANTX.IDE = CAN_ID_STD;
    CANTX.RTR = CAN_RTR_DATA;
    CANTX.StdId = 0x0050;

    HAL_CAN_AddTxMessage(&hcan2, &CANTX, p_ucData, &TXBOX);
}

// HWT605 初始化（CAN2）
void hwt605_Init(void)
{
    CAN_FilterTypeDef can_filter_st;

    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;

    // 只接收 ID = 0x0050 的数据帧
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow  = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow  = 0x0000;

    can_filter_st.FilterBank = 14;
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    // WIT SDK 初始化
    WitInit(WIT_PROTOCOL_CAN, 0x0050);
    WitCanWriteRegister(MyCanWrite);
    WitRegisterCallBack(OnRegUpdate);
}

