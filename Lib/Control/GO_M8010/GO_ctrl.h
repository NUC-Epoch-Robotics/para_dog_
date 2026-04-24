//
// Created by SlimeTommy on 25-10-15.
//

#ifndef GO_CTRL_H
#define GO_CTRL_H
#include "motor_control.h"

typedef struct pid_para {
    float K_P;
    float K_W;
    float K_I;  
    float xyPosIntegral[2];
    float Pos;
    float W;
    float T;
}pid_;


typedef struct GO_Ctrl//8010电机控制结构体
{
    MOTOR_send cmd;
    MOTOR_recv data;
    uint8_t mode;
    uint8_t id;
    pid_ pid;
    float init_pos;
}GO;
HAL_StatusTypeDef GO_init(GO *motor_ctrl,uint8_t id,uint8_t mode);
void GO_PosMode_Ctrl(GO *motor_ctrl,uint8_t mode);
void GO_TorqueMode_Ctrl(GO *motor_ctrl,uint8_t mode);
void GO_PFC_Ctrl(GO *motor_ctrl,uint8_t mode);
#endif //GO_CTRL_H
