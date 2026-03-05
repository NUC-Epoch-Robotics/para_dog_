//
// Created by SlimeTommy on 25-10-9.
//
#include "GO_ctrl.h"
HAL_StatusTypeDef GO_init(GO *motor_ctrl,uint8_t id)
{
	motor_ctrl->id=id;//设置电机ID
	motor_ctrl->cmd.id=id;
	motor_ctrl->cmd.mode=0;
	motor_ctrl->cmd.K_P=0;
	motor_ctrl->cmd.K_W=0;
	motor_ctrl->cmd.Pos=0;
	motor_ctrl->cmd.W=0;
	motor_ctrl->cmd.T=0;
    HAL_StatusTypeDef ret = SERVO_Send_recv(&motor_ctrl->cmd, &motor_ctrl->data);
    if (ret != HAL_OK) {
        return ret;
    }
	motor_ctrl->init_pos=motor_ctrl->data.Pos;
    return HAL_OK;

}
void GO_PosMode_Ctrl(GO *motor_ctrl,uint8_t mode){
	
	motor_ctrl->cmd.id  =motor_ctrl->id;
	motor_ctrl->cmd.mode=mode;
	motor_ctrl->cmd.K_P =motor_ctrl->pid.K_P;
	motor_ctrl->cmd.K_W =motor_ctrl->pid.K_W;
	motor_ctrl->cmd.Pos =motor_ctrl->pid.Pos;
	motor_ctrl->cmd.W   =motor_ctrl->pid.W;
	motor_ctrl->cmd.T   =0;
	
	SERVO_Send_recv(&motor_ctrl->cmd,&motor_ctrl->data);
}
void GO_TorqueMode_Ctrl(GO *motor_ctrl,uint8_t mode){
	
	motor_ctrl->cmd.id  =motor_ctrl->id;
	motor_ctrl->cmd.mode=mode;
	motor_ctrl->cmd.K_P =0;
	motor_ctrl->cmd.K_W =0;
	motor_ctrl->cmd.Pos =0;
	motor_ctrl->cmd.W   =0;
	motor_ctrl->cmd.T   =motor_ctrl->pid.T;
	
	SERVO_Send_recv(&motor_ctrl->cmd,&motor_ctrl->data);
}
//float GO_act_pid[29][2] = {//i
//    //legID->动作曲线->pid系数
//		{0.3f, 0.05f},//STAND_UP_		
//		{0.3f, 0.01f},{0.3f, 0.01f},{0.3f, 0.01f},{0.3f, 0.01f},//WALK_FORWARD
//		{0.3f, 0.01f},{0.3f, 0.01f},{0.3f, 0.01f},{0.3f, 0.01f},{0.3f, 0.01f},{0.3f, 0.01f},{0.3f, 0.01f},{0.3f, 0.01f},//TURN_RIGHT+TURN_LEFT
//		{0.6f, 0.01f},{0.6f, 0.01f},{0.6f, 0.01f},{0.6f, 0.01f},//LOWWALK_FORWARD
//		{0.5f, 0.05f},{3.0f, 0.01f},{0.2f, 0.01f},{0.2f, 0.010f},//JUMP_FORWARD
//		{0.2f, 0.01f},{3.0f, 0.01f},{0.2f, 0.01f},//INJUMP
//		{0.6f, 0.01f},{0.6f, 0.01f},{0.6f, 0.01f},{0.6f, 0.01f},//WALK_BACK
//		{0.0f, 0.01f}
//};
//float GO_act_exp[29][3] ={//Pos W T
//		{0, 0, 0},//STAND_UP_
//		{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},//WALK_FORWARD
//		{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},//TURN_RIGHT
//		{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},//LOWWALK_FORWARD
//		{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},//JUMP_FORWARD
//		{0, 0, 0},{0, 0, 0},{0, 0, 0},//INJUMP
//		{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},//WALK_BACK
//		{0, 0, 0}
//};
















