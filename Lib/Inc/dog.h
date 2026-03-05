//
// Created by SlimeTommy on 2025/10/27.
//aaaa

#ifndef PARALLELDOG_F429_TEST1_DOG_H
#define PARALLELDOG_F429_TEST1_DOG_H
#include "stdint.h"
#include "GO_ctrl.h"
#include "curve.h"
#include "offroad_autoctrl.h" // 移除此包含，避免循环依赖

#define RC_MODE      0
#define AUTO_OFFROAD 1
typedef enum leg_state_{//单腿状态枚举 每个动作的第一阶段状态必须为0
	STAND_UP=0,
	
	STEP_FORE1=0,
	KICK_BACK1,
	STEP_FORE2,
	KICK_BACK2,
	STEP_FORE_REBACK,
	KICK_BACK_REBACK,
	
	STEP_B1=0,	STEP_F1,	KICK_F1,	KICK_B1,
	STEP_B2,	STEP_F2,	KICK_F2,	KICK_B2,STEP_F_REBACK,STEP_B_REBACK,KICK_F_REBACK,KICK_B_REBACK,
	
	LOWSTEP_FORE1=0,
	LOWKICK_BACK1,
	LOWSTEP_FORE2,
	LOWKICK_BACK2,
	
	JUMP_PREPARE=0,
	JUMPING, 
	JUMP_LANDING,
	JUMP_BUFFER,
	
	INJUMP_PREPARE=0,
	INJUMP_TAKEOFF,
	INJUMP_LANDING,
	
	STEP_FORE1_BACK=0,
	KICK_BACK1_BACK,
	STEP_FORE2_BACK,
	KICK_BACK2_BACK,
	
	DAMPING=0
}leg_state;

typedef struct act_bezier_xpect{
	float T;  //动作时间间隔
	float fre;//取样频率	
	uint8_t n;//曲线阶数
	pid_ pid;
	float t;//曲线比例系数
	uint8_t point_sum;
	uint8_t flag;//解算完成标志0/1
	bezierPoint  ctrl_point[4];
	bezierPoint pos;
	float exp_fvel[2];//x,y
	uint32_t now_time;
	uint32_t last_end_time;
}bezier_exp;

typedef struct Leg_para{//单腿参数与对应电机参数结构体
    uint8_t id;
    float x;//
    float y;
	float fvel[2];//足端x,y速度
    float L1;//大腿长mm
    float L2;//小腿长
    float theta_fore;//后侧分腿角度，相对于x轴正向旋转总角度
    float theta_back;//前侧分腿角度
	float F[2];//足端虚拟力
    float A;
	leg_state state;
    GO motor_ctrl_linkf;
    GO motor_ctrl_linkb;
	bezier_exp bezier;
}Leg;
typedef enum{
    STAND_UP_=0,
    WALK_FORWARD,
	TURN_RIGHT,
	TURN_LEFT,
	LOWWALK_FORWARD,
	JUMP_FORWARD,
	INJUMP,
	WALK_BACK,
	DAMPING_MODE,
}dog_state;
typedef struct dog_location{
	float pos[2];//x,y位置
	float yaw;//偏航角
}dog_location;


typedef struct{
	dog_state state;
	Leg leg[4];
	uint8_t dog_mode;//
	dog_location location;//方位信息
	auto_ctrlcenter auto_ctrl;
}Dog;


void dog_task(void);
void dog_task_stand(void);













#endif //PARALLELDOG_F429_TEST1_DOG_H
