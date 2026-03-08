//
// Created by SlimeTommy on 25-10-13.
//
#include "postrue_control.h"
#include <math.h>
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include "cmsis_os.h"
#include "GO_ctrl.h"
#include "simple_matrix.h"
#define pi 3.141592f
#define VMC_FOOT_FORCE_LIMIT 300.0f
#define VMC_TORQUE_LIMIT 12.0f

volatile uint32_t g_vmc_force_sat_count[4] = {0};
volatile uint32_t g_vmc_torque_sat_count[4] = {0};

static float vmc_clamp(float value, float limit, uint8_t *is_sat)
{
	if (value > limit)
	{
		*is_sat = 1;
		return limit;
	}
	if (value < -limit)
	{
		*is_sat = 1;
		return -limit;
	}
	return value;
}
void Dog_ParaInit(Dog *dog)
{
	memset(&dog->leg, 0, sizeof(Leg) * 4);
	dog->state = STAND_UP_;
	memset(&dog->location, 0, sizeof(dog_location));	 //
	memset(&dog->auto_ctrl, 0, sizeof(auto_ctrlcenter)); // 初始化自动控制参数
	for (uint8_t i = 0; i < 4; i++)
	{
		dog->leg[i] = (Leg){
			.L1 = 110,
			.L2 = 220,
			.id = i + 1,
			.state = STAND_UP,
			.theta_fore = 0,
			.theta_back = 0};
	}

	for (uint8_t i = 0; i < 4; i++)
	{
		if (GO_init(&dog->leg[i].motor_ctrl_linkf, (dog->leg[i].id) * 2 - 1) != HAL_OK)
		{
			//            Error_Handler();
		}
		if (GO_init(&dog->leg[i].motor_ctrl_linkb, (dog->leg[i].id) * 2) != HAL_OK)
		{
			//            Error_Handler();
		}
	}
}
void leg_Inverse_Kinematics(Leg *leg)
{ // 给定x,y,驱动电机旋转

	// 设置足端坐标
	leg->x = leg->bezier.pos.x;
	leg->y = leg->bezier.pos.y;
	// 计算电机转子旋转绝对位置
	leg->A = pow(leg->x, 2) + pow(leg->y, 2) + pow(leg->L1, 2) - pow(leg->L2, 2);
	leg->theta_fore = -acos(leg->A / ((2 * leg->L1) * sqrtf(pow(leg->x, 2) + pow(leg->y, 2)))) + atan2(leg->y, leg->x); // rad连杆位置
	leg->theta_back = acos(leg->A / ((2 * leg->L1) * sqrtf(pow(leg->x, 2) + pow(leg->y, 2)))) + atan2(leg->y, leg->x);

	if (leg->id == 1)
	{
		leg->motor_ctrl_linkf.pid.Pos = (leg->theta_fore * 6.33f) + (leg->motor_ctrl_linkf.init_pos + 2.15857f * 6.33f);
		leg->motor_ctrl_linkb.pid.Pos = -(leg->theta_back * 6.33f) + (leg->motor_ctrl_linkb.init_pos + 2.78744f * 6.33f);
	}
	else if (leg->id == 2)
	{
		leg->motor_ctrl_linkf.pid.Pos = -(leg->theta_fore * 6.33f) + (leg->motor_ctrl_linkf.init_pos - 2.15857f * 6.33f);
		leg->motor_ctrl_linkb.pid.Pos = (leg->theta_back * 6.33f) + (leg->motor_ctrl_linkb.init_pos - 2.78744f * 6.33f);
	}
	else if (leg->id == 3)
	{
		leg->motor_ctrl_linkf.pid.Pos = (leg->theta_fore * 6.33f) + (leg->motor_ctrl_linkf.init_pos - 0.352552f * 6.33f);
		leg->motor_ctrl_linkb.pid.Pos = -(leg->theta_back * 6.33f) + (leg->motor_ctrl_linkb.init_pos + 5.298575f * 6.33f);
	}
	else if (leg->id == 4)
	{
		leg->motor_ctrl_linkf.pid.Pos = -(leg->theta_fore * 6.33f) + (leg->motor_ctrl_linkf.init_pos + 0.352552f * 6.33f);
		leg->motor_ctrl_linkb.pid.Pos = (leg->theta_back * 6.33f) + (leg->motor_ctrl_linkb.init_pos - 5.298575f * 6.33f);
		//		leg->motor_ctrl_linkb.pid.Pos = (leg->motor_ctrl_linkb .init_pos);
	}
}
void leg_Forward_Kinematics(Leg *leg)
{
	float Alpha1, Alpha2, a, b, deta, phi;
	Alpha1 = (56.26f / 180) * pi;
	Alpha2 = (20.21f / 180) * pi;
	a = leg->motor_ctrl_linkf.data.Pos - leg->motor_ctrl_linkf.init_pos;
	b = leg->motor_ctrl_linkb.data.Pos - leg->motor_ctrl_linkb.init_pos;
	if (leg->id == 1)
	{
		leg->theta_fore = -(pi - Alpha1) + a;
		leg->theta_back = (pi - Alpha2) - b;
	}
	else if (leg->id == 2)
	{
		leg->theta_fore = -(pi - Alpha1) - a;
		leg->theta_back = (pi - Alpha2) + b;
	}
	else if (leg->id == 3)
	{
		leg->theta_fore = Alpha1 + a;
		leg->theta_back = -Alpha2 - b;
	}
	else if (leg->id == 4)
	{
		leg->theta_fore = Alpha1 - a;
		leg->theta_back = -Alpha2 + b;
	}
	deta = 1 / 2 * (leg->theta_back - leg->theta_fore);
	phi = 1 / 2 * (leg->theta_fore + leg->theta_back);
	leg->x = cosf(phi) * (110 * cosf(-deta) + 55 * sqrtf(2.0f) * sqrtf(cosf(-2 * deta) + 7.0f));
	leg->y = sinf(phi) * (110 * cosf(-deta) + 55 * sqrtf(2.0f) * sqrtf(cosf(-2 * deta) + 7.0f));
}
void pos_Leg_MotorCtrl(Leg *leg)
{
	leg_Inverse_Kinematics(leg);

	// 前肢
	leg->motor_ctrl_linkf.pid.K_P = leg->bezier.pid.K_P; // K_p
	leg->motor_ctrl_linkf.pid.K_W = leg->bezier.pid.K_W; // K_w
	leg->motor_ctrl_linkf.pid.W = leg->bezier.pid.W;	 // W

	// 后肢
	leg->motor_ctrl_linkb.pid.K_P = leg->bezier.pid.K_P; // K_p
	leg->motor_ctrl_linkb.pid.K_W = leg->bezier.pid.K_W; // K_w
	leg->motor_ctrl_linkb.pid.W = leg->bezier.pid.W;	 // W

	// 启动电机
	GO_PosMode_Ctrl(&(leg->motor_ctrl_linkf), 1);
	GO_PosMode_Ctrl(&(leg->motor_ctrl_linkb), 1);
}
static void leg_Calc_Jacobian(const Leg *leg, float Jacobian[2][2])
{
	float sigma[6];
	sigma[0] = sqrtf(cosf(leg->theta_fore - leg->theta_back) + 7.0f);
	sigma[1] = 0.5f * leg->theta_fore - 1.5f * leg->theta_back;
	sigma[2] = 1.5f * leg->theta_fore - 0.5f * leg->theta_back;
	sigma[5] = 0.5f * (leg->theta_fore + leg->theta_back);
	sigma[3] = (385 * sqrtf(2.0f) * cosf(sigma[5])) / 2;
	sigma[4] = 385 * sqrtf(2.0f) * sinf(sigma[5]);
	Jacobian[0][0] = -(110 * sinf(leg->theta_fore) * sigma[0] + sigma[4] + 55 * sqrtf(2.0f) * sinf(sigma[2])) / (2 * sigma[0]);
	Jacobian[0][1] = -(110 * sinf(leg->theta_back) * sigma[1] + sigma[4] - 55 * sqrtf(2.0f) * cosf(sigma[1])) / (2 * sigma[0]);
	Jacobian[1][0] = (55 * cosf(leg->theta_fore) * sigma[0] + sigma[3] + (55 * sqrtf(2.0f) * cosf(sigma[2])) / 2) / sigma[0];
	Jacobian[1][1] = (55 * cosf(leg->theta_back) * sigma[0] + sigma[3] + (55 * sqrtf(2.0f) * cosf(sigma[1])) / 2) / sigma[0];
}
void vmc_Leg_MotorCtrl(Leg *leg) // 根据当前状态误差计算足端虚拟力，并将虚拟力转换为期望扭矩
{
	float err_x, err_y, err_vx, err_vy;
	float Jacobian[2][2];
	float Jacobian_trans[2][2];
	float angle_Vel[2];
	float Torque[2];
	float torque_fore;
	float torque_back;
	uint8_t force_sat;
	uint8_t torque_sat;
	uint8_t leg_idx;
	matrix_t F, J, J_trans, T, angleVel, f_Vel;
	leg_idx = (leg->id >= 1 && leg->id <= 4) ? (uint8_t)(leg->id - 1) : 0;
	// 收发一次数据更新电机状态
	GO_TorqueMode_Ctrl(&(leg->motor_ctrl_linkf), 1);
	GO_TorqueMode_Ctrl(&(leg->motor_ctrl_linkb), 1);
	// 计算雅可比矩阵
	leg_Calc_Jacobian(leg, Jacobian);
	matrix_wrap(&J, 2, 2, &Jacobian[0][0]);
	matrix_wrap(&J_trans, 2, 2, &Jacobian_trans[0][0]);
	matrix_wrap(&T, 2, 1, Torque);
	matrix_transpose(&J, &J_trans);
	// 计算足端坐标
	leg_Forward_Kinematics(leg);
	// 通过雅可比矩阵计算当前足端速度
	angle_Vel[0] = leg->motor_ctrl_linkf.data.W;
	angle_Vel[1] = leg->motor_ctrl_linkb.data.W;
	matrix_wrap(&angleVel, 2, 1, angle_Vel);
	matrix_wrap(&f_Vel, 2, 1, leg->fvel);
	matrix_mul(&J, &angleVel, &f_Vel);
	// 通过位置，速度误差计算足端虚拟力
	err_x = leg->bezier.pos.x - leg->x;
	err_y = leg->bezier.pos.y - leg->y;
	err_vx = leg->bezier.exp_fvel[0] - leg->fvel[0];
	err_vy = leg->bezier.exp_fvel[1] - leg->fvel[1];
	leg->F[0] =leg->bezier.pid.K_P * err_x + leg->bezier.pid.K_W * err_vx;
	leg->F[1] =leg->bezier.pid.K_P * err_y + leg->bezier.pid.K_W * err_vy;
	force_sat = 0;
	leg->F[0] = vmc_clamp(leg->F[0], VMC_FOOT_FORCE_LIMIT, &force_sat);
	leg->F[1] = vmc_clamp(leg->F[1], VMC_FOOT_FORCE_LIMIT, &force_sat);
	if (force_sat)
	{
		g_vmc_force_sat_count[leg_idx]++;
	}
	matrix_wrap(&F, 2, 1, leg->F);
	matrix_mul(&J_trans, &F, &T);
	torque_sat = 0;
	if (leg->id == 1 || leg->id == 3)
	{
		torque_fore = T.data[0];
		torque_back = -T.data[1];
	}
	else if (leg->id == 2 || leg->id == 4)
	{
		torque_fore = -T.data[0];
		torque_back = T.data[1];
	}
	else
	{
		torque_fore = 0.0f;
		torque_back = 0.0f;
	}
	leg->motor_ctrl_linkf.pid.T = vmc_clamp(torque_fore, VMC_TORQUE_LIMIT, &torque_sat);
	leg->motor_ctrl_linkb.pid.T = vmc_clamp(torque_back, VMC_TORQUE_LIMIT, &torque_sat);
	if (torque_sat)
	{
		g_vmc_torque_sat_count[leg_idx]++;
	}
	// 将更新后的电机扭矩发送给电机
	GO_TorqueMode_Ctrl(&(leg->motor_ctrl_linkf), 1);
	GO_TorqueMode_Ctrl(&(leg->motor_ctrl_linkb), 1);
}
// void leg_Bezier_Act_init(Leg *leg,leg_state state){//初始化bezier参数，开启新曲线的计算
//	leg->state=state;//设置腿运动状态曲线
////	leg->bezier.			= &node_prt[leg->state];		//结点坐标
//	leg->bezier.n				= bezierDimension[leg->state];	//阶数
//	leg->bezier.fre				= bezierFre[leg->state];		//取样频率
//	leg->bezier.T				= bezierT[leg->state];			//动作周期
//	leg->bezier.flag			= 0;							//一条曲线计算完成标志
//	leg->bezier.t				= 0;							//曲线比例系数
//	leg->bezier.point_sum		= 0;
//	leg->bezier.now_time		= 0;
//	leg->bezier.last_end_time	= 0;
//}
void leg_Bezier_Free_Init(Leg *leg, leg_state state, bezier_exp *bezier_para)
{						// 自由配置贝塞尔曲线参数
	leg->state = state; // 设置腿运动状态曲线
	memcpy(&leg->bezier, bezier_para, sizeof(leg->bezier));
}
void leg_BezierTargetPos_Update(Leg *leg)
{
	leg->bezier.now_time = osKernelSysTick();
	if (leg->bezier.point_sum < leg->bezier.T * leg->bezier.fre && (leg->bezier.now_time >= (leg->bezier.last_end_time + (uint32_t)(1000 / leg->bezier.fre))))
	{
		leg->bezier.pos = bezierCurve((const bezierPoint (*)[4])&leg->bezier.ctrl_point, leg->bezier.n, leg->bezier.t); // 更新pos
		leg->bezier.t += 1 / (leg->bezier.T * leg->bezier.fre);								  // 更新比例系数
		if (leg->bezier.t > 1)
		{ // 限定比例系数范围
			leg->bezier.t = 1;
		}
		leg->bezier.point_sum++; // 对已算出点计数
	}
}
void leg_Bezier_Act(Leg *leg)
{ // bezier曲线动作
	leg_BezierTargetPos_Update(leg);
	pos_Leg_MotorCtrl(leg); //
	if (leg->bezier.point_sum >= leg->bezier.T * leg->bezier.fre)
	{
		leg->bezier.flag = 1;
	}
	leg->bezier.last_end_time = osKernelSysTick();
}

void vmc_leg_Bezier_Act(Leg *leg)// 虚拟模型控制器，更新目标位置，根据当前状态误差计算足端虚拟力，并将虚拟力转换为期望扭矩
{ 
	leg_BezierTargetPos_Update(leg);
	vmc_Leg_MotorCtrl(leg);
	if (leg->bezier.point_sum >= leg->bezier.T * leg->bezier.fre)
	{
		leg->bezier.flag = 1;
	}
	leg->bezier.last_end_time = osKernelSysTick();
}
void standUP(Dog *dog);
void walkForward(Dog *dog);
void Turn(Dog *dog);
void LowWalkForward(Dog *dog);
void JumpForward(Dog *dog);
void InJump(Dog *dog);
void WalkBack(Dog *dog);
void Damping_mode(Dog *dog);
void Jump_TurnRight(Dog *dog);

void dogTaskCtrl(Dog *dog)
{
	if (dog->dog_mode == RC_MODE || dog->dog_mode == AUTO_OFFROAD && dog->auto_ctrl.task_attr.task_type == TRACK)
	{
		switch (dog->state)
		{
		case STAND_UP_:
			standUP(dog);
			break;
		case WALK_FORWARD:
			walkForward(dog);
			break;
		case TURN_RIGHT:
			Turn(dog);
			break;
		case TURN_LEFT:
			Turn(dog);
			break;
		case LOWWALK_FORWARD:
			for (uint8_t i = 0; i <= 3; i++)
			{
				dog->leg[i].bezier.pos.x = 0;
				dog->leg[i].bezier.pos.y = 160.0f;
			}
			for (uint8_t i = 0; i <= 3; i++)
			{
				leg_Inverse_Kinematics(&dog->leg[i]);
			}
			osDelay(3000);
			LowWalkForward(dog);
			break;
		case JUMP_FORWARD:
			JumpForward(dog);
			break;
		case INJUMP:
			InJump(dog);
			break;
		case WALK_BACK:
			WalkBack(dog);
			break;
		case DAMPING_MODE:
			Damping_mode(dog);
			break;
		default:
			standUP(dog);
			break;
		}
	}
	else if (dog->dog_mode == AUTO_OFFROAD && dog->auto_ctrl.task_attr.task_type == ACTION)
	{
	}
}
void standUP(Dog *dog)
{
	static bezierPoint ctrl_point[4] = {{0, 190.53}, {0, 0}, {0, 0}, {0, 0}};
	static bezier_exp bezier_exp[1] = {
		{.T = 1.0f,
		 .fre = 1.0f,
		 .n = 0,
		 .pid = {.K_P = 0.3f, .K_W = 0.05f, .Pos = 0, .W = 0, .T = 0},
		 .t = 0.0f,
		 .point_sum = 0,
		 .flag = 0,
		 .ctrl_point = {{0}}, // 初始化所有控制点为 {0,0}
		 .pos = {0.0f, 0.0f},
		 .now_time = 0,
		 .last_end_time = 0}};

	memcpy(&bezier_exp[0].ctrl_point, &ctrl_point, sizeof(bezier_exp[0].ctrl_point));
	leg_Bezier_Free_Init(&dog->leg[0], STAND_UP, &bezier_exp[0]);
	leg_Bezier_Free_Init(&dog->leg[1], STAND_UP, &bezier_exp[0]);
	leg_Bezier_Free_Init(&dog->leg[2], STAND_UP, &bezier_exp[0]);
	leg_Bezier_Free_Init(&dog->leg[3], STAND_UP, &bezier_exp[0]);
	for (uint8_t i = 0; i < 4; i++)
	{
		leg_Bezier_Act(&dog->leg[i]);
	}
}

void walkForward(Dog *dog)
{
	static const bezierPoint ctrl_point[6][4] = {
		{{0, 245.96f}, {25.0f, 150.0f}, {50.0f, 245.96f}, {100, 190.53f}},	// STEP_FORE1
		{{0, 245.96f}, {-25.0f, 270.0f}, {-50.0f, 245.96}, {99, 214}},		// KICK_BACK1
		{{-50.0f, 245.96f}, {-25, 130.0f}, {25, 130.0f}, {50.0f, 245.96f}}, // STEP_FORE2
		{{50.0f, 245.96f}, {0, 290.0f}, {-50.0f, 245.96f}, {99, 214}},		// KICK_BACK2

		{{50.0f, 245.96f}, {25.0f, 270.0f}, {0, 245.96f}, {0, 0}},	// STEP_FORE_REBACK
		{{-50.0f, 245.96f}, {-25.0f, 150.0f}, {0, 245.96f}, {0, 0}} // KICK_BACK_REBACK
	};
	// 初始化所有控制点为 {0,0}.pos = {0.0f, 0.0f},.now_time = 0,.last_end_time = 0
	static bezier_exp bezier_exp[6] = {
		{.T = 0.14f, .fre = 100.0f, .n = 2, .pid = {.K_P = 1.0f, .K_W = 0.1f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.14f, .fre = 100.0f, .n = 2, .pid = {.K_P = 1.0f, .K_W = 0.1f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.14f, .fre = 100.0f, .n = 3, .pid = {.K_P = 1.0f, .K_W = 0.1f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.14f, .fre = 100.0f, .n = 2, .pid = {.K_P = 1.0f, .K_W = 0.1f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},

		{.T = 0.14f, .fre = 100.0f, .n = 2, .pid = {.K_P = 1.0f, .K_W = 0.1f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.14f, .fre = 100.0f, .n = 2, .pid = {.K_P = 1.0f, .K_W = 0.1f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
	};
	for (uint8_t i = 0; i < 6; i++)
	{
		memcpy(&bezier_exp[i].ctrl_point, &ctrl_point[i], sizeof(bezier_exp[i].ctrl_point));
	}
	// 轮流初始化
	leg_Bezier_Free_Init(&dog->leg[0], STEP_FORE1, &bezier_exp[STEP_FORE1]);
	leg_Bezier_Free_Init(&dog->leg[2], STEP_FORE1, &bezier_exp[STEP_FORE1]);
	leg_Bezier_Free_Init(&dog->leg[1], KICK_BACK1, &bezier_exp[KICK_BACK1]);
	leg_Bezier_Free_Init(&dog->leg[3], KICK_BACK1, &bezier_exp[KICK_BACK1]);
	while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
	{ // 将当前轨迹走完
		for (uint8_t i = 0; i <= 3; i++)
		{
			leg_Bezier_Act(&dog->leg[i]);
		}
	}
	osDelay(1);
	//
	while (dog->state == WALK_FORWARD)
	{
		leg_Bezier_Free_Init(&dog->leg[0], KICK_BACK2, &bezier_exp[KICK_BACK2]);
		leg_Bezier_Free_Init(&dog->leg[2], KICK_BACK2, &bezier_exp[KICK_BACK2]);
		leg_Bezier_Free_Init(&dog->leg[1], STEP_FORE2, &bezier_exp[STEP_FORE2]);
		leg_Bezier_Free_Init(&dog->leg[3], STEP_FORE2, &bezier_exp[STEP_FORE2]);
		while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
		{
			for (uint8_t i = 0; i <= 3; i++)
			{
				leg_Bezier_Act(&dog->leg[i]);
			}
		}
		leg_Bezier_Free_Init(&dog->leg[0], STEP_FORE2, &bezier_exp[STEP_FORE2]);
		leg_Bezier_Free_Init(&dog->leg[2], STEP_FORE2, &bezier_exp[STEP_FORE2]);
		leg_Bezier_Free_Init(&dog->leg[1], KICK_BACK2, &bezier_exp[KICK_BACK2]);
		leg_Bezier_Free_Init(&dog->leg[3], KICK_BACK2, &bezier_exp[KICK_BACK2]);
		while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
		{
			for (uint8_t i = 0; i <= 3; i++)
			{
				leg_Bezier_Act(&dog->leg[i]);
			}
		}
		osDelay(1);
	}
	leg_Bezier_Free_Init(&dog->leg[0], STEP_FORE_REBACK, &bezier_exp[STEP_FORE_REBACK]);
	leg_Bezier_Free_Init(&dog->leg[2], STEP_FORE_REBACK, &bezier_exp[STEP_FORE_REBACK]);
	leg_Bezier_Free_Init(&dog->leg[1], KICK_BACK_REBACK, &bezier_exp[KICK_BACK_REBACK]);
	leg_Bezier_Free_Init(&dog->leg[3], KICK_BACK_REBACK, &bezier_exp[KICK_BACK_REBACK]);
}
void Turn(Dog *dog)
{ // 动作分为两个阶段，起步阶段和行进阶段
	static const bezierPoint ctrl_point[12][4] = {
		{{0, 190.53f}, {-25.0f, 100.0f}, {-25.0f, 190.53f}, {-100, 190.53f}},	  // STEP_B1
		{{0, 190.53f}, {25, 100.0f}, {50, 190.53f}, {100, 190.53f}},			  // STEP_F1
		{{0, 190.53}, {25.0f, 200.0f}, {50.0f, 190.53}, {99, 214}},				  // KICK_F1
		{{0, 190.53}, {-25, 200.0f}, {-50, 190.53}, {99, 214}},					  // KICK_B1
		{{50.0f, 190.53f}, {50.0f, 100.0f}, {-50.0f, 100.0f}, {-50.0f, 190.53f}}, // STEP_B2
		{{-50, 190.53f}, {-50, 100.0f}, {50, 100.0f}, {50, 190.53f}},			  // STEP_F2
		{{-50.0f, 190.53}, {0, 200.0f}, {50.0f, 190.53f}, {99, 214}},			  // KICK_F2
		{{50, 190.53}, {0, 200.0f}, {-50, 190.53f}, {99, 214}},					  // KICK_B2

		{{50.0f, 190.53f}, {25.0f, 200.0f}, {0, 190.53f}, {0, 0}},	 // STEP_F_REBACK
		{{-50.0f, 190.53f}, {-25.0f, 200.0f}, {0, 190.53f}, {0, 0}}, // STEP_B_REBACK
		{{50.0f, 190.53f}, {25.0f, 100.0f}, {0, 190.53f}, {0, 0}},	 // KICK_F_REBACK
		{{-50.0f, 190.53f}, {-25.0f, 100.0f}, {0, 190.53f}, {0, 0}}	 // KICK_B_REBACK
	};
	// 初始化所有控制点为 {0,0}.pos = {0.0f, 0.0f},.now_time = 0,.last_end_time = 0
	static bezier_exp bezier_exp[12] = {
		{.T = 0.1f, .fre = 100.0f, .n = 2, .pid = {.K_P = 0.3f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.1f, .fre = 100.0f, .n = 2, .pid = {.K_P = 0.3f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.1f, .fre = 100.0f, .n = 2, .pid = {.K_P = 0.3f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.1f, .fre = 100.0f, .n = 2, .pid = {.K_P = 0.3f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.1f, .fre = 100.0f, .n = 3, .pid = {.K_P = 0.3f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.1f, .fre = 100.0f, .n = 3, .pid = {.K_P = 0.3f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.1f, .fre = 100.0f, .n = 2, .pid = {.K_P = 0.3f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},

		{.T = 0.1f, .fre = 100.0f, .n = 2, .pid = {.K_P = 0.3f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.1f, .fre = 100.0f, .n = 2, .pid = {.K_P = 0.3f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.1f, .fre = 100.0f, .n = 2, .pid = {.K_P = 0.3f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.1f, .fre = 100.0f, .n = 2, .pid = {.K_P = 0.3f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0}

	};
	for (uint8_t i = 0; i < 12; i++)
	{ // 重置控制点
		memcpy(&bezier_exp[i].ctrl_point, &ctrl_point[i], sizeof(bezier_exp[i].ctrl_point));
	}

	if (dog->state == TURN_RIGHT)
	{
		// 轮流初始化
		leg_Bezier_Free_Init(&dog->leg[1], STEP_F1, &bezier_exp[STEP_F1]);
		leg_Bezier_Free_Init(&dog->leg[3], STEP_B1, &bezier_exp[STEP_B1]);
		leg_Bezier_Free_Init(&dog->leg[0], KICK_F1, &bezier_exp[KICK_F1]);
		leg_Bezier_Free_Init(&dog->leg[2], KICK_B1, &bezier_exp[KICK_B1]);
		while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
		{ // 将当前轨迹走完
			for (uint8_t i = 0; i <= 3; i++)
			{
				leg_Bezier_Act(&dog->leg[i]);
			}
		}
		osDelay(1);
		while (dog->state == TURN_RIGHT)
		{
			leg_Bezier_Free_Init(&dog->leg[1], KICK_B2, &bezier_exp[KICK_B2]);
			leg_Bezier_Free_Init(&dog->leg[3], KICK_F2, &bezier_exp[KICK_F2]);
			leg_Bezier_Free_Init(&dog->leg[0], STEP_B2, &bezier_exp[STEP_B2]);
			leg_Bezier_Free_Init(&dog->leg[2], STEP_F2, &bezier_exp[STEP_F2]);
			while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 ||
				   dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
			{
				for (uint8_t i = 0; i <= 3; i++)
				{
					leg_Bezier_Act(&dog->leg[i]);
				}
			}
			leg_Bezier_Free_Init(&dog->leg[1], STEP_F2, &bezier_exp[STEP_F2]);
			leg_Bezier_Free_Init(&dog->leg[3], STEP_B2, &bezier_exp[STEP_B2]);
			leg_Bezier_Free_Init(&dog->leg[0], KICK_F2, &bezier_exp[KICK_F2]);
			leg_Bezier_Free_Init(&dog->leg[2], KICK_B2, &bezier_exp[KICK_B2]);
			while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 ||
				   dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
			{
				for (uint8_t i = 0; i <= 3; i++)
				{
					leg_Bezier_Act(&dog->leg[i]);
				}
			}
			osDelay(1);
		}
		leg_Bezier_Free_Init(&dog->leg[1], STEP_F_REBACK, &bezier_exp[STEP_F_REBACK]);
		leg_Bezier_Free_Init(&dog->leg[3], STEP_B_REBACK, &bezier_exp[STEP_B_REBACK]);
		leg_Bezier_Free_Init(&dog->leg[0], KICK_F_REBACK, &bezier_exp[KICK_F_REBACK]);
		leg_Bezier_Free_Init(&dog->leg[2], KICK_B_REBACK, &bezier_exp[KICK_B_REBACK]);
	}
	else if (dog->state == TURN_LEFT)
	{
		// 轮流初始化
		leg_Bezier_Free_Init(&dog->leg[1], STEP_B1, &bezier_exp[STEP_B1]);
		leg_Bezier_Free_Init(&dog->leg[3], STEP_F1, &bezier_exp[STEP_F1]);
		leg_Bezier_Free_Init(&dog->leg[0], KICK_B1, &bezier_exp[KICK_B1]);
		leg_Bezier_Free_Init(&dog->leg[2], KICK_F1, &bezier_exp[KICK_F1]);
		while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
		{ // 将当前轨迹走完
			for (uint8_t i = 0; i <= 3; i++)
			{
				leg_Bezier_Act(&dog->leg[i]);
			}
		}
		osDelay(1);
		while (dog->state == TURN_LEFT)
		{
			leg_Bezier_Free_Init(&dog->leg[1], KICK_F2, &bezier_exp[KICK_F2]);
			leg_Bezier_Free_Init(&dog->leg[3], KICK_B2, &bezier_exp[KICK_B2]);
			leg_Bezier_Free_Init(&dog->leg[0], STEP_F2, &bezier_exp[STEP_F2]);
			leg_Bezier_Free_Init(&dog->leg[2], STEP_B2, &bezier_exp[STEP_B2]);
			while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 ||
				   dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
			{
				for (uint8_t i = 0; i <= 3; i++)
				{
					leg_Bezier_Act(&dog->leg[i]);
				}
			}
			leg_Bezier_Free_Init(&dog->leg[1], STEP_B2, &bezier_exp[STEP_B2]);
			leg_Bezier_Free_Init(&dog->leg[3], STEP_F2, &bezier_exp[STEP_F2]);
			leg_Bezier_Free_Init(&dog->leg[0], KICK_B2, &bezier_exp[KICK_B2]);
			leg_Bezier_Free_Init(&dog->leg[2], KICK_F2, &bezier_exp[KICK_F2]);
			while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 ||
				   dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
			{
				for (uint8_t i = 0; i <= 3; i++)
				{
					leg_Bezier_Act(&dog->leg[i]);
				}
			}
			osDelay(1);
		}
		leg_Bezier_Free_Init(&dog->leg[1], STEP_B_REBACK, &bezier_exp[STEP_B_REBACK]);
		leg_Bezier_Free_Init(&dog->leg[3], STEP_F_REBACK, &bezier_exp[STEP_F_REBACK]);
		leg_Bezier_Free_Init(&dog->leg[0], KICK_B_REBACK, &bezier_exp[KICK_B_REBACK]);
		leg_Bezier_Free_Init(&dog->leg[2], KICK_F_REBACK, &bezier_exp[KICK_F_REBACK]);
	}
}
void LowWalkForward(Dog *dog)
{
	static const bezierPoint ctrl_point[4][4] = {
		{{0, 180.53f}, {12.5, 100.0f}, {25, 180.53f}, {100, 180.53f}}, // LOWSTEP_FORE1
		{{0, 180.53}, {-12.5, 200.0f}, {-25, 180.53}, {99, 214}},	   // LOWKICK_BACK1
		{{-25, 180.53f}, {-30, 100.0f}, {30, 100.0f}, {25, 180.53f}},  // LOWSTEP_FORE2
		{{25, 180.53}, {0, 200.0f}, {-25, 180.53f}, {99, 214}},		   // LOWKICK_BACK2};
	};
	// 初始化所有控制点为 {0,0}.pos = {0.0f, 0.0f},.now_time = 0,.last_end_time = 0
	static bezier_exp bezier_exp[4] = {
		{.T = 0.3f, .fre = 50.0f, .n = 2, .pid = {.K_P = 0.6f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.3f, .fre = 50.0f, .n = 2, .pid = {.K_P = 0.6f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.3f, .fre = 50.0f, .n = 3, .pid = {.K_P = 0.6f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.3f, .fre = 50.0f, .n = 2, .pid = {.K_P = 0.6f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0}};
	for (uint8_t i = 0; i < 4; i++)
	{ // 重置控制点
		memcpy(&bezier_exp[i].ctrl_point, &ctrl_point[i], sizeof(bezier_exp[i].ctrl_point));
	}
	// Individual initialization with state parameters
	leg_Bezier_Free_Init(&dog->leg[0], LOWSTEP_FORE1, &bezier_exp[LOWSTEP_FORE1]);
	leg_Bezier_Free_Init(&dog->leg[1], LOWKICK_BACK1, &bezier_exp[LOWKICK_BACK1]);
	leg_Bezier_Free_Init(&dog->leg[2], LOWSTEP_FORE1, &bezier_exp[LOWSTEP_FORE1]);
	leg_Bezier_Free_Init(&dog->leg[3], LOWKICK_BACK1, &bezier_exp[LOWKICK_BACK1]);

	while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
	{
		for (uint8_t i = 0; i <= 3; i++)
		{
			leg_Bezier_Act(&dog->leg[i]);
		}
	}
	osDelay(1);

	while (dog->state == LOWWALK_FORWARD)
	{
		// Individual initialization with state parameters
		leg_Bezier_Free_Init(&dog->leg[0], LOWKICK_BACK2, &bezier_exp[LOWKICK_BACK2]);
		leg_Bezier_Free_Init(&dog->leg[1], LOWSTEP_FORE2, &bezier_exp[LOWSTEP_FORE2]);
		leg_Bezier_Free_Init(&dog->leg[2], LOWKICK_BACK2, &bezier_exp[LOWKICK_BACK2]);
		leg_Bezier_Free_Init(&dog->leg[3], LOWSTEP_FORE2, &bezier_exp[LOWSTEP_FORE2]);

		while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
		{
			for (uint8_t i = 0; i <= 3; i++)
			{
				leg_Bezier_Act(&dog->leg[i]);
			}
		}

		// Individual initialization with state parameters
		leg_Bezier_Free_Init(&dog->leg[0], LOWSTEP_FORE2, &bezier_exp[LOWSTEP_FORE2]);
		leg_Bezier_Free_Init(&dog->leg[1], LOWKICK_BACK2, &bezier_exp[LOWKICK_BACK2]);
		leg_Bezier_Free_Init(&dog->leg[2], LOWSTEP_FORE2, &bezier_exp[LOWSTEP_FORE2]);
		leg_Bezier_Free_Init(&dog->leg[3], LOWKICK_BACK2, &bezier_exp[LOWKICK_BACK2]);

		while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
		{
			for (uint8_t i = 0; i <= 3; i++)
			{
				leg_Bezier_Act(&dog->leg[i]);
			}
		}
		osDelay(1);
	}
}

void JumpForward(Dog *dog)
{
	static const bezierPoint ctrl_point[4][4] = {
		{{-50, 140}, {-50, 140}, {0, 0}, {-0, 0}},		   // JUMP_PREPARE
		{{-50, 140}, {-114.285f, 320.0f}, {0, 0}, {0, 0}}, // JUMPING
		{{-114.285f, 320.0f}, {75, 210}, {0, 0}, {0, 0}},
		{{75, 210}, {50, 140}, {0, 0}, {0, 0.0}}, // JUMP_BUFFER
	};
	// 初始化所有控制点为 {0,0}.pos = {0.0f, 0.0f},.now_time = 0,.last_end_time = 0
	static bezier_exp bezier_exp[4] = {
		{.T = 1.0f, .fre = 1.0f, .n = 0, .pid = {.K_P = 0.5f, .K_W = 0.05f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.1f, .fre = 50.0f, .n = 1, .pid = {.K_P = 3.0f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.1f, .fre = 50.0f, .n = 1, .pid = {.K_P = 0.2f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
		{.T = 0.2f, .fre = 50.0f, .n = 1, .pid = {.K_P = 0.2f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0}};
	for (uint8_t i = 0; i < 4; i++)
	{ // 重置控制点
		memcpy(&bezier_exp[i].ctrl_point, &ctrl_point[i], sizeof(bezier_exp[i].ctrl_point));
	}
	// Individual initialization with state parameters
	leg_Bezier_Free_Init(&dog->leg[0], JUMP_PREPARE, &bezier_exp[JUMP_PREPARE]);
	leg_Bezier_Free_Init(&dog->leg[1], JUMP_PREPARE, &bezier_exp[JUMP_PREPARE]);
	leg_Bezier_Free_Init(&dog->leg[2], JUMP_PREPARE, &bezier_exp[JUMP_PREPARE]);
	leg_Bezier_Free_Init(&dog->leg[3], JUMP_PREPARE, &bezier_exp[JUMP_PREPARE]);

	while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 ||
		   dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			leg_Bezier_Act(&dog->leg[i]);
		}
	}
	osDelay(1000);
	// Individual initialization with state parameters
	leg_Bezier_Free_Init(&dog->leg[0], JUMPING, &bezier_exp[JUMPING]);
	leg_Bezier_Free_Init(&dog->leg[1], JUMPING, &bezier_exp[JUMPING]);
	leg_Bezier_Free_Init(&dog->leg[2], JUMPING, &bezier_exp[JUMPING]);
	leg_Bezier_Free_Init(&dog->leg[3], JUMPING, &bezier_exp[JUMPING]);

	while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 ||
		   dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			leg_Bezier_Act(&dog->leg[i]);
		}
	}
	//     osDelay(1000);
	osDelay(200);
	// Individual initialization with state parameters
	leg_Bezier_Free_Init(&dog->leg[0], JUMP_LANDING, &bezier_exp[JUMP_LANDING]);
	leg_Bezier_Free_Init(&dog->leg[1], JUMP_LANDING, &bezier_exp[JUMP_LANDING]);
	leg_Bezier_Free_Init(&dog->leg[2], JUMP_LANDING, &bezier_exp[JUMP_LANDING]);
	leg_Bezier_Free_Init(&dog->leg[3], JUMP_LANDING, &bezier_exp[JUMP_LANDING]);

	while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 ||
		   dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			leg_Bezier_Act(&dog->leg[i]);
		}
	}
	osDelay(100);
	// Individual initialization with state parameters
	leg_Bezier_Free_Init(&dog->leg[0], JUMP_BUFFER, &bezier_exp[JUMP_BUFFER]);
	leg_Bezier_Free_Init(&dog->leg[1], JUMP_BUFFER, &bezier_exp[JUMP_BUFFER]);
	leg_Bezier_Free_Init(&dog->leg[2], JUMP_BUFFER, &bezier_exp[JUMP_BUFFER]);
	leg_Bezier_Free_Init(&dog->leg[3], JUMP_BUFFER, &bezier_exp[JUMP_BUFFER]);

	while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 ||
		   dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			leg_Bezier_Act(&dog->leg[i]);
		}
	}
	osDelay(1000);
}

void InJump(Dog *dog)
{
	static const bezierPoint ctrl_point[3][4] = {
		{{0, 130}, {0, 0}, {0, 0}, {-0, 0}},			// INJUMP_PREPARE
		{{0, 320}, {0, 320}, {35, 279.79f}, {70, 100}}, // INJUMP_TAKEOFF
		{{0, 320}, {0, 130}, {0, 100.0}, {0, 0.0}}		// INJUMP_LANDING
	};
	// 初始化所有控制点为 {0,0}.pos = {0.0f, 0.0f},.now_time = 0,.last_end_time = 0
	static bezier_exp bezier_exp[3] = {//
									   {.T = 1.0f, .fre = 1.0f, .n = 0, .pid = {.K_P = 0.2f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
									   {.T = 1.0f, .fre = 1.0f, .n = 0, .pid = {.K_P = 3.0f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
									   {.T = 0.2f, .fre = 50.0f, .n = 1, .pid = {.K_P = 0.2f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0}};
	for (uint8_t i = 0; i < 3; i++)
	{ // 重置控制点
		memcpy(&bezier_exp[i].ctrl_point, &ctrl_point[i], sizeof(bezier_exp[i].ctrl_point));
	}
	// Individual initialization with state parameters
	leg_Bezier_Free_Init(&dog->leg[0], INJUMP_PREPARE, &bezier_exp[INJUMP_PREPARE]);
	leg_Bezier_Free_Init(&dog->leg[1], INJUMP_PREPARE, &bezier_exp[INJUMP_PREPARE]);
	leg_Bezier_Free_Init(&dog->leg[2], INJUMP_PREPARE, &bezier_exp[INJUMP_PREPARE]);
	leg_Bezier_Free_Init(&dog->leg[3], INJUMP_PREPARE, &bezier_exp[INJUMP_PREPARE]);

	while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 ||
		   dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
	{
		for (uint8_t i = 0; i <= 3; i++)
		{
			leg_Bezier_Act(&dog->leg[i]);
		}
	}

	osDelay(300);
	// Individual initialization with state parameters
	leg_Bezier_Free_Init(&dog->leg[0], INJUMP_TAKEOFF, &bezier_exp[INJUMP_TAKEOFF]);
	leg_Bezier_Free_Init(&dog->leg[1], INJUMP_TAKEOFF, &bezier_exp[INJUMP_TAKEOFF]);
	leg_Bezier_Free_Init(&dog->leg[2], INJUMP_TAKEOFF, &bezier_exp[INJUMP_TAKEOFF]);
	leg_Bezier_Free_Init(&dog->leg[3], INJUMP_TAKEOFF, &bezier_exp[INJUMP_TAKEOFF]);

	while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 ||
		   dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
	{
		for (uint8_t i = 0; i <= 3; i++)
		{
			leg_Bezier_Act(&dog->leg[i]);
		}
	}

	osDelay(500);
	// Individual initialization with state parameters
	leg_Bezier_Free_Init(&dog->leg[0], INJUMP_LANDING, &bezier_exp[INJUMP_LANDING]);
	leg_Bezier_Free_Init(&dog->leg[1], INJUMP_LANDING, &bezier_exp[INJUMP_LANDING]);
	leg_Bezier_Free_Init(&dog->leg[2], INJUMP_LANDING, &bezier_exp[INJUMP_LANDING]);
	leg_Bezier_Free_Init(&dog->leg[3], INJUMP_LANDING, &bezier_exp[INJUMP_LANDING]);
	while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 ||
		   dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
	{
		for (uint8_t i = 0; i <= 3; i++)
		{
			leg_Bezier_Act(&dog->leg[i]);
		}
	}
	osDelay(1000);
}

void WalkBack(Dog *dog)
{
	static const bezierPoint ctrl_point[4][4] = {
		{{0, 190.53f}, {-12.5, 100.0f}, {-25, 190.53f}, {100, 190.53f}}, // STEP_FORE1
		{{0, 190.53}, {12.5, 200.0f}, {25, 190.53}, {99, 214}},			 // KICK_BACK1
		{{25, 190.53f}, {30, 100.0f}, {-30, 100.0f}, {-25, 190.53f}},	 // STEP_FORE2
		{{-25, 190.53}, {0, 200.0f}, {25, 190.53f}, {99, 214}},			 // KICK_BACK2
	};
	// 初始化所有控制点为 {0,0}.pos = {0.0f, 0.0f},.now_time = 0,.last_end_time = 0
	static bezier_exp bezier_exp[4] = {//
									   {.T = 0.3f, .fre = 50.0f, .n = 2, .pid = {.K_P = 0.6f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
									   {.T = 0.3f, .fre = 50.0f, .n = 2, .pid = {.K_P = 0.6f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
									   {.T = 0.3f, .fre = 50.0f, .n = 3, .pid = {.K_P = 0.6f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
									   {.T = 0.3f, .fre = 50.0f, .n = 2, .pid = {.K_P = 0.6f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0}};
	for (uint8_t i = 0; i < 4; i++)
	{ // 重置控制点
		memcpy(&bezier_exp[i].ctrl_point, &ctrl_point[i], sizeof(bezier_exp[i].ctrl_point));
	}
	// Individual initialization with state parameters
	leg_Bezier_Free_Init(&dog->leg[0], STEP_FORE1_BACK, &bezier_exp[STEP_FORE1_BACK]);
	leg_Bezier_Free_Init(&dog->leg[1], KICK_BACK1_BACK, &bezier_exp[KICK_BACK1_BACK]);
	leg_Bezier_Free_Init(&dog->leg[2], STEP_FORE1_BACK, &bezier_exp[STEP_FORE1_BACK]);
	leg_Bezier_Free_Init(&dog->leg[3], KICK_BACK1_BACK, &bezier_exp[KICK_BACK1_BACK]);

	while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
	{
		for (uint8_t i = 0; i <= 3; i++)
		{
			leg_Bezier_Act(&dog->leg[i]);
		}
	}
	osDelay(1);
	while (dog->state == WALK_BACK)
	{
		// Individual initialization with state parameters
		leg_Bezier_Free_Init(&dog->leg[0], KICK_BACK2_BACK, &bezier_exp[KICK_BACK2_BACK]);
		leg_Bezier_Free_Init(&dog->leg[1], STEP_FORE2_BACK, &bezier_exp[STEP_FORE2_BACK]);
		leg_Bezier_Free_Init(&dog->leg[2], KICK_BACK2_BACK, &bezier_exp[KICK_BACK2_BACK]);
		leg_Bezier_Free_Init(&dog->leg[3], STEP_FORE2_BACK, &bezier_exp[STEP_FORE2_BACK]);

		while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
		{
			for (uint8_t i = 0; i <= 3; i++)
			{
				leg_Bezier_Act(&dog->leg[i]);
			}
		}

		// Individual initialization with state parameters
		leg_Bezier_Free_Init(&dog->leg[0], STEP_FORE2_BACK, &bezier_exp[STEP_FORE2_BACK]);
		leg_Bezier_Free_Init(&dog->leg[1], KICK_BACK2_BACK, &bezier_exp[KICK_BACK2_BACK]);
		leg_Bezier_Free_Init(&dog->leg[2], STEP_FORE2_BACK, &bezier_exp[STEP_FORE2_BACK]);
		leg_Bezier_Free_Init(&dog->leg[3], KICK_BACK2_BACK, &bezier_exp[KICK_BACK2_BACK]);

		while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
		{
			for (uint8_t i = 0; i <= 3; i++)
			{
				leg_Bezier_Act(&dog->leg[i]);
			}
		}
		osDelay(1);
	}
}

void Damping_mode(Dog *dog)
{
	static bezier_exp bezier_exp = {.T = 1.0f, .fre = 1.0f, .n = 0, .pid = {.K_P = 0.0f, .K_W = 0.01f, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .ctrl_point = {0}, .pos = {0.0f, 190.53f}, .now_time = 0, .last_end_time = 0};
	for (uint8_t i = 0; i < 4; i++)
	{
		leg_Bezier_Free_Init(&dog->leg[i], DAMPING, &bezier_exp);
	}
	for (uint8_t i = 0; i < 4; i++)
	{
		pos_Leg_MotorCtrl(&dog->leg[i]);
	}
	while (dog->state == DAMPING_MODE)
	{
		osDelay(100);
	}
}

// VMC 模式
void vmc_Stand(Dog *dog)
{
	static bezierPoint ctrl_point[4] = {{0, 190.53}, {0, 0}, {0, 0}, {0, 0}};
	static bezier_exp bezier_exp[1] = {
		{.T = 1.0f,
		 .fre = 1.0f,
		 .n = 0,
		 .pid = {.K_P = 0.3f, .K_W = 0.05f, .Pos = 0, .W = 0, .T = 0},
		 .t = 0.0f,
		 .point_sum = 0,
		 .flag = 0,
		 .ctrl_point = {{0}}, // 初始化所有控制点为 {0,0}
		 .pos = {0.0f, 0.0f},
		 .now_time = 0,
		 .last_end_time = 0}};
	memcpy(&bezier_exp[0].ctrl_point, &ctrl_point, sizeof(bezier_exp[0].ctrl_point));
	leg_Bezier_Free_Init(&dog->leg[0], STAND_UP, &bezier_exp[0]);
	leg_Bezier_Free_Init(&dog->leg[1], STAND_UP, &bezier_exp[0]);
	leg_Bezier_Free_Init(&dog->leg[2], STAND_UP, &bezier_exp[0]);
	leg_Bezier_Free_Init(&dog->leg[3], STAND_UP, &bezier_exp[0]);
	for (uint8_t i = 0; i < 4; i++)
	{
		vmc_leg_Bezier_Act(&dog->leg[i]);
	}
}
