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
#define LINK_INIT1 2.15857f
#define LINK_INIT2 2.78744f
#define VMC_FOOT_FORCE_LIMIT 300.0f // N
#define VMC_TORQUE_LIMIT 0.2f
#define VMC_VEL_LPF_ALPHA 0.25f
#define VMC_POS_ERR_SHAPE_MM 40.0f
#define VMC_VEL_ERR_LIMIT 1200.0f
#define VMC_FORCE_DEADZONE 0.005f
#define VMC_Y_AXIS_STIFFNESS_SCALE 1.0f

volatile uint32_t g_vmc_force_sat_count[4] = {0};
volatile uint32_t g_vmc_torque_sat_count[4] = {0};
static float g_vmc_fvel_lpf[4][2] = {0};
float vofa_data=0;
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

static float vmc_clamp_scalar(float value, float limit)
{
  if (value > limit)
    {
      return limit;
    }
  if (value < -limit)
    {
      return -limit;
    }
  return value;
}

static float vmc_deadzone(float value, float deadzone)
{
  if (fabsf(value) < deadzone)
    {
      return 0.0f;
    }
  return value;
}

static float vmc_shape_pos_err(float err)
{
  return VMC_POS_ERR_SHAPE_MM * tanhf(err / VMC_POS_ERR_SHAPE_MM);
}
// 初始化整机状态、腿部参数和电机通信链路。
void Dog_ParaInit(Dog *dog)
{
  memset(&dog->leg, 0, sizeof(Leg) * 4);
  dog->state = STAND_UP_;
  memset(&dog->location, 0, sizeof(dog_location));	 //
  memset(&dog->auto_ctrl, 0, sizeof(auto_ctrlcenter)); // 初始化自动控制参数
  for (uint8_t i = 0; i < 4; i++)
    {
      dog->leg[i] = (Leg)
      {
        .L1 = 110,
        .L2 = 220,
        .id = i + 1,
        .state = STAND_UP,
        .theta_fore = 0,
        .theta_back = 0
      };
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
// 逆运动学：将足端目标位置转换为关节角和电机位置指令。
void leg_Inverse_Kinematics(Leg *leg)
{
  float A = pow(leg->bezier.exp_pos.x, 2) + pow(leg->bezier.exp_pos.y, 2) + pow(leg->L1, 2) - pow(leg->L2, 2);
  float theta_fore = -acos(A / ((2 * leg->L1) * sqrtf(pow(leg->bezier.exp_pos.x, 2) + pow(leg->bezier.exp_pos.y, 2)))) + atan2(leg->bezier.exp_pos.y, leg->bezier.exp_pos.x); // rad连杆位置
  float theta_back = acos(A / ((2 * leg->L1) * sqrtf(pow(leg->bezier.exp_pos.x, 2) + pow(leg->bezier.exp_pos.y, 2)))) + atan2(leg->bezier.exp_pos.y, leg->bezier.exp_pos.x);

  if (leg->id == 1)
    {
      leg->motor_ctrl_linkf.pid.Pos = (theta_fore * 6.33f) + (leg->motor_ctrl_linkf.init_pos + 2.15857f * 6.33f);
      leg->motor_ctrl_linkb.pid.Pos = -(theta_back * 6.33f) + (leg->motor_ctrl_linkb.init_pos + 2.78744f * 6.33f);
    }
  else if (leg->id == 2)
    {
      leg->motor_ctrl_linkf.pid.Pos = -(theta_fore * 6.33f) + (leg->motor_ctrl_linkf.init_pos - 2.15857f * 6.33f);
      leg->motor_ctrl_linkb.pid.Pos = (theta_back * 6.33f) + (leg->motor_ctrl_linkb.init_pos - 2.78744f * 6.33f);
    }
  else if (leg->id == 3)
    {
      leg->motor_ctrl_linkf.pid.Pos = (theta_fore * 6.33f) + (leg->motor_ctrl_linkf.init_pos - 0.352552f * 6.33f);
      leg->motor_ctrl_linkb.pid.Pos = -(theta_back * 6.33f) + (leg->motor_ctrl_linkb.init_pos + 5.298575f * 6.33f);
    }
  else if (leg->id == 4)
    {
      leg->motor_ctrl_linkf.pid.Pos = -(theta_fore * 6.33f) + (leg->motor_ctrl_linkf.init_pos + 0.352552f * 6.33f);
      leg->motor_ctrl_linkb.pid.Pos = (theta_back * 6.33f) + (leg->motor_ctrl_linkb.init_pos - 5.298575f * 6.33f);
      //		leg->motor_ctrl_linkb.pid.Pos = (leg->motor_ctrl_linkb .init_pos);
    }
}
// 正运动学：由关节角估计当前足端位置。
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
      leg->theta_fore = Alpha2 + a;
      leg->theta_back = (2.0f*pi-Alpha1) - b;
    }
  else if (leg->id == 4)
    {
      leg->theta_fore = Alpha2 - a;
      leg->theta_back = (2.0f*pi-Alpha1) + b;
    }
  deta = 0.5f * (leg->theta_back - leg->theta_fore);
  phi = 0.5f * (leg->theta_fore + leg->theta_back);
  leg->x = cosf(phi) * (110 * cosf(-deta) + 55 * sqrtf(2.0f) * sqrtf(cosf(-2 * deta) + 7.0f));
  leg->y = sinf(phi) * (110 * cosf(-deta) + 55 * sqrtf(2.0f) * sqrtf(cosf(-2 * deta) + 7.0f));
  leg->phi = phi;
}
// 位置模式单腿控制：按当前贝塞尔目标发送位置指令�?
void posCtrlSolve(Leg *leg)
{
  leg_Inverse_Kinematics(leg);

  // 前肢
  leg->motor_ctrl_linkf.pid.K_P = leg->bezier.pid[0].K_P; // K_p
  leg->motor_ctrl_linkf.pid.K_W = leg->bezier.pid[0].K_W; // K_w
  leg->motor_ctrl_linkf.pid.W = leg->bezier.pid[0].W;		// W

  // 后肢
  leg->motor_ctrl_linkb.pid.K_P = leg->bezier.pid[0].K_P; // K_p
  leg->motor_ctrl_linkb.pid.K_W = leg->bezier.pid[0].K_W; // K_w
  leg->motor_ctrl_linkb.pid.W = leg->bezier.pid[0].W;		// W
}
// 构建雅可比矩阵，用于速度/力映射�?
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
// VMC主循环：状态估�?-> 虚拟力计�?-> 关节扭矩映射�?
void vmc_Solve(Leg *leg)
{
  // 根据当前状态误差计算足端虚拟力，并将虚拟力转换为期望扭�?{
  float err_x, err_y, err_vx, err_vy;
  float Jacobian[2][2];
  float Jacobian_trans[2][2];
  float angle_Vel[2];
  float Torque[2];
  float torque_fore; // N*m
  float torque_back;
  uint8_t force_sat;
  uint8_t torque_sat;
  uint8_t leg_idx;
  matrix_t Fxy, J, J_trans, T, angleVel, f_Vel;
  leg_idx = (leg->id >= 1 && leg->id <= 4) ? (uint8_t)(leg->id - 1) : 0;
  // 更新关节转角，正解算计算足端坐标
  leg_Forward_Kinematics(leg);
  // 计算雅可比矩阵
  leg_Calc_Jacobian(leg, Jacobian);
  matrix_wrap(&J, 2, 2, &Jacobian[0][0]);
  matrix_wrap(&J_trans, 2, 2, &Jacobian_trans[0][0]);
  matrix_wrap(&T, 2, 1, Torque);
  matrix_transpose(&J, &J_trans);
  // 通过雅可比矩阵计算当前足端速度
  angle_Vel[0] = leg->motor_ctrl_linkf.data.W / 6.33f; // 连杆转速rad/s
  angle_Vel[1] = leg->motor_ctrl_linkb.data.W / 6.33f; // rad/s
  matrix_wrap(&angleVel, 2, 1, angle_Vel);
  matrix_wrap(&f_Vel, 2, 1, leg->fvel);
  matrix_mul(&J, &angleVel, &f_Vel); // mm/s
  // 通过位置，速度误差计算足端虚拟力
  err_x = leg->bezier.exp_pos.x - leg->x; // mm
  err_y = leg->bezier.exp_pos.y - leg->y; // mm
  // Filter measured foot velocity to reduce derivative noise in force control.
  g_vmc_fvel_lpf[leg_idx][0] += VMC_VEL_LPF_ALPHA * (leg->fvel[0] - g_vmc_fvel_lpf[leg_idx][0]);
  g_vmc_fvel_lpf[leg_idx][1] += VMC_VEL_LPF_ALPHA * (leg->fvel[1] - g_vmc_fvel_lpf[leg_idx][1]);
  err_vx = leg->bezier.exp_fvel[0] - g_vmc_fvel_lpf[leg_idx][0];
  err_vy = leg->bezier.exp_fvel[1] - g_vmc_fvel_lpf[leg_idx][1];
  err_x = vmc_shape_pos_err(err_x);
  err_y = vmc_shape_pos_err(err_y);
  err_vx = vmc_clamp_scalar(err_vx, VMC_VEL_ERR_LIMIT);
  err_vy = vmc_clamp_scalar(err_vy, VMC_VEL_ERR_LIMIT);
  // 计算虚拟力以及限�?
  leg->bezier.pid[1].xyPosIntegral[0] += err_x * leg->bezier.pid[1].K_I; // 积分�?
  leg->bezier.pid[1].xyPosIntegral[1] += err_y * leg->bezier.pid[1].K_I;
  leg->Fxy[0] = leg->bezier.pid[1].K_P * err_x + leg->bezier.pid[1].K_W * err_vx + leg->bezier.pid[1].xyPosIntegral[0];
  leg->Fxy[1] = (leg->bezier.pid[1].K_P * VMC_Y_AXIS_STIFFNESS_SCALE) * err_y + leg->bezier.pid[1].K_W * err_vy + leg->bezier.pid[1].xyPosIntegral[1]; // N
  leg->Fxy[0] = vmc_deadzone(leg->Fxy[0], VMC_FORCE_DEADZONE);																						 // 死区处理
  leg->Fxy[1] = vmc_deadzone(leg->Fxy[1], VMC_FORCE_DEADZONE);
  force_sat = 0;
  leg->Fxy[0] = vmc_clamp(leg->Fxy[0], VMC_FOOT_FORCE_LIMIT, &force_sat); // 限幅处理
  leg->Fxy[1] = vmc_clamp(leg->Fxy[1], VMC_FOOT_FORCE_LIMIT, &force_sat);
  if (force_sat)
    {
      g_vmc_force_sat_count[leg_idx]++;
    }
  matrix_wrap(&Fxy, 2, 1, leg->Fxy);
  matrix_mul(&J_trans, &Fxy, &T); // 足端虚拟力转扭矩N*mm
  torque_sat = 0;
  if (leg->id == 1 || leg->id == 3)
    {
      torque_fore = T.data[0] / (6.33f * 1000.0f);
      torque_back = -T.data[1] / (6.33f * 1000.0f);
    }
  else if (leg->id == 2 || leg->id == 4)
    {
      torque_fore = -T.data[0] / (6.33f * 1000.0f);
      torque_back = T.data[1] / (6.33f * 1000.0f);
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
}
// void leg_Bezier_Act_init(Leg *leg,leg_state state){//初始化bezier参数，开启新曲线的计�?//	leg->state=state;//设置腿运动状态曲�?////	leg->bezier.			= &node_prt[leg->state];		//结点坐标
//	leg->bezier.config.n				= bezierDimension[leg->state];	//阶数
//	leg->bezier.config.fre				= bezierFre[leg->state];		//取样频率
//	leg->bezier.config.T				= bezierT[leg->state];			//动作周期
//	leg->bezier.flag			= 0;							//一条曲线计算完成标�?//	leg->bezier.t				= 0;							//曲线比例系数
//	leg->bezier.point_sum		= 0;
//	leg->bezier.config.now_time		= 0;
//	leg->bezier.last_end_time	= 0;
//}
// 为当前腿状态装载贝塞尔轨迹参数�?
void leg_Bezier_Free_Init(Leg *leg, leg_state state, bezier_exp *bezier_para)
{
  leg->state = state;
  memcpy(&leg->bezier, bezier_para, sizeof(leg->bezier));
}
// 按动作周期与采样频率更新贝塞尔目标点
void leg_BezierTargetPos_Update(Leg *leg)
{
  leg->bezier.exp_pos = bezierCurve((const bezierPoint(*)[4]) & leg->bezier.config.ctrl_point, leg->bezier.config.n, leg->bezier.t); // 更新pos
  leg->bezier.t += 1 / (leg->bezier.config.T * leg->bezier.config.fre);															   // 更新比例系数
  if (leg->bezier.t > 1)
    {
      // 限定比例系数范围
      leg->bezier.t = 1;
    }
  leg->bezier.point_sum++; // 对已算出点计数
}

// 执行一步位置模式贝塞尔轨迹控制
void leg_Bezier_Act(Leg *leg)
{
  leg_BezierTargetPos_Update(leg);
  posCtrlSolve(leg);
  // 启动电机
  GO_PosMode_Ctrl(&(leg->motor_ctrl_linkf), 1);
  GO_PosMode_Ctrl(&(leg->motor_ctrl_linkb), 1);
  if (leg->bezier.point_sum >= leg->bezier.config.T * leg->bezier.config.fre)
    {
      leg->bezier.flag = 1;
    }
  leg->bezier.last_end_time = osKernelSysTick();
}

// 执行一步VMC扭矩模式贝塞尔轨迹控制
void vmc_leg_Bezier_Act(Leg *leg) // 虚拟模型控制器，更新目标位置，根据当前状态误差计算足端虚拟力，并将虚拟力转换为期望扭矩
{
  leg_BezierTargetPos_Update(leg);
  vmc_Solve(leg);
  // 将更新后的电机扭矩发送给电机
  GO_TorqueMode_Ctrl(&(leg->motor_ctrl_linkf), 0);
  GO_TorqueMode_Ctrl(&(leg->motor_ctrl_linkb), 0);
  if (leg->bezier.point_sum >= leg->bezier.config.T * leg->bezier.config.fre)
    {
      leg->bezier.flag = 1;
    }
  leg->bezier.last_end_time = osKernelSysTick();
}


void fpc_Leg_Bezier_Act(Leg *leg)
{
  leg->bezier.now_time = osKernelSysTick();
  if(leg->bezier.point_sum>0)
    {
      GO_PFC_Ctrl(&(leg->motor_ctrl_linkf), 0);
      GO_PFC_Ctrl(&(leg->motor_ctrl_linkb), 0);
    }
  if (leg->bezier.point_sum < leg->bezier.config.T * leg->bezier.config.fre && (leg->bezier.now_time >= (leg->bezier.last_end_time + (uint32_t)(1000 / leg->bezier.config.fre))))
    {

      leg_BezierTargetPos_Update(leg);
      posCtrlSolve(leg);
      vmc_Solve(leg);
      GO_PFC_Ctrl(&(leg->motor_ctrl_linkf), 0);
      GO_PFC_Ctrl(&(leg->motor_ctrl_linkb), 0);
      if (leg->bezier.point_sum >= leg->bezier.config.T * leg->bezier.config.fre)
        {
          leg->bezier.flag = 1;
        }
      leg->bezier.last_end_time = osKernelSysTick();
    }
  vmc_Solve(leg);
  GO_PFC_Ctrl(&(leg->motor_ctrl_linkf), 0);
  GO_PFC_Ctrl(&(leg->motor_ctrl_linkb), 0);
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
void standUP_FPC(Dog *dog);
// 行为状态机分发函数�?
void dogTaskCtrl(Dog *dog)
{
  if (dog->dog_mode == RC_MODE || (dog->dog_mode == AUTO_OFFROAD && dog->auto_ctrl.task_attr.task_type == TRACK))
    {
      switch (dog->state)
        {
        case STAND_UP_:
          standUP_FPC(dog);
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
              dog->leg[i].bezier.exp_pos.x = 0;
              dog->leg[i].bezier.exp_pos.y = 160.0f;
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
// 四腿站立动作流程�?
void standUP(Dog *dog)
{
  static bezierPoint ctrl_point[4] = {{0, 190.53f}, {0, 0}, {0, 0}, {0, 0}};
  static bezier_exp bezier_exp[1] =
  {
    {
      .config = {.T = 1, .fre = 1, .n = 0, .ctrl_point = {0}},

      .pid = {{.K_P = 0.3f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
      .t = 0.0f,
      .point_sum = 0,
      .flag = 0,
      .exp_pos = {0.0f, 0.0f},
      .now_time = 0,
      .last_end_time = 0
    }
  };

  memcpy(&bezier_exp[0].config.ctrl_point, &ctrl_point, sizeof(bezier_exp[0].config.ctrl_point));
  leg_Bezier_Free_Init(&dog->leg[0], STAND_UP, &bezier_exp[0]);
  leg_Bezier_Free_Init(&dog->leg[1], STAND_UP, &bezier_exp[0]);
  leg_Bezier_Free_Init(&dog->leg[2], STAND_UP, &bezier_exp[0]);
  leg_Bezier_Free_Init(&dog->leg[3], STAND_UP, &bezier_exp[0]);
  for (uint8_t i = 0; i < 4; i++)
    {
      leg_Bezier_Act(&dog->leg[i]);
    }
}

// 前进步态流程�?
void walkForward(Dog *dog)
{
  static const bezierPoint ctrl_point[6][4] =
  {
    {{0, 245.96f}, {25.0f, 150.0f}, {50.0f, 245.96f}, {100, 190.53f}},	// STEP_FORE1
    {{0, 245.96f}, {-25.0f, 270.0f}, {-50.0f, 245.96}, {99, 214}},		// KICK_BACK1
    {{-50.0f, 245.96f}, {-25, 130.0f}, {25, 130.0f}, {50.0f, 245.96f}}, // STEP_FORE2
    {{50.0f, 245.96f}, {0, 290.0f}, {-50.0f, 245.96f}, {99, 214}},		// KICK_BACK2

    {{50.0f, 245.96f}, {25.0f, 270.0f}, {0, 245.96f}, {0, 0}},	// STEP_FORE_REBACK
    {{-50.0f, 245.96f}, {-25.0f, 150.0f}, {0, 245.96f}, {0, 0}} // KICK_BACK_REBACK
  };
  // 初始化所有控制点�?{0,0}.exp_pos = {0.0f, 0.0f},.now_time = 0,.last_end_time = 0
  static bezier_exp bezier_exp[6] =
  {
    {.config.T = 0.14f, .config.fre = 100.0f, .config.n = 2, .pid = {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.14f, .config.fre = 100.0f, .config.n = 2, .pid = {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.14f, .config.fre = 100.0f, .config.n = 3, .pid = {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.14f, .config.fre = 100.0f, .config.n = 2, .pid = {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},

    {.config.T = 0.14f, .config.fre = 100.0f, .config.n = 2, .pid = {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.14f, .config.fre = 100.0f, .config.n = 2, .pid = {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
  };

  for (uint8_t i = 0; i < 6; i++)
    {
      memcpy(&bezier_exp[i].config.ctrl_point, &ctrl_point[i], sizeof(bezier_exp[i].config.ctrl_point));
    }
  // 轮流初始�?	leg_Bezier_Free_Init(&dog->leg[0], STEP_FORE1, &bezier_exp[STEP_FORE1]);
  leg_Bezier_Free_Init(&dog->leg[2], STEP_FORE1, &bezier_exp[STEP_FORE1]);
  leg_Bezier_Free_Init(&dog->leg[1], KICK_BACK1, &bezier_exp[KICK_BACK1]);
  leg_Bezier_Free_Init(&dog->leg[3], KICK_BACK1, &bezier_exp[KICK_BACK1]);
  while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
    {
      // 将当前轨迹走�?
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
// 转向步态流程（左转/右转共用）�?
void Turn(Dog *dog)
{
  // 动作分为两个阶段，起步阶段和行进阶段
  static const bezierPoint ctrl_point[12][4] =
  {
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
  // 初始化所有控制点�?{0,0}.exp_pos = {0.0f, 0.0f},.now_time = 0,.last_end_time = 0
  static bezier_exp bezier_exp[12] =
  {
    {.config.T = 0.1f, .config.fre = 100.0f, .config.n = 2, .pid[0]= {.K_P = 0.3f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.1f, .config.fre = 100.0f, .config.n = 2, .pid[0]= {.K_P = 0.3f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.1f, .config.fre = 100.0f, .config.n = 2, .pid[0]= {.K_P = 0.3f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.1f, .config.fre = 100.0f, .config.n = 2, .pid[0]= {.K_P = 0.3f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.1f, .config.fre = 100.0f, .config.n = 3, .pid[0]= {.K_P = 0.3f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.1f, .config.fre = 100.0f, .config.n = 3, .pid[0]= {.K_P = 0.3f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.1f, .config.fre = 100.0f, .config.n = 2, .pid[0]= {.K_P = 0.3f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.1f, .config.fre = 100.0f, .config.n = 2, .pid[0]= {.K_P = 0.3f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},

    {.config.T = 0.1f, .config.fre = 100.0f, .config.n = 2, .pid[0]= {.K_P = 0.3f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.1f, .config.fre = 100.0f, .config.n = 2, .pid[0]= {.K_P = 0.3f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.1f, .config.fre = 100.0f, .config.n = 2, .pid[0]= {.K_P = 0.3f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.1f, .config.fre = 100.0f, .config.n = 2, .pid[0]= {.K_P = 0.3f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0}

  };
  for (uint8_t i = 0; i < 12; i++)
    {
      // 重置控制�?
      memcpy(&bezier_exp[i].config.ctrl_point, &ctrl_point[i], sizeof(bezier_exp[i].config.ctrl_point));
    }

  if (dog->state == TURN_RIGHT)
    {
      // 轮流初始�?		leg_Bezier_Free_Init(&dog->leg[1], STEP_F1, &bezier_exp[STEP_F1]);
      leg_Bezier_Free_Init(&dog->leg[3], STEP_B1, &bezier_exp[STEP_B1]);
      leg_Bezier_Free_Init(&dog->leg[0], KICK_F1, &bezier_exp[KICK_F1]);
      leg_Bezier_Free_Init(&dog->leg[2], KICK_B1, &bezier_exp[KICK_B1]);
      while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
        {
          // 将当前轨迹走�?
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
      // 轮流初始�?
      leg_Bezier_Free_Init(&dog->leg[1], STEP_B1, &bezier_exp[STEP_B1]);
      leg_Bezier_Free_Init(&dog->leg[3], STEP_F1, &bezier_exp[STEP_F1]);
      leg_Bezier_Free_Init(&dog->leg[0], KICK_B1, &bezier_exp[KICK_B1]);
      leg_Bezier_Free_Init(&dog->leg[2], KICK_F1, &bezier_exp[KICK_F1]);
      while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
        {
          // 将当前轨迹走�?
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
// 低姿态前进步态流程�?
void LowWalkForward(Dog *dog)
{
  static const bezierPoint ctrl_point[4][4] =
  {
    {{0, 180.53f}, {12.5, 100.0f}, {25, 180.53f}, {100, 180.53f}}, // LOWSTEP_FORE1
    {{0, 180.53}, {-12.5, 200.0f}, {-25, 180.53}, {99, 214}},	   // LOWKICK_BACK1
    {{-25, 180.53f}, {-30, 100.0f}, {30, 100.0f}, {25, 180.53f}},  // LOWSTEP_FORE2
    {{25, 180.53}, {0, 200.0f}, {-25, 180.53f}, {99, 214}},		   // LOWKICK_BACK2};
  };
  // 初始化所有控制点�?{0,0}.exp_pos = {0.0f, 0.0f},.now_time = 0,.last_end_time = 0
  static bezier_exp bezier_exp[4] =
  {
    {.config.T = 0.3f, .config.fre = 50.0f, .config.n = 2, .pid[0] = {.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.3f, .config.fre = 50.0f, .config.n = 2, .pid[0] = {.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.3f, .config.fre = 50.0f, .config.n = 3, .pid[0]= {.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.3f, .config.fre = 50.0f, .config.n = 2, .pid[0]= {.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0}
  };
  for (uint8_t i = 0; i < 4; i++)
    {
      // 重置控制�?
      memcpy(&bezier_exp[i].config.ctrl_point, &ctrl_point[i], sizeof(bezier_exp[i].config.ctrl_point));
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

// 前跳动作流程�?
void JumpForward(Dog *dog)
{
  static const bezierPoint ctrl_point[4][4] =
  {
    {{-50, 140}, {-50, 140}, {0, 0}, {-0, 0}},		   // JUMP_PREPARE
    {{-50, 140}, {-114.285f, 320.0f}, {0, 0}, {0, 0}}, // JUMPING
    {{-114.285f, 320.0f}, {75, 210}, {0, 0}, {0, 0}},
    {{75, 210}, {50, 140}, {0, 0}, {0, 0.0}}, // JUMP_BUFFER
  };
  // 初始化所有控制点�?{0,0}.exp_pos = {0.0f, 0.0f},.now_time = 0,.last_end_time = 0
  static bezier_exp bezier_exp[4] =
  {
    {.config.T = 1.0f, .config.fre = 1.0f, .config.n = 0, .pid[0] = {.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.1f, .config.fre = 50.0f, .config.n = 1, .pid[0] = {.K_P = 3.0f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.1f, .config.fre = 50.0f, .config.n = 1, .pid[0] = {.K_P = 0.2f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.2f, .config.fre = 50.0f, .config.n = 1, .pid[0] = {.K_P = 0.2f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0}
  };
  for (uint8_t i = 0; i < 4; i++)
    {
      // 重置控制�?
      memcpy(&bezier_exp[i].config.ctrl_point, &ctrl_point[i], sizeof(bezier_exp[i].config.ctrl_point));
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

// 原地跳动作流程�?
void InJump(Dog *dog)
{
  static const bezierPoint ctrl_point[3][4] =
  {
    {{0, 130}, {0, 0}, {0, 0}, {-0, 0}},			// INJUMP_PREPARE
    {{0, 320}, {0, 320}, {35, 279.79f}, {70, 100}}, // INJUMP_TAKEOFF
    {{0, 320}, {0, 130}, {0, 100.0}, {0, 0.0}}		// INJUMP_LANDING
  };
  // 初始化所有控制点�?{0,0}.exp_pos = {0.0f, 0.0f},.now_time = 0,.last_end_time = 0
  static bezier_exp bezier_exp[3] =
  {
    {.config.T = 1.0f, .config.fre = 1.0f, .config.n = 0, .pid[0] = {.K_P = 0.2f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 1.0f, .config.fre = 1.0f, .config.n = 0, .pid[0] = {.K_P = 3.0f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.2f, .config.fre = 50.0f, .config.n = 1, .pid[0] = {.K_P = 0.2f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0}
  };
  for (uint8_t i = 0; i < 3; i++)
    {
      // 重置控制�?
      memcpy(&bezier_exp[i].config.ctrl_point, &ctrl_point[i], sizeof(bezier_exp[i].config.ctrl_point));
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

// 后退步态流程�?
void WalkBack(Dog *dog)
{
  static const bezierPoint ctrl_point[4][4] =
  {
    {{0, 190.53f}, {-12.5, 100.0f}, {-25, 190.53f}, {100, 190.53f}}, // STEP_FORE1
    {{0, 190.53}, {12.5, 200.0f}, {25, 190.53}, {99, 214}},			 // KICK_BACK1
    {{25, 190.53f}, {30, 100.0f}, {-30, 100.0f}, {-25, 190.53f}},	 // STEP_FORE2
    {{-25, 190.53}, {0, 200.0f}, {25, 190.53f}, {99, 214}},			 // KICK_BACK2
  };
  // 初始化所有控制点�?{0,0}.exp_pos = {0.0f, 0.0f},.now_time = 0,.last_end_time = 0
  static bezier_exp bezier_exp[4] =  //
  {
    {.config.T = 0.3f, .config.fre = 50.0f, .config.n = 2, .pid[0] = {.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.3f, .config.fre = 50.0f, .config.n = 2, .pid[0] = {.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.3f, .config.fre = 50.0f, .config.n = 3, .pid[0] = {.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0},
    {.config.T = 0.3f, .config.fre = 50.0f, .config.n = 2, .pid[0] = {.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 0.0f}, .now_time = 0, .last_end_time = 0}
  };
  for (uint8_t i = 0; i < 4; i++)
    {
      // 重置控制�?
      memcpy(&bezier_exp[i].config.ctrl_point, &ctrl_point[i], sizeof(bezier_exp[i].config.ctrl_point));
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

// 阻尼模式流程（低刚度）�?
void Damping_mode(Dog *dog)
{
  static bezier_exp bezier_exp = {.config.T = 1.0f, .config.fre = 1.0f, .config.n = 0, .pid[0] = {.K_P = 0.0f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, .t = 0.0f, .point_sum = 0, .flag = 0, .config.ctrl_point = {0}, .exp_pos = {0.0f, 190.53f}, .now_time = 0, .last_end_time = 0};
  for (uint8_t i = 0; i < 4; i++)
    {
      leg_Bezier_Free_Init(&dog->leg[i], DAMPING, &bezier_exp);
    }
  for (uint8_t i = 0; i < 4; i++)
    {
      posCtrlSolve(&dog->leg[i]);
    }
  while (dog->state == DAMPING_MODE)
    {
      osDelay(100);
    }
}

// VMC 模式
// 基于VMC的站立流程�?
void vmc_Stand(Dog *dog)
{
  static bezierPoint ctrl_point[4] = {{0, 190.53}, {0, 0}, {0, 0}, {0, 0}};
  static bezier_exp bezier_exp[1] =
  {
    {
      .config.T = 1.0f,
      .config.fre = 1.0f,
      .config.n = 0,
      .pid[0] = {.K_P = 0.3f, .K_W = 0.05f, .K_I = 0.01f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
      .t = 0.0f,
      .point_sum = 0,
      .flag = 0,
      .config.ctrl_point = {{0}}, // 初始化所有控制点�?{0,0}
      .exp_pos = {0.0f, 0.0f},
      .now_time = 0,
      .last_end_time = 0
    }
  };
  memcpy(&bezier_exp[0].config.ctrl_point, &ctrl_point, sizeof(bezier_exp[0].config.ctrl_point));
  leg_Bezier_Free_Init(&dog->leg[0], STAND_UP, &bezier_exp[0]);
  leg_Bezier_Free_Init(&dog->leg[1], STAND_UP, &bezier_exp[0]);
  leg_Bezier_Free_Init(&dog->leg[2], STAND_UP, &bezier_exp[0]);
  leg_Bezier_Free_Init(&dog->leg[3], STAND_UP, &bezier_exp[0]);
  for (uint8_t i = 0; i < 4; i++)
    {
      vmc_leg_Bezier_Act(&dog->leg[i]);
    }
}

void standUP_FPC(Dog *dog)
{
  static bezierPoint ctrl_point[4] = {{0, 190.53}, {0, 0}, {0, 0}, {0, 0}};
  static bezier_exp bezier_exp[1] =
  {
    {
      .config = {.T = 1.0f, .fre = 50.0f, .n = 0, .ctrl_point = {0}},
      .pid[0] = {.K_P = 0.25f, .K_W = 0.025f, .K_I = 0.01f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
      .pid[1] = {.K_P = 2.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
      .t = 0.0f,
      .point_sum = 0,
      .flag = 0,
      .exp_pos = {0.0f, 0.0f},
      .exp_fvel = {0.0f, 0.0f},
      .now_time = 0,
      .last_end_time = 0
    }
  };
  memcpy(&bezier_exp[0].config.ctrl_point, &ctrl_point, sizeof(bezier_exp[0].config.ctrl_point));
  bezier_exp[0].pid[0].K_P=vofa_data;
  
  leg_Bezier_Free_Init(&dog->leg[0], STAND_UP, &bezier_exp[0]);
  leg_Bezier_Free_Init(&dog->leg[1], STAND_UP, &bezier_exp[0]);
  leg_Bezier_Free_Init(&dog->leg[2], STAND_UP, &bezier_exp[0]);
  leg_Bezier_Free_Init(&dog->leg[3], STAND_UP, &bezier_exp[0]);
  while (dog->leg[0].bezier.flag == 0 || dog->leg[1].bezier.flag == 0 || dog->leg[2].bezier.flag == 0 || dog->leg[3].bezier.flag == 0)
    {
//  while(dog->leg[2].bezier.flag==0)      {

      for (uint8_t i = 0; i < 4; i++)
        {
          fpc_Leg_Bezier_Act(&dog->leg[i]);
        }
    }
//   }
}


