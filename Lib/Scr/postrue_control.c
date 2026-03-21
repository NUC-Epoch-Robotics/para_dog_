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
#include "postrue_control.h"
#include "vofa_Debug.h"

#define pi 3.141592f
#define LINK_INIT1 2.15857f
#define LINK_INIT2 2.78744f
#define VMC_FOOT_FORCE_LIMIT 300.0f // N
#define VMC_TORQUE_LIMIT 1.0f
#define VMC_VEL_LPF_ALPHA 0.25f
#define VMC_POS_ERR_SHAPE_MM 40.0f
#define VMC_VEL_ERR_LIMIT 1200.0f
#define VMC_FORCE_DEADZONE 0.005f
#define VMC_Y_AXIS_STIFFNESS_SCALE 1.0f

volatile uint32_t g_vmc_force_sat_count[4] = {0};
volatile uint32_t g_vmc_torque_sat_count[4] = {0};
static float g_vmc_fvel_lpf[4][2] = {0};
extern float vofa_data[10];
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
  memset(&dog->location, 0, sizeof(dog_location)); //
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
  a = (leg->motor_ctrl_linkf.data.Pos - leg->motor_ctrl_linkf.init_pos) / 6.33f;
  b = (leg->motor_ctrl_linkb.data.Pos - leg->motor_ctrl_linkb.init_pos) / 6.33f;
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
    leg->theta_back = (2.0f * pi - Alpha1) - b;
  }
  else if (leg->id == 4)
  {
    leg->theta_fore = Alpha2 - a;
    leg->theta_back = (2.0f * pi - Alpha1) + b;
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
  leg->motor_ctrl_linkf.pid.W = leg->bezier.pid[0].W;     // W
  leg->motor_ctrl_linkf.pid.T = leg->bezier.pid[0].T;

  // 后肢
  leg->motor_ctrl_linkb.pid.K_P = leg->bezier.pid[0].K_P; // K_p
  leg->motor_ctrl_linkb.pid.K_W = leg->bezier.pid[0].K_W; // K_w
  leg->motor_ctrl_linkb.pid.W = leg->bezier.pid[0].W;     // W
  leg->motor_ctrl_linkb.pid.T = leg->bezier.pid[0].T;
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
  leg->Fxy[0] = vmc_deadzone(leg->Fxy[0], VMC_FORCE_DEADZONE);                                                                                         // 死区处理
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
void leg_Bezier_Free_Inituper(Leg *leg, leg_state state, bezier_exp *bezier_para, pid_ *pid_para)
{
  leg->state = state;
  memcpy(&leg->bezier, bezier_para, sizeof(leg->bezier));
  if (pid_para != NULL)
  {
    memcpy(&leg->bezier.pid, pid_para, sizeof(leg->bezier.pid));
  }
}

static void bezier_exp_reset(bezier_exp *bezier_para,
                             float duration,
                             float frequency,
                             uint8_t order,
                             const bezierPoint ctrl_point[4],
                             const pid_ pid_para[2])
{
  bezier_para->config.T = duration;
  bezier_para->config.fre = frequency;
  bezier_para->config.n = order;
  bezier_para->t = 0.0f;
  bezier_para->point_sum = 0;
  bezier_para->flag = 0;
  bezier_para->exp_pos.x = 0.0f;
  bezier_para->exp_pos.y = 0.0f;
  bezier_para->exp_fvel[0] = 0.0f;
  bezier_para->exp_fvel[1] = 0.0f;
  bezier_para->now_time = 0;
  bezier_para->last_end_time = 0;
  memcpy(&bezier_para->config.ctrl_point, ctrl_point, sizeof(bezier_para->config.ctrl_point));
  if (pid_para != NULL)
  {
    memcpy(&bezier_para->pid, pid_para, sizeof(bezier_para->pid));
  }
  else
  {
    memset(&bezier_para->pid, 0, sizeof(bezier_para->pid));
  }
}

static void pid_build_scaled(const pid_ base_pid[2], const float loop_ratio[2], pid_ out_pid[2])
{
  for (uint8_t loop = 0; loop < 2; loop++)
  {
    out_pid[loop] = base_pid[loop];
    out_pid[loop].K_P *= loop_ratio[loop];
    out_pid[loop].K_W *= loop_ratio[loop];
    out_pid[loop].K_I *= loop_ratio[loop];
    out_pid[loop].xyPosIntegral[0] *= loop_ratio[loop];
    out_pid[loop].xyPosIntegral[1] *= loop_ratio[loop];
    out_pid[loop].Pos *= loop_ratio[loop];
    out_pid[loop].W *= loop_ratio[loop];
    out_pid[loop].T *= loop_ratio[loop];
  }
}

static void dog_InitLegBezierBatch(Dog *dog, const leg_state state_by_leg[4], void *const bezier_by_leg[4])
{
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    leg_Bezier_Free_Init(&dog->leg[leg], state_by_leg[leg], (bezier_exp *)bezier_by_leg[leg]);
  }
}

static void dog_InitSingleLegBezier(Dog *dog, uint8_t leg_index, leg_state state, bezier_exp *bezier_para)
{
  leg_Bezier_Free_Init(&dog->leg[leg_index], state, bezier_para);
}

static uint8_t dog_IsBezierBatchFinished(const Dog *dog)
{
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    if (dog->leg[leg].bezier.flag == 0)
    {
      return 0;
    }
  }
  return 1;
}

static void dog_RunBezierBatch(Dog *dog, void (*act)(Leg *))
{
  while (!dog_IsBezierBatchFinished(dog))
  {
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      act(&dog->leg[leg]);
    }
  }
}

static void dog_RunSingleLegBezierUntilFinish(Dog *dog, uint8_t leg_index, void (*act)(Leg *))
{
  while (dog->leg[leg_index].bezier.flag == 0)
  {
    act(&dog->leg[leg_index]);
  }
}
// 按动作周期与采样频率更新贝塞尔目标点
void leg_BezierTargetPos_Update(Leg *leg)
{
  leg->bezier.exp_pos = bezierCurve((const bezierPoint(*)[4]) & leg->bezier.config.ctrl_point, leg->bezier.config.n, leg->bezier.t); // 更新pos
  leg->bezier.t += 1 / (leg->bezier.config.T * leg->bezier.config.fre);                                                              // 更新比例系数
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
  leg->bezier.now_time = osKernelSysTick();
  if (leg->bezier.point_sum < leg->bezier.config.T * leg->bezier.config.fre && (leg->bezier.now_time >= (leg->bezier.last_end_time + (uint32_t)(1000 / leg->bezier.config.fre))))
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
  //  if(leg->bezier.point_sum>0)
  //    {
  //      GO_PFC_Ctrl(&(leg->motor_ctrl_linkf), 1);
  //      GO_PFC_Ctrl(&(leg->motor_ctrl_linkb), 1);
  //    }
  if (leg->bezier.point_sum < leg->bezier.config.T * leg->bezier.config.fre && (leg->bezier.now_time >= (leg->bezier.last_end_time + (uint32_t)(1000 / leg->bezier.config.fre))))
  {

    leg_BezierTargetPos_Update(leg);
    posCtrlSolve(leg);
    vmc_Solve(leg);

    GO_PFC_Ctrl(&(leg->motor_ctrl_linkf), 1);
    GO_PFC_Ctrl(&(leg->motor_ctrl_linkb), 1);
    if (leg->bezier.point_sum >= leg->bezier.config.T * leg->bezier.config.fre)
    {
      leg->bezier.flag = 1;
    }
    leg->bezier.last_end_time = osKernelSysTick();
  }
  //  GO_PFC_Ctrl(&(leg->motor_ctrl_linkf), 1);
  //  GO_PFC_Ctrl(&(leg->motor_ctrl_linkb), 1);
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
void trotRun_FPC(Dog *dog, float speed_level);
void walk_FPC(Dog *dog);
void Trot_RotateJump_FPC(Dog *dog);
// 行为状态机分发函数�?
void dogTaskCtrl(Dog *dog)
{
  if (dog->dog_mode == RC_MODE)
  {
    switch (dog->state)
    {
    case STAND_UP_:
      standUP_FPC(dog);
      //          standUP(dog);
      break;
    case WALK_FORWARD:
      //        walkForward_FPC(dog);
      trotRun_FPC(dog, vofa_data[2]);
      //        walkForward(dog);
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
      trotRun_FPC(dog, vofa_data[2]);
      break;
    case DAMPING_MODE:
      Damping_mode(dog);
      break;
    case TROT_ROTATEJUMP:
      Trot_RotateJump_FPC(dog);
      break;
    default:
      standUP(dog);
      break;
    }
  }
}
// 四腿站立动作流程�?
void standUP(Dog *dog)
{
  static const float stage_duration[1] = {1.0f};
  static const float stage_frequency[1] = {1.0f};
  static const uint8_t stage_order[1] = {0};
  static const bezierPoint ctrl_point[1][4] =
      {
          {{0, 190.53f}, {0, 0}, {0, 0}, {0, 0}}};
  // PID stage mapping: [0]=STAND_UP
  static const pid_ pid[1][2] =
      {
          {{.K_P = 0.2f, .K_W = 0.02f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {0}}};
  static bezier_exp bezier_table[4][1];
  static const leg_state stage_state[4] = {STAND_UP, STAND_UP, STAND_UP, STAND_UP};
  void *stage_bezier[4];

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    bezier_exp_reset(&bezier_table[leg][STAND_UP],
                     stage_duration[STAND_UP],
                     stage_frequency[STAND_UP],
                     stage_order[STAND_UP],
                     ctrl_point[STAND_UP],
                     pid[STAND_UP]);
    stage_bezier[leg] = &bezier_table[leg][STAND_UP];
  }

  dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
  for (uint8_t i = 0; i < 4; i++)
  {
    leg_Bezier_Act(&dog->leg[i]);
  }
}

// 前进步态流程�?
void walkForward(Dog *dog)
{
  static const float stage_duration[6] = {0.14f, 0.14f, 0.14f, 0.14f, 0.14f, 0.14f};
  static const float stage_frequency[6] = {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};
  static const uint8_t stage_order[6] = {2, 2, 3, 2, 2, 2};
  static const bezierPoint ctrl_point[6][4] =
      {
          {{0, 245.96f}, {25.0f, 150.0f}, {50.0f, 245.96f}, {100, 190.53f}},  // STEP_FORE1
          {{0, 245.96f}, {-25.0f, 270.0f}, {-50.0f, 245.96}, {99, 214}},      // KICK_BACK1
          {{-50.0f, 245.96f}, {-25, 130.0f}, {25, 130.0f}, {50.0f, 245.96f}}, // STEP_FORE2
          {{50.0f, 245.96f}, {0, 290.0f}, {-50.0f, 245.96f}, {99, 214}},      // KICK_BACK2

          {{50.0f, 245.96f}, {25.0f, 270.0f}, {0, 245.96f}, {0, 0}},  // STEP_FORE_REBACK
          {{-50.0f, 245.96f}, {-25.0f, 150.0f}, {0, 245.96f}, {0, 0}} // KICK_BACK_REBACK
      };
  // PID stage mapping: [0]=STEP_FORE1, [1]=KICK_BACK1, [2]=STEP_FORE2, [3]=KICK_BACK2, [4]=STEP_FORE_REBACK, [5]=KICK_BACK_REBACK
  static const pid_ pid_base[6][2] =
      {
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
      };
  static const float leg_pid_ratio[4][2] =
      {
          {1.0f, 1.0f},
          {1.0f, 1.0f},
          {1.0f, 1.0f},
          {1.0f, 1.0f},
      };
  static bezier_exp bezier_table[4][6];
  void *stage_bezier[4];
  leg_state stage_state[4];

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    for (uint8_t stage = 0; stage < 6; stage++)
    {
      pid_ leg_pid[2];
      pid_build_scaled(pid_base[stage], leg_pid_ratio[leg], leg_pid);
      bezier_exp_reset(&bezier_table[leg][stage],
                       stage_duration[stage],
                       stage_frequency[stage],
                       stage_order[stage],
                       ctrl_point[stage],
                       leg_pid);
    }
  }

  stage_state[0] = STEP_FORE1;
  stage_state[1] = KICK_BACK1;
  stage_state[2] = STEP_FORE1;
  stage_state[3] = KICK_BACK1;
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
  }
  dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
  dog_RunBezierBatch(dog, leg_Bezier_Act);
  osDelay(1);

  while (dog->state == WALK_FORWARD)
  {
    stage_state[0] = KICK_BACK2;
    stage_state[1] = STEP_FORE2;
    stage_state[2] = KICK_BACK2;
    stage_state[3] = STEP_FORE2;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, leg_Bezier_Act);

    stage_state[0] = STEP_FORE2;
    stage_state[1] = KICK_BACK2;
    stage_state[2] = STEP_FORE2;
    stage_state[3] = KICK_BACK2;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, leg_Bezier_Act);
    osDelay(1);
  }

  stage_state[0] = STEP_FORE_REBACK;
  stage_state[1] = KICK_BACK_REBACK;
  stage_state[2] = STEP_FORE_REBACK;
  stage_state[3] = KICK_BACK_REBACK;
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
  }
  dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
}
// 转向步态流程（左转/右转共用）�?
void Turn(Dog *dog)
{
  static const float stage_duration[12] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f};
  static const float stage_frequency[12] = {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};
  static const uint8_t stage_order[12] = {2, 2, 2, 2, 3, 3, 2, 2, 2, 2, 2, 2};
  static const bezierPoint ctrl_point[12][4] =
      {
          {{0, 190.53f}, {-25.0f, 100.0f}, {-25.0f, 190.53f}, {-100, 190.53f}},     // STEP_B1
          {{0, 190.53f}, {25, 100.0f}, {50, 190.53f}, {100, 190.53f}},              // STEP_F1
          {{0, 190.53}, {25.0f, 200.0f}, {50.0f, 190.53}, {99, 214}},               // KICK_F1
          {{0, 190.53}, {-25, 200.0f}, {-50, 190.53}, {99, 214}},                   // KICK_B1
          {{50.0f, 190.53f}, {50.0f, 100.0f}, {-50.0f, 100.0f}, {-50.0f, 190.53f}}, // STEP_B2
          {{-50, 190.53f}, {-50, 100.0f}, {50, 100.0f}, {50, 190.53f}},             // STEP_F2
          {{-50.0f, 190.53}, {0, 200.0f}, {50.0f, 190.53f}, {99, 214}},             // KICK_F2
          {{50, 190.53}, {0, 200.0f}, {-50, 190.53f}, {99, 214}},                   // KICK_B2

          {{50.0f, 190.53f}, {25.0f, 200.0f}, {0, 190.53f}, {0, 0}},   // STEP_F_REBACK
          {{-50.0f, 190.53f}, {-25.0f, 200.0f}, {0, 190.53f}, {0, 0}}, // STEP_B_REBACK
          {{50.0f, 190.53f}, {25.0f, 100.0f}, {0, 190.53f}, {0, 0}},   // KICK_F_REBACK
          {{-50.0f, 190.53f}, {-25.0f, 100.0f}, {0, 190.53f}, {0, 0}}  // KICK_B_REBACK
      };
  // PID stage mapping: [0]=STEP_B1, [1]=STEP_F1, [2]=KICK_F1, [3]=KICK_B1, [4]=STEP_B2, [5]=STEP_F2, [6]=KICK_F2, [7]=KICK_B2, [8]=STEP_F_REBACK, [9]=STEP_B_REBACK, [10]=KICK_F_REBACK, [11]=KICK_B_REBACK
  static const pid_ pid[12][2] =
      {
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.4f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.4f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
      };
  static bezier_exp bezier_table[4][12];
  void *stage_bezier[4];
  leg_state stage_state[4];

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    for (uint8_t stage = 0; stage < 12; stage++)
    {
      bezier_exp_reset(&bezier_table[leg][stage],
                       stage_duration[stage],
                       stage_frequency[stage],
                       stage_order[stage],
                       ctrl_point[stage],
                       pid[stage]);
    }
  }

  if (dog->state == TURN_RIGHT)
  {
    stage_state[0] = KICK_F1;
    stage_state[1] = STEP_F1;
    stage_state[2] = KICK_B1;
    stage_state[3] = STEP_B1;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
    osDelay(1);

    while (dog->state == TURN_RIGHT)
    {
      stage_state[0] = STEP_B2;
      stage_state[1] = KICK_B2;
      stage_state[2] = STEP_F2;
      stage_state[3] = KICK_F2;
      for (uint8_t leg = 0; leg < 4; leg++)
      {
        stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
      }
      dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
      dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);

      stage_state[0] = KICK_F2;
      stage_state[1] = STEP_F2;
      stage_state[2] = KICK_B2;
      stage_state[3] = STEP_B2;
      for (uint8_t leg = 0; leg < 4; leg++)
      {
        stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
      }
      dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
      dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
      osDelay(1);
    }

    stage_state[0] = KICK_F_REBACK;
    stage_state[1] = STEP_F_REBACK;
    stage_state[2] = KICK_B_REBACK;
    stage_state[3] = STEP_B_REBACK;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
    osDelay(1);
  }
  else if (dog->state == TURN_LEFT)
  {
    stage_state[0] = KICK_B1;
    stage_state[1] = STEP_B1;
    stage_state[2] = KICK_F1;
    stage_state[3] = STEP_F1;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
    osDelay(1);

    while (dog->state == TURN_LEFT)
    {
      stage_state[0] = STEP_F2;
      stage_state[1] = KICK_F2;
      stage_state[2] = STEP_B2;
      stage_state[3] = KICK_B2;
      for (uint8_t leg = 0; leg < 4; leg++)
      {
        stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
      }
      dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
      dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);

      stage_state[0] = KICK_B2;
      stage_state[1] = STEP_B2;
      stage_state[2] = KICK_F2;
      stage_state[3] = STEP_F2;
      for (uint8_t leg = 0; leg < 4; leg++)
      {
        stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
      }
      dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
      dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
      osDelay(1);
    }

    stage_state[0] = KICK_B_REBACK;
    stage_state[1] = STEP_B_REBACK;
    stage_state[2] = KICK_F_REBACK;
    stage_state[3] = STEP_F_REBACK;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
    osDelay(1);
  }
}
// 低姿态前进步态流程�?
void LowWalkForward(Dog *dog)
{
  static const float stage_duration[4] = {0.3f, 0.3f, 0.3f, 0.3f};
  static const float stage_frequency[4] = {50.0f, 50.0f, 50.0f, 50.0f};
  static const uint8_t stage_order[4] = {2, 2, 3, 2};
  static const bezierPoint ctrl_point[4][4] =
      {
          {{0, 180.53f}, {12.5, 100.0f}, {25, 180.53f}, {100, 180.53f}}, // LOWSTEP_FORE1
          {{0, 180.53}, {-12.5, 200.0f}, {-25, 180.53}, {99, 214}},      // LOWKICK_BACK1
          {{-25, 180.53f}, {-30, 100.0f}, {30, 100.0f}, {25, 180.53f}},  // LOWSTEP_FORE2
          {{25, 180.53}, {0, 200.0f}, {-25, 180.53f}, {99, 214}},        // LOWKICK_BACK2};
      };
  // PID stage mapping: [0]=LOWSTEP_FORE1, [1]=LOWKICK_BACK1, [2]=LOWSTEP_FORE2, [3]=LOWKICK_BACK2
  static const pid_ pid[4][2] =
      {
          {{.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
      };
  static bezier_exp bezier_table[4][4];
  void *stage_bezier[4];
  leg_state stage_state[4];

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    for (uint8_t stage = 0; stage < 4; stage++)
    {
      bezier_exp_reset(&bezier_table[leg][stage],
                       stage_duration[stage],
                       stage_frequency[stage],
                       stage_order[stage],
                       ctrl_point[stage],
                       pid[stage]);
    }
  }

  stage_state[0] = LOWSTEP_FORE1;
  stage_state[1] = LOWKICK_BACK1;
  stage_state[2] = LOWSTEP_FORE1;
  stage_state[3] = LOWKICK_BACK1;
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
  }
  dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
  dog_RunBezierBatch(dog, leg_Bezier_Act);
  osDelay(1);

  while (dog->state == LOWWALK_FORWARD)
  {
    stage_state[0] = LOWKICK_BACK2;
    stage_state[1] = LOWSTEP_FORE2;
    stage_state[2] = LOWKICK_BACK2;
    stage_state[3] = LOWSTEP_FORE2;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, leg_Bezier_Act);

    stage_state[0] = LOWSTEP_FORE2;
    stage_state[1] = LOWKICK_BACK2;
    stage_state[2] = LOWSTEP_FORE2;
    stage_state[3] = LOWKICK_BACK2;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, leg_Bezier_Act);
    osDelay(1);
  }
}

// 前跳动作流程�?
void JumpForward(Dog *dog)
{
  static const float stage_duration[4] = {1.0f, 0.2f, 0.2f, 0.2f};
  static const float stage_frequency[4] = {10.0f, 100.0f, 50.0f, 50.0f};
  static const uint8_t stage_order[4] = {0, 3, 3, 1};
  static const bezierPoint ctrl_point[4][4] =
      {
          {{-50, 140}, {-50, 140}, {0, 0}, {-0, 0}},                                    // JUMP_PREPARE
          {{-50, 140.0f}, {-114.285f, 350.0f}, {-178.57, 350.0f}, {-242.855f, 140.0f}}, // JUMPING
          {{-242.855f, 140.0f}, {-178.57, 130}, {-114.285, 130}, {50.0f, 140}},
          {{0, 190.53}, {50, 140}, {0, 0}, {0, 0.0}}, // JUMP_BUFFER
      };
  // PID stage mapping: [0]=JUMP_PREPARE, [1]=JUMPING, [2]=JUMP_LANDING, [3]=JUMP_BUFFER
  static const pid_ pid_base[4][2] =
      {
          {{.K_P = 0.1f, .K_W = 0.03f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 5.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.03f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.03f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
      };
  static const float leg_pid_ratio[4][2] =
      {
          {1.0f, 1.0f},
          {1.0f, 1.0f},
          {1.0f, 1.0f},
          {1.0f, 1.0f},
      };
  static bezier_exp bezier_table[4][4];
  void *stage_bezier[4];

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    for (uint8_t stage = 0; stage < 4; stage++)
    {
      pid_ leg_pid[2];
      pid_build_scaled(pid_base[stage], leg_pid_ratio[leg], leg_pid);
      bezier_exp_reset(&bezier_table[leg][stage],
                       stage_duration[stage],
                       stage_frequency[stage],
                       stage_order[stage],
                       ctrl_point[stage],
                       leg_pid);
    }
  }

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][JUMP_PREPARE];
  }
  dog_InitLegBezierBatch(dog,
                         (const leg_state[4]){
                             JUMP_PREPARE, JUMP_PREPARE, JUMP_PREPARE, JUMP_PREPARE},
                         stage_bezier);
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
  osDelay(1000);

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][JUMPING];
  }
  dog_InitLegBezierBatch(dog,
                         (const leg_state[4]){
                             JUMPING, JUMPING, JUMPING, JUMPING},
                         stage_bezier);
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
  osDelay(200);

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][JUMP_LANDING];
  }
  dog_InitLegBezierBatch(dog,
                         (const leg_state[4]){
                             JUMP_LANDING, JUMP_LANDING, JUMP_LANDING, JUMP_LANDING},
                         stage_bezier);
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
  osDelay(100);

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][JUMP_BUFFER];
  }
  dog_InitLegBezierBatch(dog,
                         (const leg_state[4]){
                             JUMP_BUFFER, JUMP_BUFFER, JUMP_BUFFER, JUMP_BUFFER},
                         stage_bezier);
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
  osDelay(1000);
  dog->state = STAND_UP_;
}

// 原地跳动作流程�?
void InJump(Dog *dog)
{
  static const float stage_duration[3] = {1.0f, 1.0f, 0.2f};
  static const float stage_frequency[3] = {1.0f, 1.0f, 50.0f};
  static const uint8_t stage_order[3] = {0, 0, 1};
  static const bezierPoint ctrl_point[3][4] =
      {
          {{0, 130}, {0, 0}, {0, 0}, {-0, 0}},            // INJUMP_PREPARE
          {{0, 320}, {0, 320}, {35, 279.79f}, {70, 100}}, // INJUMP_TAKEOFF
          {{0, 320}, {0, 130}, {0, 100.0}, {0, 0.0}}      // INJUMP_LANDING
      };
  // PID stage mapping: [0]=INJUMP_PREPARE, [1]=INJUMP_TAKEOFF, [2]=INJUMP_LANDING
  static const pid_ pid[3][2] =
      {
          {{.K_P = 0.2f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 3.0f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 0.2f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
      };
  static bezier_exp bezier_table[4][3];
  static const leg_state stage_state[4] = {INJUMP_PREPARE, INJUMP_PREPARE, INJUMP_PREPARE, INJUMP_PREPARE};
  void *stage_bezier[4];

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    for (uint8_t stage = 0; stage < 3; stage++)
    {
      bezier_exp_reset(&bezier_table[leg][stage],
                       stage_duration[stage],
                       stage_frequency[stage],
                       stage_order[stage],
                       ctrl_point[stage],
                       pid[stage]);
    }
  }

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][INJUMP_PREPARE];
  }
  dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
  dog_RunBezierBatch(dog, leg_Bezier_Act);

  osDelay(300);
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][INJUMP_TAKEOFF];
  }
  dog_InitLegBezierBatch(dog, (const leg_state[4]){INJUMP_TAKEOFF, INJUMP_TAKEOFF, INJUMP_TAKEOFF, INJUMP_TAKEOFF}, stage_bezier);
  dog_RunBezierBatch(dog, leg_Bezier_Act);

  osDelay(500);
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][INJUMP_LANDING];
  }
  dog_InitLegBezierBatch(dog, (const leg_state[4]){INJUMP_LANDING, INJUMP_LANDING, INJUMP_LANDING, INJUMP_LANDING}, stage_bezier);
  dog_RunBezierBatch(dog, leg_Bezier_Act);
  osDelay(1000);
}

// 后退步态流程�?
void WalkBack(Dog *dog)
{
  static const float stage_duration[4] = {0.3f, 0.3f, 0.3f, 0.3f};
  static const float stage_frequency[4] = {50.0f, 50.0f, 50.0f, 50.0f};
  static const uint8_t stage_order[4] = {2, 2, 3, 2};
  static const bezierPoint ctrl_point[4][4] =
      {
          {{0, 190.53f}, {-12.5, 100.0f}, {-25, 190.53f}, {100, 190.53f}}, // STEP_FORE1
          {{0, 190.53}, {12.5, 200.0f}, {25, 190.53}, {99, 214}},          // KICK_BACK1
          {{25, 190.53f}, {30, 100.0f}, {-30, 100.0f}, {-25, 190.53f}},    // STEP_FORE2
          {{-25, 190.53}, {0, 200.0f}, {25, 190.53f}, {99, 214}},          // KICK_BACK2
      };
  // PID stage mapping: [0]=STEP_FORE1_BACK, [1]=KICK_BACK1_BACK, [2]=STEP_FORE2_BACK, [3]=KICK_BACK2_BACK
  static const pid_ pid[4][2] =
      {
          {{.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 0.6f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
      };
  static bezier_exp bezier_table[4][4];
  void *stage_bezier[4];
  leg_state stage_state[4];

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    for (uint8_t stage = 0; stage < 4; stage++)
    {
      bezier_exp_reset(&bezier_table[leg][stage],
                       stage_duration[stage],
                       stage_frequency[stage],
                       stage_order[stage],
                       ctrl_point[stage],
                       pid[stage]);
    }
  }

  stage_state[0] = STEP_FORE1_BACK;
  stage_state[1] = KICK_BACK1_BACK;
  stage_state[2] = STEP_FORE1_BACK;
  stage_state[3] = KICK_BACK1_BACK;
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
  }
  dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
  dog_RunBezierBatch(dog, leg_Bezier_Act);
  osDelay(1);
  while (dog->state == WALK_BACK)
  {
    stage_state[0] = KICK_BACK2_BACK;
    stage_state[1] = STEP_FORE2_BACK;
    stage_state[2] = KICK_BACK2_BACK;
    stage_state[3] = STEP_FORE2_BACK;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, leg_Bezier_Act);

    stage_state[0] = STEP_FORE2_BACK;
    stage_state[1] = KICK_BACK2_BACK;
    stage_state[2] = STEP_FORE2_BACK;
    stage_state[3] = KICK_BACK2_BACK;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, leg_Bezier_Act);
    osDelay(1);
  }
}

// 阻尼模式流程（低刚度）�?
void Damping_mode(Dog *dog)
{
  // PID stage mapping: [0]=position loop, [1]=force/torque loop (unused here)
  static const pid_ pid[2] =
      {
          {.K_P = 0.0f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
          {0}};
  static const bezierPoint ctrl_point[4] = {{0.0f, 190.53f}, {0, 0}, {0, 0}, {0, 0}};
  static bezier_exp bezier_table[4][1];
  static const leg_state stage_state[4] = {DAMPING, DAMPING, DAMPING, DAMPING};
  void *stage_bezier[4];

  for (uint8_t i = 0; i < 4; i++)
  {
    bezier_exp_reset(&bezier_table[i][DAMPING], 1.0f, 1.0f, 0, ctrl_point, pid);
    bezier_table[i][DAMPING].exp_pos.x = 0.0f;
    bezier_table[i][DAMPING].exp_pos.y = 190.53f;
    stage_bezier[i] = &bezier_table[i][DAMPING];
  }

  dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
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

void standUP_FPC(Dog *dog)
{
  static const bezierPoint ctrl_point[4] = {{0, 190.53f}, {0, 0}, {0, 0}, {0, 0}};
  // PID stage mapping: [0]=position loop, [1]=force/torque loop
  static const pid_ pid[2] =
      {
          {.K_P = 0.075f, .K_W = 0.0075f, .K_I = 0.01f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
          {.K_P = 0.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}};
  static bezier_exp bezier_table[4][1];
  static const leg_state stage_state[4] = {STAND_UP, STAND_UP, STAND_UP, STAND_UP};
  void *stage_bezier[4];

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    bezier_exp_reset(&bezier_table[leg][STAND_UP], 1.0f, 50.0f, 0, ctrl_point, pid);
    stage_bezier[leg] = &bezier_table[leg][STAND_UP];
  }

  vofa_data_write(0, &bezier_table[0][STAND_UP].pid[0].K_P);
  dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
}

void trotRun_FPC(Dog *dog, float speed_level)
{
  static const float stage_duration[6] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.3f};
  static const float stage_frequency[6] = {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};
  static const uint8_t stage_order[6] = {2, 2, 3, 3, 2, 2};
  static const bezierPoint ctrl_point[6][4] =
      {
          {{0, 190.53f}, {50.0f, 60.0f}, {100.0f, 190.53f}, {100, 190.53f}},  // STEP_FORE1
          {{0, 190.53f}, {-50.0f, 270.0f}, {-100.0f, 190.53}, {99, 214}},     // KICK_BACK1
          {{-100.0f, 190.53f}, {0, 100.0f}, {80, 100.0f}, {100.0f, 190.53f}}, // STEP_FORE2
          {{100.0f, 190.53f}, {80, 270.0f}, {0.0f, 270.0f}, {-100, 190.53}},  // KICK_BACK2

          {{100.0f, 190.53f}, {50.0f, 270.0f}, {0, 190.53f}, {0, 0}},  // STEP_FORE_REBACK
          {{-100.0f, 190.53f}, {-50.0f, 150.0f}, {0, 190.53f}, {0, 0}} // KICK_BACK_REBACK
      };
  // PID stage mapping: [0]=STEP_FORE1, [1]=KICK_BACK1, [2]=STEP_FORE2, [3]=KICK_BACK2, [4]=STEP_FORE_REBACK, [5]=KICK_BACK_REBACK
  static const pid_ pid_base[6][2] =
      {
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.4f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
      };
  static const float leg_pid_ratio[6][4][2] =
      {
          {{1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}},
          {{1.0f, 1.0f}, {1.0f, 1.0f}, {1.25f, 1.25f}, {1.25f, 1.25f}},
          {{1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}},
          {{1.0f, 1.0f}, {1.0f, 1.0f}, {1.25f, 1.25f}, {1.25f, 1.25f}},
          {{1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}},
          {{1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}},
      };
  static bezier_exp bezier_table[4][6];
  void *stage_bezier[4];
  leg_state stage_state[4];

  if (speed_level > 1 || speed_level < 0)
  {
    speed_level = 1;
  }
  bezierPoint ctrl_point_scaled[6][4];
  const float neutral_y = 190.53f;
  memcpy(ctrl_point_scaled, ctrl_point, sizeof(ctrl_point_scaled));

  static const uint8_t scale_x_idx[][2] = {
      {0, 1}, {0, 2}, {1, 1}, {1, 2}, {2, 0}, {2, 2}, {2, 3}, {3, 0}, {3, 1}, {3, 3}};
  static const uint8_t scale_y_idx[][2] = {
      {0, 1}, {1, 1}, {2, 1}, {2, 2}, {3, 1}, {3, 2}};

  for (uint8_t i = 0; i < (uint8_t)(sizeof(scale_x_idx) / sizeof(scale_x_idx[0])); i++)
  {
    const uint8_t stage = scale_x_idx[i][0];
    const uint8_t point = scale_x_idx[i][1];
    ctrl_point_scaled[stage][point].x *= speed_level;
  }

  for (uint8_t i = 0; i < (uint8_t)(sizeof(scale_y_idx) / sizeof(scale_y_idx[0])); i++)
  {
    const uint8_t stage = scale_y_idx[i][0];
    const uint8_t point = scale_y_idx[i][1];
    ctrl_point_scaled[stage][point].y = neutral_y + (ctrl_point_scaled[stage][point].y - neutral_y) * speed_level;
  }

  if (dog->state == WALK_BACK) // 判断前进方向
  {
    static const uint8_t mirror_x_idx[][2] = {
        {0, 1}, {0, 2}, {1, 1}, {1, 2}, {2, 0}, {2, 2}, {2, 3}, {3, 0}, {3, 1}, {3, 3}, {4, 0}, {4, 1}, {5, 0}, {5, 1}};
    for (uint8_t i = 0; i < (uint8_t)(sizeof(mirror_x_idx) / sizeof(mirror_x_idx[0])); i++)
    {
      const uint8_t stage = mirror_x_idx[i][0];
      const uint8_t point = mirror_x_idx[i][1];
      ctrl_point_scaled[stage][point].x *= -1.0f;
    }
  }

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    for (uint8_t stage = 0; stage < 6; stage++)
    {
      pid_ leg_pid[2];
      pid_build_scaled(pid_base[stage], leg_pid_ratio[stage][leg], leg_pid);
      bezier_exp_reset(&bezier_table[leg][stage],
                       stage_duration[stage],
                       stage_frequency[stage],
                       stage_order[stage],
                       ctrl_point_scaled[stage],
                       leg_pid);
    }
  }

  stage_state[0] = STEP_FORE1;
  stage_state[1] = KICK_BACK1;
  stage_state[2] = STEP_FORE1;
  stage_state[3] = KICK_BACK1;
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
  }
  dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
  osDelay(1);

  while (dog->state == WALK_FORWARD)
  {
    stage_state[0] = KICK_BACK2;
    stage_state[1] = STEP_FORE2;
    stage_state[2] = KICK_BACK2;
    stage_state[3] = STEP_FORE2;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);

    stage_state[0] = STEP_FORE2;
    stage_state[1] = KICK_BACK2;
    stage_state[2] = STEP_FORE2;
    stage_state[3] = KICK_BACK2;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
    osDelay(1);
  }

  stage_state[0] = STEP_FORE_REBACK;
  stage_state[1] = KICK_BACK_REBACK;
  stage_state[2] = STEP_FORE_REBACK;
  stage_state[3] = KICK_BACK_REBACK;
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
  }
  dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
}

void walk_FPC(Dog *dog)
{
  static const float stage_duration[6] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f};
  static const float stage_frequency[6] = {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};
  static const uint8_t stage_order[6] = {2, 2, 3, 3, 2, 2};
  static const bezierPoint ctrl_point[6][4] =
      {
          {{0, 190.53f}, {40.0f, 60.0f}, {80.0f, 190.53f}, {100, 190.53f}},   // STEP_FORE1
          {{0, 190.53f}, {-40.0f, 270.0f}, {-80.0f, 190.53}, {99, 214}},      // KICK_BACK1
          {{-80.0f, 190.53f}, {-64, 100.0f}, {64, 100.0f}, {80.0f, 190.53f}}, // STEP_FORE2
          {{80.0f, 190.53f}, {64, 270.0f}, {-64.0f, 270.0f}, {-80, 190.53}},  // KICK_BACK2

          {{100.0f, 190.53f}, {50.0f, 270.0f}, {0, 190.53f}, {0, 0}},  // STEP_FORE_REBACK
          {{-100.0f, 190.53f}, {-50.0f, 150.0f}, {0, 190.53f}, {0, 0}} // KICK_BACK_REBACK
      };
  // PID stage mapping: [0]=STEP_FORE1, [1]=KICK_BACK1, [2]=STEP_FORE2, [3]=KICK_BACK2, [4]=STEP_FORE_REBACK, [5]=KICK_BACK_REBACK
  static const pid_ pid_base[6][2] =
      {
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 0.3f, .K_W = 0.03f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 0.25f, .K_W = 0.025f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
      };
  static const float leg_pid_ratio[4][2] =
      {
          {1.0f, 1.0f},
          {1.0f, 1.0f},
          {1.0f, 1.0f},
          {1.0f, 1.0f},
      };
  static bezier_exp bezier_table[4][6];
  void *stage_bezier[4];
  leg_state stage_state[4];

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    for (uint8_t stage = 0; stage < 6; stage++)
    {
      pid_ leg_pid[2];
      pid_build_scaled(pid_base[stage], leg_pid_ratio[leg], leg_pid);
      bezier_exp_reset(&bezier_table[leg][stage],
                       stage_duration[stage],
                       stage_frequency[stage],
                       stage_order[stage],
                       ctrl_point[stage],
                       leg_pid);
    }
  }

  stage_state[0] = KICK_BACK1;
  stage_state[1] = STEP_FORE1;
  stage_state[2] = KICK_BACK1;
  stage_state[3] = KICK_BACK1;
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
  }
  dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);

  dog_InitSingleLegBezier(dog, 2, STEP_FORE2, &bezier_table[2][STEP_FORE2]);
  dog_RunSingleLegBezierUntilFinish(dog, 2, fpc_Leg_Bezier_Act);
  osDelay(1);

  while (dog->state == WALK_FORWARD)
  {
    dog_InitSingleLegBezier(dog, 3, STEP_FORE2, &bezier_table[3][STEP_FORE2]);
    dog_RunSingleLegBezierUntilFinish(dog, 3, fpc_Leg_Bezier_Act);
    osDelay(100);

    stage_state[0] = STEP_FORE2;
    stage_state[1] = KICK_BACK2;
    stage_state[2] = KICK_BACK2;
    stage_state[3] = KICK_BACK2;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
    osDelay(100);

    dog_InitSingleLegBezier(dog, 3, STEP_FORE2, &bezier_table[3][STEP_FORE2]);
    dog_RunSingleLegBezierUntilFinish(dog, 3, fpc_Leg_Bezier_Act);
    osDelay(100);

    dog_InitSingleLegBezier(dog, 2, STEP_FORE2, &bezier_table[2][STEP_FORE2]);
    dog_RunSingleLegBezierUntilFinish(dog, 2, fpc_Leg_Bezier_Act);
    osDelay(100);

    stage_state[0] = KICK_BACK2;
    stage_state[1] = STEP_FORE2;
    stage_state[2] = KICK_BACK2;
    stage_state[3] = KICK_BACK2;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act);
    osDelay(100);

    dog_InitSingleLegBezier(dog, 2, STEP_FORE2, &bezier_table[2][STEP_FORE2]);
    dog_RunSingleLegBezierUntilFinish(dog, 2, fpc_Leg_Bezier_Act);
    osDelay(100);
  }

  stage_state[0] = STEP_FORE_REBACK;
  stage_state[1] = KICK_BACK_REBACK;
  stage_state[2] = STEP_FORE_REBACK;
  stage_state[3] = KICK_BACK_REBACK;
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
  }
  dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
}

void Trot_RotateJump_FPC(Dog *dog)
{
  static const float stage_duration[6] = {0.1f, 0.1f, 0.2f, 0.2f, 0.2f, 0.2f};
  static const float stage_frequency[6] = {10.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};
  static const uint8_t stage_order[6] = {2, 2, 3, 3, 2, 2};
  static const bezierPoint ctrl_point[6][4] =
      {
          {{0, 190.53f}, {40.0f, 60.0f}, {80.0f, 190.53f}, {100, 190.53f}},   // STEP_FORE1
          {{0, 190.53f}, {-40.0f, 270.0f}, {-80.0f, 190.53}, {99, 214}},      // KICK_BACK1
          {{-80.0f, 190.53f}, {-64, 100.0f}, {64, 100.0f}, {80.0f, 190.53f}}, // STEP_FORE2
          {{80.0f, 190.53f}, {64, 270.0f}, {-64.0f, 270.0f}, {-80, 190.53}},  // KICK_BACK2

          {{100.0f, 190.53f}, {50.0f, 270.0f}, {0, 190.53f}, {0, 0}},  // STEP_FORE_REBACK
          {{-100.0f, 190.53f}, {-50.0f, 150.0f}, {0, 190.53f}, {0, 0}} // KICK_BACK_REBACK
      };
  // PID stage mapping: [0]=STEP_FORE1, [1]=KICK_BACK1, [2]=STEP_FORE2, [3]=KICK_BACK2, [4]=STEP_FORE_REBACK, [5]=KICK_BACK_REBACK
  static const pid_ pid_base[6][2] =
      {
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 0.3f, .K_W = 0.03f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
          {{.K_P = 0.25f, .K_W = 0.025f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
      };
  static const float leg_pid_ratio[4][2] =
      {
          {1.0f, 1.0f},
          {1.0f, 1.0f},
          {1.0f, 1.0f},
          {1.0f, 1.0f},
      };
  static bezier_exp bezier_table[4][6];
  void *stage_bezier[4];
  leg_state stage_state[4];
}
