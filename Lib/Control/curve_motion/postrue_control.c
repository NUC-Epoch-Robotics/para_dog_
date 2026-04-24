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
#define GAIT_SCALE_MIN 0.0f
#define GAIT_SCALE_MAX 1.8f

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

void leg_Inverse_Kinematics(Leg *leg)
{
  float A = pow(leg->bezier.exp_pos.x, 2) + pow(leg->bezier.exp_pos.y, 2) + pow(leg->L1, 2) - pow(leg->L2, 2);
  float theta_fore = -acos(A / ((2 * leg->L1) * sqrtf(pow(leg->bezier.exp_pos.x, 2) + pow(leg->bezier.exp_pos.y, 2)))) + atan2(leg->bezier.exp_pos.y, leg->bezier.exp_pos.x); // rad杩炴潌浣嶇疆
  float theta_back = acos(A / ((2 * leg->L1) * sqrtf(pow(leg->bezier.exp_pos.x, 2) + pow(leg->bezier.exp_pos.y, 2)))) + atan2(leg->bezier.exp_pos.y, leg->bezier.exp_pos.x);
  if (leg->id == 1)
  {
    leg->motor_ctrl_linkf.pid.Pos = (theta_fore * 6.33f) + (leg->motor_ctrl_linkf.init_pos + 1.77584f * 6.33f);
    leg->motor_ctrl_linkb.pid.Pos = -(theta_back * 6.33f) + (leg->motor_ctrl_linkb.init_pos + 2.52456f * 6.33f);
  }
  else if (leg->id == 2)
  {
    leg->motor_ctrl_linkf.pid.Pos = -(theta_fore * 6.33f) + (leg->motor_ctrl_linkf.init_pos - 1.77584f * 6.33f);
    leg->motor_ctrl_linkb.pid.Pos = (theta_back * 6.33f) + (leg->motor_ctrl_linkb.init_pos - 2.52456f * 6.33f);
  }
  else if (leg->id == 3)
  {
    leg->motor_ctrl_linkf.pid.Pos = (theta_fore * 6.33f) + (leg->motor_ctrl_linkf.init_pos - 0.61544f * 6.33f);
    leg->motor_ctrl_linkb.pid.Pos = -(theta_back * 6.33f) + (leg->motor_ctrl_linkb.init_pos + 4.9159f * 6.33f);
  }
  else if (leg->id == 4)
  {
    leg->motor_ctrl_linkf.pid.Pos = -(theta_fore * 6.33f) + (leg->motor_ctrl_linkf.init_pos + 0.61544f * 6.33f);
    leg->motor_ctrl_linkb.pid.Pos = (theta_back * 6.33f) + (leg->motor_ctrl_linkb.init_pos - 4.9159f * 6.33f);
    //		leg->motor_ctrl_linkb.pid.Pos = (leg->motor_ctrl_linkb .init_pos);
  }
}
// 姝ｈ繍鍔ㄥ锛氱敱鍏宠妭瑙掍及璁″綋鍓嶈冻绔綅缃�?
void leg_Forward_Kinematics(Leg *leg)
{
  static float Alpha1 = (78.1954f / 180) * pi;
  static float Alpha2 = (35.2819f / 180) * pi;
  float a, b, deta, phi;
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
  deta = 0.5f * (leg->theta_fore - leg->theta_back);
  phi = 0.5f * (leg->theta_fore + leg->theta_back);
  leg->x = cosf(phi) * (125.0f * cosf(deta) + 62.5f * sqrtf(2.0f) * sqrtf(cosf(2 * deta) + 7.0f));
  leg->y = sinf(phi) * (125.0f * cosf(deta) + 62.5f * sqrtf(2.0f) * sqrtf(cosf(2 * deta) + 7.0f));
  leg->phi = phi;
}
// 浣嶇疆妯″紡鍗曡吙鎺у埗锛氭寜褰撳墠璐濆灏旂洰鏍囧彂閫佷綅缃寚浠わ�?
void posCtrlSolve(Leg *leg)
{
  leg_Inverse_Kinematics(leg);

  // 鍓嶈�?
  leg->motor_ctrl_linkf.pid.K_P = leg->bezier.pid[0].K_P; // K_p
  leg->motor_ctrl_linkf.pid.K_W = leg->bezier.pid[0].K_W; // K_w
  leg->motor_ctrl_linkf.pid.W = leg->bezier.pid[0].W;     // W
  leg->motor_ctrl_linkf.pid.T = leg->bezier.pid[0].T;

  // 鍚庤�?
  leg->motor_ctrl_linkb.pid.K_P = leg->bezier.pid[0].K_P; // K_p
  leg->motor_ctrl_linkb.pid.K_W = leg->bezier.pid[0].K_W; // K_w
  leg->motor_ctrl_linkb.pid.W = leg->bezier.pid[0].W;     // W
  leg->motor_ctrl_linkb.pid.T = leg->bezier.pid[0].T;
}
// 鏋勫缓闆呭彲姣旂煩闃碉紝�?��簬閫熷害/鍔涙槧灏勶拷?
static void leg_Calc_Jacobian(const Leg *leg, float Jacobian[2][2])
{
  float sigma[6];
  // ?? = ?cos(?? - ??) + 7
  sigma[0] = sqrtf(cosf(leg->theta_fore - leg->theta_back) + 7.0f);
  sigma[0] = fmaxf(sigma[0], 1e-6f); // ????0

  // ?? = ??/2 - 3??/2
  sigma[1] = 0.5f * leg->theta_fore - 1.5f * leg->theta_back;

  // ?? = 3??/2 - ??/2
  sigma[2] = 1.5f * leg->theta_fore - 0.5f * leg->theta_back;

  // ?? = (?? + ??)/2
  sigma[5] = 0.5f * (leg->theta_fore + leg->theta_back);

  // ?? = 875?2 sin(??)
  sigma[3] = 875.0f * sqrtf(2.0f) * sinf(sigma[5]);

  // ?? = 875?2 cos(??)
  sigma[4] = 875.0f * sqrtf(2.0f) * cosf(sigma[5]);

  // ????????
  Jacobian[0][0] = -((250.0f * sinf(leg->theta_fore) * sigma[0] + sigma[3] + 125.0f * sqrtf(2.0f) * sinf(sigma[2])) / (4.0f * sigma[0]));

  // ????????
  Jacobian[0][1] = -((250.0f * sinf(leg->theta_back) * sigma[0] + sigma[3] - 125.0f * sqrtf(2.0f) * sinf(sigma[1])) / (4.0f * sigma[0]));

  // ????????
  Jacobian[1][0] = ((250.0f * cosf(leg->theta_fore) * sigma[0] + sigma[4] + 125.0f * sqrtf(2.0f) * cosf(sigma[2])) / (4.0f * sigma[0]));

  // ????????
  Jacobian[1][1] = ((250.0f * cosf(leg->theta_back) * sigma[0] + sigma[4] + 125.0f * sqrtf(2.0f) * cosf(sigma[1])) / (4.0f * sigma[0]));
}
void vmc_Solve(Leg *leg)
{
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

  leg_Forward_Kinematics(leg);
  leg_Calc_Jacobian(leg, Jacobian);
  matrix_wrap(&J, 2, 2, &Jacobian[0][0]);
  matrix_wrap(&J_trans, 2, 2, &Jacobian_trans[0][0]);
  matrix_wrap(&T, 2, 1, Torque);
  matrix_transpose(&J, &J_trans);
  // 閫氳繃闆呭彲姣旂煩闃佃绠楀綋鍓嶈冻绔€熷�?
  angle_Vel[0] = leg->motor_ctrl_linkf.data.W / 6.33f; // 杩炴潌杞€焤ad/s
  angle_Vel[1] = leg->motor_ctrl_linkb.data.W / 6.33f; // rad/s
  matrix_wrap(&angleVel, 2, 1, angle_Vel);
  matrix_wrap(&f_Vel, 2, 1, leg->fvel);
  matrix_mul(&J, &angleVel, &f_Vel); // mm/s
  // 閫氳繃浣嶇疆锛岄€熷害璇樊璁＄畻瓒崇铏氭嫙鍔?
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
  // 璁＄畻铏氭嫙鍔涗互鍙婇檺�?
  leg->bezier.pid[1].xyPosIntegral[0] += err_x * leg->bezier.pid[1].K_I; // 绉垎锟?
  leg->bezier.pid[1].xyPosIntegral[1] += err_y * leg->bezier.pid[1].K_I;
  leg->Fxy[0] = leg->bezier.pid[1].K_P * err_x + leg->bezier.pid[1].K_W * err_vx + leg->bezier.pid[1].xyPosIntegral[0];
  leg->Fxy[1] = (leg->bezier.pid[1].K_P * VMC_Y_AXIS_STIFFNESS_SCALE) * err_y + leg->bezier.pid[1].K_W * err_vy + leg->bezier.pid[1].xyPosIntegral[1]; // N
  leg->Fxy[0] = vmc_deadzone(leg->Fxy[0], VMC_FORCE_DEADZONE);                                                                                         // 姝诲尯澶勭悊
  leg->Fxy[1] = vmc_deadzone(leg->Fxy[1], VMC_FORCE_DEADZONE);
  force_sat = 0;
  leg->Fxy[0] = vmc_clamp(leg->Fxy[0], VMC_FOOT_FORCE_LIMIT, &force_sat); // 闄愬箙澶勭悊
  leg->Fxy[1] = vmc_clamp(leg->Fxy[1], VMC_FOOT_FORCE_LIMIT, &force_sat);
  if (force_sat)
  {
    g_vmc_force_sat_count[leg_idx]++;
  }
  matrix_wrap(&Fxy, 2, 1, leg->Fxy);
  matrix_mul(&J_trans, &Fxy, &T); // 瓒崇铏氭嫙鍔涜浆鎵煩N*mm
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
// void leg_Bezier_Act_init(Leg *leg,leg_state state){//鍒濆鍖朾ezier鍙傛暟锛屽紑鍚柊鏇茬嚎鐨勮锟?//	leg->state=state;//璁剧疆鑵胯繍鍔ㄧ姸鎬佹洸�?////	leg->bezier.			= &node_prt[leg->state];		//缁撶偣鍧愭爣
//	leg->bezier.config.n				= bezierDimension[leg->state];	//闃舵�?
//	leg->bezier.config.fre				= bezierFre[leg->state];		//鍙栨牱棰戠巼
//	leg->bezier.config.T				= bezierT[leg->state];			//鍔ㄤ綔鍛ㄦ湡
//	leg->bezier.flag			= 0;							//涓€鏉℃洸绾胯绠楀畬鎴愭爣�?//	leg->bezier.t				= 0;							//鏇茬嚎姣斾緥绯绘�?
//	leg->bezier.point_sum		= 0;
//	leg->bezier.config.now_time		= 0;
//	leg->bezier.last_end_time	= 0;
//}
// 涓哄綋鍓嶈吙鐘舵€佽杞借礉濉炲皵杞ㄨ抗鍙傛暟锟?
void leg_Bezier_Free_Init(Leg *leg, leg_state state, BezierExp *bezier_para)
{
  leg->state = state;
  memcpy(&leg->bezier, bezier_para, sizeof(leg->bezier));
}
void leg_Bezier_Free_Inituper(Leg *leg, leg_state state, BezierExp *bezier_para, pid_ *pid_para)
{
  leg->state = state;
  memcpy(&leg->bezier, bezier_para, sizeof(leg->bezier));
  if (pid_para != NULL)
  {
    memcpy(&leg->bezier.pid, pid_para, sizeof(leg->bezier.pid));
  }
}

static void bezier_exp_reset(BezierExp *bezier_para,
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

static float gait_clamp_scale(float scale)
{
  if (isnan(scale) || isinf(scale))
  {
    return 1.0f;
  }
  if (scale < GAIT_SCALE_MIN)
  {
    return GAIT_SCALE_MIN;
  }
  if (scale > GAIT_SCALE_MAX)
  {
    return GAIT_SCALE_MAX;
  }
  return scale;
}

static void gait_scale_ctrl_points(bezierPoint dst[][4], const bezierPoint src[][4], uint8_t stage_count,
                                   float scale_x, float scale_y, float neutral_y, uint8_t mirror_x)
{
  for (uint8_t stage = 0; stage < stage_count; stage++)
  {
    bezierCurve_Scale(dst[stage], src[stage], 4, scale_x, scale_y, neutral_y);
    if (mirror_x)
    {
      for (uint8_t i = 0; i < 4; i++)
      {
        dst[stage][i].x *= -1.0f;
      }
    }
  }
}

static void dog_InitLegBezierBatch(Dog *dog, const leg_state state_by_leg[4], void *const bezier_by_leg[4])
{
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    leg_Bezier_Free_Init(&dog->leg[leg], state_by_leg[leg], (BezierExp *)bezier_by_leg[leg]);
  }
}

static void dog_InitSingleLegBezier(Dog *dog, uint8_t leg_index, leg_state state, BezierExp *bezier_para)
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

static void dog_RunBezierBatch(Dog *dog, void (*act)(Leg *), uint16_t interval)
{
  uint32_t now_time = 0;
  static uint32_t last_end_time = 0;
  while (!dog_IsBezierBatchFinished(dog))
  {
    now_time = osKernelSysTick();
    if (now_time - last_end_time >= interval)
    {
      for (uint8_t leg = 0; leg < 4; leg++)
      {
        act(&dog->leg[leg]);
      }
      for (uint8_t leg = 0; leg < 4; leg++)
      {
        GO_PFC_Ctrl(&(dog->leg[leg].motor_ctrl_linkf), 1);
        GO_PFC_Ctrl(&(dog->leg[leg].motor_ctrl_linkb), 1);
      }
      last_end_time = osKernelSysTick();
    }
    osDelay(1);
  }
}
static void dog_RunSingleLegBezierUntilFinish(Dog *dog, uint8_t leg_index, void (*act)(Leg *))
{
  while (dog->leg[leg_index].bezier.flag == 0)
  {
    act(&dog->leg[leg_index]);
  }
}

void leg_BezierTargetPos_Update(Leg *leg)
{
  leg->bezier.exp_pos = bezierCurve((const bezierPoint(*)[4]) & leg->bezier.config.ctrl_point, leg->bezier.config.n, leg->bezier.t); // 鏇存柊pos
  leg->bezier.t += 1 / (leg->bezier.config.T * leg->bezier.config.fre);                                                              // 鏇存柊姣斾緥绯绘�?
  if (leg->bezier.t > 1)
  {
    // 闄愬畾姣斾緥绯绘暟鑼冨洿
    leg->bezier.t = 1;
  }
  leg->bezier.point_sum++; // 瀵瑰凡绠楀嚭鐐硅�?
}

void leg_Bezier_Act(Leg *leg)
{
  leg->bezier.now_time = osKernelSysTick();
  if (leg->bezier.point_sum < leg->bezier.config.T * leg->bezier.config.fre && (leg->bezier.now_time >= (leg->bezier.last_end_time + (uint32_t)(1000 / leg->bezier.config.fre))))
  {
    leg_BezierTargetPos_Update(leg);
    posCtrlSolve(leg);
    // 鍚姩鐢垫満
    GO_PosMode_Ctrl(&(leg->motor_ctrl_linkf), 0);
    GO_PosMode_Ctrl(&(leg->motor_ctrl_linkb), 0);
    if (leg->bezier.point_sum >= leg->bezier.config.T * leg->bezier.config.fre)
    {
      leg->bezier.flag = 1;
    }
    leg->bezier.last_end_time = osKernelSysTick();
  }
}

void vmc_leg_Bezier_Act(Leg *leg)
{
  leg_BezierTargetPos_Update(leg);
  vmc_Solve(leg);
  // 灏嗘洿鏂板悗鐨勭數鏈烘壄鐭╁彂閫佺粰鐢垫満
  GO_TorqueMode_Ctrl(&(leg->motor_ctrl_linkf), 1);
  GO_TorqueMode_Ctrl(&(leg->motor_ctrl_linkb), 1);
  if (leg->bezier.point_sum >= leg->bezier.config.T * leg->bezier.config.fre)
  {
    leg->bezier.flag = 1;
  }
  leg->bezier.last_end_time = osKernelSysTick();
}

void fpc_Leg_Bezier_Act(Leg *leg)
{
  //  if(leg->bezier.point_sum>0)
  //    {
  //      GO_PFC_Ctrl(&(leg->motor_ctrl_linkf), 1);
  //      GO_PFC_Ctrl(&(leg->motor_ctrl_linkb), 1);
  //    }
  if (leg->bezier.point_sum < leg->bezier.config.T * leg->bezier.config.fre)
  {
    leg_BezierTargetPos_Update(leg);
    posCtrlSolve(leg);
    vmc_Solve(leg);
    if (leg->bezier.point_sum >= leg->bezier.config.T * leg->bezier.config.fre)
    {
      leg->bezier.flag = 1;
    }
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
void standUP_FPC(Dog *dog, float stand_height);
void trotRun_FPC(Dog *dog, float speed_level);
void walk_FPC(Dog *dog);
void Trot_RotateJump_FPC(Dog *dog);

void Dog_Init(Dog *dog)
{
  if (dog == NULL)
  {
    Error_Handler();
  }

  memset(&dog->leg, 0, sizeof(Leg) * 4);
  dog->state = STAND_UP_;
  memset(&dog->location, 0, sizeof(DogLocation)); //
  for (uint8_t i = 0; i < 4; i++)
  {
    dog->leg[i] = (Leg){
        .L1 = 125,
        .L2 = 250,
        .id = i + 1,
        .state = STAND_UP,
        .theta_fore = 0,
        .theta_back = 0};
  }

  for (uint8_t i = 0; i < 4; i++)
  {
    if (GO_init(&dog->leg[i].motor_ctrl_linkf, (dog->leg[i].id) * 2 - 1) != HAL_OK)
    {
      Error_Handler();
    }
    if (GO_init(&dog->leg[i].motor_ctrl_linkb, (dog->leg[i].id) * 2) != HAL_OK)
    {
      Error_Handler();
    }
  }
  //  standUP_FPC(dog, 216.5f);
}
void dogTaskCtrl(Dog *dog)
{
  switch (dog->state)
  {
  case STAND_UP_:
    standUP_FPC(dog, 216.5f);
    //          standUP(dog);
    break;
  case WALK_FORWARD:
    //        walkForward_FPC(dog);
    trotRun_FPC(dog, 1);
    //        walkForward(dog);
    break;
  case TURN_RIGHT:
    Turn(dog);
    break;
  case TURN_LEFT:
    Turn(dog);
    break;
  case LOWWALK_FORWARD:
    standUP_FPC(dog, 180.0f);
    osDelay(2000);
    LowWalkForward(dog);
    break;
  case JUMP_FORWARD:
    JumpForward(dog);
    break;
  case INJUMP:
    InJump(dog);
    break;
  case WALK_BACK:
    trotRun_FPC(dog, 1);
    break;
  case DAMPING_MODE:
    Damping_mode(dog);
    break;
  //    case TROT_ROTATEJUMP:
  //      Trot_RotateJump_FPC(dog);
  //      break;
  default:
    standUP(dog);
    break;
  }
}
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
         {{.K_P = 0.1f, .K_W = 0.02f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {0}}};
  static BezierExp bezier_table[4][1];
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
  static BezierExp bezier_table[4][6];
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
  dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
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
    dog_RunBezierBatch(dog, leg_Bezier_Act, 10);

    stage_state[0] = STEP_FORE2;
    stage_state[1] = KICK_BACK2;
    stage_state[2] = STEP_FORE2;
    stage_state[3] = KICK_BACK2;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
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
void Turn(Dog *dog)
{
  static const float stand_height = 216.5f;
  static const float step1_height = 20.0f;
  static const float step2_height = 40.0f;
  static const float kick1_depth = 20.0f;
  static const float kick2_depth = 20.0f;
  static const float range = 90.0f;

  static const float stage_duration[12] = {0.1f, 0.15f, 0.15f, 0.15f, 0.15f, 0.15f, 0.15f, 0.15f, 0.15f, 0.15f, 0.15f, 0.15f};
  static const float stage_frequency[12] = {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};
  static const uint8_t stage_order[12] = {2, 2, 2, 2, 3, 3, 2, 2, 2, 2, 2, 2};
  static const bezierPoint ctrl_point[12][4] =
      {
          {{0, stand_height}, {-0.25f * range, stand_height - 2.0f * step1_height}, {-0.5f * range, stand_height}, {-100, 190.53f}},                                                  // STEP_B1
          {{0, stand_height}, {0.25f * range, stand_height - 2.0f * step1_height}, {0.5f * range, stand_height}, {100, 190.53f}},                                                     // STEP_F1
          {{0, stand_height}, {0.25f * range, stand_height + 2.0f * kick1_depth}, {0.5f * range, stand_height}, {99, 214}},                                                           // KICK_F1
          {{0, stand_height}, {-0.25f * range, stand_height + 2.0f * kick1_depth}, {-0.5f * range, stand_height}, {99, 214}},                                                         // KICK_B1
          {{0.5f * range, stand_height}, {0.25f * range, stand_height - 1.33f * step2_height}, {-0.25f * range, stand_height - 1.33f * step2_height}, {-0.5f * range, stand_height}}, // STEP_B2
          {{-0.5f * range, stand_height}, {-0.25f * range, stand_height - 1.33f * step2_height}, {0.25f * range, stand_height - 1.33f * step2_height}, {0.5f * range, stand_height}}, // STEP_F2
          {{-0.5f * range, stand_height}, {0, stand_height + 2.0f * kick2_depth}, {0.5f * range, stand_height}, {0.5f * range, 250.0f}},                                              // KICK_F2
          {{0.5f * range, stand_height}, {0, stand_height + 2.0f * kick2_depth}, {-0.5f * range, stand_height}, {-0.5f * range, 250.0f}},                                             // KICK_B2

          {{50.0f, 190.53f}, {25.0f, 200.0f}, {0, 190.53f}, {0, 0}},   // STEP_F_REBACK
          {{-50.0f, 190.53f}, {-25.0f, 200.0f}, {0, 190.53f}, {0, 0}}, // STEP_B_REBACK
          {{50.0f, 190.53f}, {25.0f, 100.0f}, {0, 190.53f}, {0, 0}},   // KICK_F_REBACK
          {{-50.0f, 190.53f}, {-25.0f, 100.0f}, {0, 190.53f}, {0, 0}}  // KICK_B_REBACK
      };
  // PID stage mapping: [0]=STEP_B1, [1]=STEP_F1, [2]=KICK_F1, [3]=KICK_B1, [4]=STEP_B2, [5]=STEP_F2, [6]=KICK_F2, [7]=KICK_B2, [8]=STEP_F_REBACK, [9]=STEP_B_REBACK, [10]=KICK_F_REBACK, [11]=KICK_B_REBACK
  static const pid_ pid[12][2] =
      {
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 0.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 0.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 0.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 0.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 0.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 0.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 0.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 0.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 0.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 0.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 0.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 0.5f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},

      };
  // static const pid_ pid[12][2] =
  //     {
  //         {{.K_P = 0.7f, .K_W = 0.07f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //        {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.7f, .K_W = 0.07f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //        {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.7f, .K_W = 0.07f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //        {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.7f, .K_W = 0.07f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //        {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.7f, .K_W = 0.07f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //        {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.7f, .K_W = 0.07f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //        {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.7f, .K_W = 0.07f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //        {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.7f, .K_W = 0.07f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //        {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.7f, .K_W = 0.07f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //        {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.7f, .K_W = 0.07f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //          {.K_P = 1.4f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.7f, .K_W = 0.07f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //        {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //     };
  static BezierExp bezier_table[4][12];
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
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
    // dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
    osDelay(50);

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
      dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
      // dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
      osDelay(50);

      stage_state[0] = KICK_F2;
      stage_state[1] = STEP_F2;
      stage_state[2] = KICK_B2;
      stage_state[3] = STEP_B2;
      for (uint8_t leg = 0; leg < 4; leg++)
      {
        stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
      }
      dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
      dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
      // dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
      osDelay(50);
    }

    // stage_state[0] = KICK_F_REBACK;
    // stage_state[1] = STEP_F_REBACK;
    // stage_state[2] = KICK_B_REBACK;
    // stage_state[3] = STEP_B_REBACK;
    // for (uint8_t leg = 0; leg < 4; leg++)
    // {
    //   stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    // }
    // dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    // dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
    // osDelay(50);
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
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
    // dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
    osDelay(50);

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
      dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
      // dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
      osDelay(50);
      stage_state[0] = KICK_B2;
      stage_state[1] = STEP_B2;
      stage_state[2] = KICK_F2;
      stage_state[3] = STEP_F2;
      for (uint8_t leg = 0; leg < 4; leg++)
      {
        stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
      }
      dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
      dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
      // dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
      osDelay(50);
    }

    // stage_state[0] = KICK_B_REBACK;
    // stage_state[1] = STEP_B_REBACK;
    // stage_state[2] = KICK_F_REBACK;
    // stage_state[3] = STEP_F_REBACK;
    // for (uint8_t leg = 0; leg < 4; leg++)
    // {
    //   stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    // }
    // dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    // dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
    // osDelay(50);
  }
}
void LowWalkForward(Dog *dog)
{
  static const float stand_height = 180.0f;
  static const float step1_height = 20.0f;
  static const float step2_height = 40.0f;
  static const float kick1_depth = 20.0f;
  static const float kick2_depth = 20.0f;
  static const float range = 80.0f;

  static const float stage_duration[6] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.3f};
  static const float stage_frequency[6] = {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};
  static const uint8_t stage_order[6] = {2, 2, 3, 3, 2, 2};
  static const bezierPoint ctrl_point[6][4] =
      {
          {{0, stand_height}, {0.25f * range, stand_height - 2.0f * step1_height}, {0.5f * range, stand_height}, {100, 300.0f}},                                                      // STEP_FORE1
          {{0, stand_height}, {-0.25f * range, stand_height + 2.0f * kick1_depth}, {-0.5f * range, stand_height}, {99, 214}},                                                         // KICK_BACK1
          {{-0.5f * range, stand_height}, {-0.25f * range, stand_height - 1.33f * step2_height}, {0.25f * range, stand_height - 1.33f * step2_height}, {0.5f * range, stand_height}}, // STEP_FORE2
          {{0.5f * range, stand_height}, {0.25f * range, stand_height + 1.33f * kick2_depth}, {-0.25f * range, stand_height + 1.33f * kick2_depth}, {-0.5f * range, stand_height}},   // KICK_BACK2

          {{0.5f * range, stand_height}, {0.25f * range, stand_height + 2.0f * kick1_depth}, {0, stand_height}, {0, 0}},   // STEP_FORE_REBACK
          {{-0.5f * range, stand_height}, {-0.25f * range, stand_height - 2.0f * step1_height}, {0, stand_height}, {0, 0}} // KICK_BACK_REBACK
      };
  static const pid_ pid_base[6][2] =
      {
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
      };
  // static const pid_ pid_base[6][2] =
  //     {
  //         {{.K_P = 0.75f, .K_W = 0.075f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //          {.K_P = 0.75f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.75f, .K_W = 0.075f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //          {.K_P = 0.75f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.75f, .K_W = 0.075f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //          {.K_P = 0.75f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.75f, .K_W = 0.075f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //          {.K_P = 0.75f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.75f, .K_W = 0.075f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //          {.K_P = 0.75f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
  //         {{.K_P = 0.75f, .K_W = 0.075f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //          {.K_P = 0.75f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}}};
  static const float leg_pid_ratio[6][4][2] =
      {
          {{1.5f, 1.5f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.5f, 1.5f}},
          {{1.5f, 1.5f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.5f, 1.5f}},
          {{1.5f, 1.5f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.5f, 1.5f}},
          {{1.5f, 1.5f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.5f, 1.5f}},
          {{1.5f, 1.5f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.5f, 1.5f}},
          {{1.5f, 1.5f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.5f, 1.5f}}};
  static BezierExp bezier_table[4][6];
  void *stage_bezier[4];
  leg_state stage_state[4];

  float speed_level = 1;
  bezierPoint ctrl_point_scaled[6][4];

  gait_scale_ctrl_points(ctrl_point_scaled, ctrl_point, 6, speed_level, speed_level, stand_height,
                         (dog->state == WALK_BACK) ? 1U : 0U);

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
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
  // dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
  osDelay(1);

  while (dog->state == LOWWALK_FORWARD)
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
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
    // dog_RunBezierBatch(dog, leg_Bezier_Act, 10);

    stage_state[0] = STEP_FORE2;
    stage_state[1] = KICK_BACK2;
    stage_state[2] = STEP_FORE2;
    stage_state[3] = KICK_BACK2;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
    // dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
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
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
}

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
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.5f, .K_W = 0.05f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 5.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.03f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
          {{.K_P = 0.1f, .K_W = 0.03f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
      };
  static const float leg_pid_ratio[4][2] =
      {
          {1.0f, 1.0f},
          {1.0f, 1.0f},
          {1.0f, 1.0f},
          {1.0f, 1.0f},
      };
  static BezierExp bezier_table[4][4];
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
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
  osDelay(1000);

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][JUMPING];
  }
  dog_InitLegBezierBatch(dog,
                         (const leg_state[4]){
                             JUMPING, JUMPING, JUMPING, JUMPING},
                         stage_bezier);
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
  osDelay(200);

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][JUMP_LANDING];
  }
  dog_InitLegBezierBatch(dog,
                         (const leg_state[4]){
                             JUMP_LANDING, JUMP_LANDING, JUMP_LANDING, JUMP_LANDING},
                         stage_bezier);
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
  osDelay(100);

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][JUMP_BUFFER];
  }
  dog_InitLegBezierBatch(dog,
                         (const leg_state[4]){
                             JUMP_BUFFER, JUMP_BUFFER, JUMP_BUFFER, JUMP_BUFFER},
                         stage_bezier);
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
  osDelay(1000);
  dog->state = STAND_UP_;
}

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
  static BezierExp bezier_table[4][3];
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
  dog_RunBezierBatch(dog, leg_Bezier_Act, 10);

  osDelay(300);
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][INJUMP_TAKEOFF];
  }
  dog_InitLegBezierBatch(dog, (const leg_state[4]){INJUMP_TAKEOFF, INJUMP_TAKEOFF, INJUMP_TAKEOFF, INJUMP_TAKEOFF}, stage_bezier);
  dog_RunBezierBatch(dog, leg_Bezier_Act, 10);

  osDelay(500);
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    stage_bezier[leg] = &bezier_table[leg][INJUMP_LANDING];
  }
  dog_InitLegBezierBatch(dog, (const leg_state[4]){INJUMP_LANDING, INJUMP_LANDING, INJUMP_LANDING, INJUMP_LANDING}, stage_bezier);
  dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
  osDelay(1000);
}

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
  static BezierExp bezier_table[4][4];
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
  dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
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
    dog_RunBezierBatch(dog, leg_Bezier_Act, 10);

    stage_state[0] = STEP_FORE2_BACK;
    stage_state[1] = KICK_BACK2_BACK;
    stage_state[2] = STEP_FORE2_BACK;
    stage_state[3] = KICK_BACK2_BACK;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
    osDelay(1);
  }
}

void Damping_mode(Dog *dog)
{
  // PID stage mapping: [0]=position loop, [1]=force/torque loop (unused here)
  static const pid_ pid[2] =
      {
          {.K_P = 0.0f, .K_W = 0.01f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
          {0}};
  static const bezierPoint ctrl_point[4] = {{0.0f, 190.53f}, {0, 0}, {0, 0}, {0, 0}};
  static BezierExp bezier_table[4][1];
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

void standUP_FPC(Dog *dog, float stand_height)
{
  bezierPoint ctrl_point[4] = {{0, stand_height}, {0, 0}, {0, 0}, {0, 0}};

  static const pid_ pid[2] =
      {
          {.K_P = 0.15f, .K_W = 0.030f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
          {.K_P = 1.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}};
  // static const pid_ pid[2] =
  //     {
  //         {.K_P = 0.6f, .K_W = 0.06f, .K_I = 0.01f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
  //         {.K_P = 0.0f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}};
  static BezierExp bezier_table[4][1];
  static const leg_state stage_state[4] = {STAND_UP, STAND_UP, STAND_UP, STAND_UP};
  void *stage_bezier[4];
  static const float leg_pid_ratio[4][2] =
      {
          {1.0f, 1.0f},
          {1.0f, 1.0f},
          {1.0f, 1.0f},
          {1.0f, 1.0f},
      };
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    pid_ leg_pid[2];
    pid_build_scaled(pid, leg_pid_ratio[leg], leg_pid);
    bezier_exp_reset(&bezier_table[leg][STAND_UP], 0.5f, 100.0f, 0, ctrl_point, leg_pid);
    stage_bezier[leg] = &bezier_table[leg][STAND_UP];
  }
  dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
  // dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
}

void trotRun_FPC(Dog *dog, float speed_level)
{
  static const float stand_height = 216.5f;
  static const float step1_height = 30.0f;
  static const float step2_height = 70.0f;
  static const float kick1_depth = 25.0f;
  static const float kick2_depth = 25.0f;
  static const float range = 100.0f;

  static const float stage_duration[6] = {0.15f, 0.20f, 0.15f, 0.20f, 0.15f, 0.15f};
  static const float stage_frequency[6] = {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};
  static const uint8_t stage_order[6] = {2, 2, 3, 3, 2, 2};
  static const bezierPoint ctrl_point[6][4] =
      {
          {{0, stand_height}, {0.25f * range, stand_height - 2.0f * step1_height}, {0.5f * range, stand_height}, {100, 300.0f}},                                                      // STEP_FORE1
          {{0, stand_height}, {-0.25f * range, stand_height + 2.0f * kick1_depth}, {-0.5f * range, stand_height}, {99, 214}},                                                         // KICK_BACK1
          {{-0.5f * range, stand_height}, {-0.3f * range, stand_height - 1.33f * step2_height}, {0.3f * range, stand_height - 1.33f * step2_height}, {0.5f * range, stand_height}}, // STEP_FORE2
          {{0.5f * range, stand_height}, {0.3f * range, stand_height + 1.33f * kick2_depth}, {-0.3f * range, stand_height + 1.33f * kick2_depth}, {-0.5f * range, stand_height}},   // KICK_BACK2

          {{0.5f * range, stand_height}, {0.25f * range, stand_height + 2.0f * kick1_depth}, {0, stand_height}, {0, 0}},   // STEP_FORE_REBACK
          {{-0.5f * range, stand_height}, {-0.25f * range, stand_height - 2.0f * step1_height}, {0, stand_height}, {0, 0}} // KICK_BACK_REBACK
      };
  static const pid_ pid_base[6][2] =
      {
          {{.K_P = 0.1f, .K_W = 0.02f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
         {{.K_P = 0.1f, .K_W = 0.02f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
         {{.K_P = 0.1f, .K_W = 0.02f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
         {{.K_P = 0.1f, .K_W = 0.02f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
         {{.K_P = 0.1f, .K_W = 0.02f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},
         {{.K_P = 0.1f, .K_W = 0.02f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0},
           {.K_P = 1.2f, .K_W = 0.0f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}},

      };
  static const float leg_pid_ratio[6][4][2] =
      {
          {{1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}},
{{1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}},
{{1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}},
{{1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}},
{{1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}},
{{1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 1.0f}},
      };
  static BezierExp bezier_table[4][6];
  void *stage_bezier[4];
  leg_state stage_state[4];
  bezierPoint ctrl_point_scaled[6][4];
  speed_level = gait_clamp_scale(speed_level); // ???????????????

  gait_scale_ctrl_points(ctrl_point_scaled, ctrl_point, 6, speed_level, speed_level, stand_height,
                         (dog->state == WALK_BACK) ? 1U : 0U);

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
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
  // dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
  osDelay(1);

  while (dog->state == WALK_FORWARD || dog->state == WALK_BACK)
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
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
    // dog_RunBezierBatch(dog, leg_Bezier_Act, 10);

    stage_state[0] = STEP_FORE2;
    stage_state[1] = KICK_BACK2;
    stage_state[2] = STEP_FORE2;
    stage_state[3] = KICK_BACK2;
    for (uint8_t leg = 0; leg < 4; leg++)
    {
      stage_bezier[leg] = &bezier_table[leg][stage_state[leg]];
    }
    dog_InitLegBezierBatch(dog, stage_state, stage_bezier);
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
    // dog_RunBezierBatch(dog, leg_Bezier_Act, 10);
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
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
}

void walk_FPC(Dog *dog)
{
  static const float stage_duration[6] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f};
  static const float stage_frequency[6] = {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};
  static const uint8_t stage_order[6] = {2, 2, 3, 3, 2, 2};
  static const bezierPoint ctrl_point[6][4] =
      {
          {{0, 190.53f}, {50.0f, 230.0f}, {100.0f, 242.49f}, {100, 190.53f}}, // STEP_FORE1
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
  static BezierExp bezier_table[4][6];
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
  dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);

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
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
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
    dog_RunBezierBatch(dog, fpc_Leg_Bezier_Act, 10);
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

// void Trot_RotateJump_FPC(Dog *dog)
//{
//   static const float stage_duration[6] = {0.1f, 0.1f, 0.2f, 0.2f, 0.2f, 0.2f};
//   static const float stage_frequency[6] = {10.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};
//   static const uint8_t stage_order[6] = {2, 2, 3, 3, 2, 2};
//   static const bezierPoint ctrl_point[6][4] =
//       {
//           {{0, 190.53f}, {40.0f, 60.0f}, {80.0f, 190.53f}, {100, 190.53f}},   // STEP_FORE1
//           {{0, 190.53f}, {-40.0f, 270.0f}, {-80.0f, 190.53}, {99, 214}},      // KICK_BACK1
//           {{-80.0f, 190.53f}, {-64, 100.0f}, {64, 100.0f}, {80.0f, 190.53f}}, // STEP_FORE2
//           {{80.0f, 190.53f}, {64, 270.0f}, {-64.0f, 270.0f}, {-80, 190.53}},  // KICK_BACK2

//          {{100.0f, 190.53f}, {50.0f, 270.0f}, {0, 190.53f}, {0, 0}},  // STEP_FORE_REBACK
//          {{-100.0f, 190.53f}, {-50.0f, 150.0f}, {0, 190.53f}, {0, 0}} // KICK_BACK_REBACK
//      };
//  // PID stage mapping: [0]=STEP_FORE1, [1]=KICK_BACK1, [2]=STEP_FORE2, [3]=KICK_BACK2, [4]=STEP_FORE_REBACK, [5]=KICK_BACK_REBACK
//  static const pid_ pid_base[6][2] =
//      {
//          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
//          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
//          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
//          {{.K_P = 1.0f, .K_W = 0.1f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
//          {{.K_P = 0.3f, .K_W = 0.03f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
//          {{.K_P = 0.25f, .K_W = 0.025f, .K_I = 0.0f, .xyPosIntegral = {0, 0}, .Pos = 0, .W = 0, .T = 0}, {0}},
//      };
//  static const float leg_pid_ratio[4][2] =
//      {
//          {1.0f, 1.0f},
//          {1.0f, 1.0f},
//          {1.0f, 1.0f},
//          {1.0f, 1.0f},
//      };
//  static BezierExp bezier_table[4][6];
//  void *stage_bezier[4];
//  leg_state stage_state[4];
//}
