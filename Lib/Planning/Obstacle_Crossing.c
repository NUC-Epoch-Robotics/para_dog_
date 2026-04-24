#include "Obstacle_Crossing.h"

#include <math.h>
#include <string.h>
#include "dog.h"
#include "stm32f4xx_hal.h"

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

#define RAD2DEG (180.0f / PI_F)
#define DEG2RAD (PI_F / 180.0f)
// 控制器内部目标点（单位与 DogLocation 保持一致）
typedef struct
{
    float x;
    float y;
} TargetPoint_t;

typedef struct
{
    ActionState_t state;
    Dog *dog;
    TargetPoint_t target;
    float current_x;
    float current_y;
    float current_angle;
    float target_angle;
    float distance_threshold;
    float angle_threshold;
} ActionContext_t;

ActionContext_t g_action_ctx = {
    .state = ACTION_STATE_IDLE,
    .dog = NULL,
    .target = {0.0f, 0.0f},
    .current_x = 0.0f,
    .current_y = 0.0f,
    .current_angle = 0.0f,
    .target_angle = 0.0f,
    .distance_threshold = 50.0f,
    .angle_threshold = 5.0f,
};

static bool g_ctx_initialized = false;

static float CalculateDistance(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

static float NormalizeAngle(float angle)
{
    while (angle > PI_F)
    {
        angle -= 2 * PI_F;
    }
    while (angle < -PI_F)
    {
        angle += 2 * PI_F;
    }
    return angle;
}

static float CalculateTargetAngle(float current_x, float current_y, float target_x, float target_y)
{
    float dx = target_x - current_x;
    float dy = target_y - current_y;
    return atan2f(dy, dx);
}

static float CalculateAngleDiff(float current_angle, float target_angle)
{
    return (target_angle - current_angle);
}

static void ActionControl_SyncPoseFromDog(void)
{
    if (g_action_ctx.dog == NULL)
    {
        return;
    }

    g_action_ctx.current_x = g_action_ctx.dog->location.pos.x;
    g_action_ctx.current_y = g_action_ctx.dog->location.pos.y;
    g_action_ctx.current_angle = g_action_ctx.dog->location.yaw * DEG2RAD;
}

void Motion_StandUp(void)
{
    if (g_action_ctx.dog == NULL)
    {
        return;
    }

    g_action_ctx.dog->state = STAND_UP_;
}

void Motion_MoveForward(float distance_mm)
{
    g_action_ctx.dog->state = WALK_FORWARD;
}

void Motion_MoveBackward(float distance_mm)
{
    g_action_ctx.dog->state = WALK_BACK;
}

void Motion_RotateLeft(void)
{
    g_action_ctx.dog->state = TURN_LEFT;
}

void Motion_RotateRight(void)
{
    g_action_ctx.dog->state = TURN_RIGHT;
}

void Motion_Stop(void)
{
    if (g_action_ctx.dog != NULL)
    {
        g_action_ctx.dog->state = STAND_UP_;
        g_action_ctx.dog->plan.type = IDLE_;
    }
}

void Motion_SetSpeed(float speed_level)
{
    if (speed_level < 0.0f)
    {
        speed_level = 0.0f;
    }
    else if (speed_level > 1.0f)
    {
        speed_level = 1.0f;
    }
}

void ActionControl_Init(void)
{
    ActionContext_t reset_ctx;

    memset(&reset_ctx, 0, sizeof(reset_ctx));
    reset_ctx.state = ACTION_STATE_IDLE;
    reset_ctx.distance_threshold = 0.3f;
    reset_ctx.angle_threshold = 0.15f;

    g_action_ctx = reset_ctx;
    g_ctx_initialized = true;
}

void ActionControl_InitWithDog(Dog *dog)
{
    ActionControl_Init();
    g_action_ctx.dog = dog;
}

uint8_t ActionControl_GetPosition(float *x, float *y)
{
    if ((!g_ctx_initialized) || (x == NULL) || (y == NULL) || (g_action_ctx.dog == NULL))
    {
        return 0U;
    }

    *x = g_action_ctx.dog->location.pos.x;
    *y = g_action_ctx.dog->location.pos.y;
    return 1U;
}

float ActionControl_GetAngle(void)
{
    if ((!g_ctx_initialized) || (g_action_ctx.dog == NULL))
    {
        return 0.0f;
    }

    return NormalizeAngle(g_action_ctx.dog->location.yaw * RAD2DEG);
}

bool ActionControl_SetTarget(float target_x, float target_y)
{
    if (!g_ctx_initialized) // 判断是否初始化
    {
        return false;
    }

    g_action_ctx.target.x = target_x;
    g_action_ctx.target.y = target_y;
    g_action_ctx.state = ACTION_STATE_ROTATE;

    if (g_action_ctx.dog != NULL)
    {
        g_action_ctx.dog->plan.type = POINT_TO_POINT;
        g_action_ctx.dog->plan.finish_flag = false;
    }
    return true;
}
float distance_to_target;
float angle_diff;
void ActionControl_Update(void)
{
    if ((!g_ctx_initialized) || (g_action_ctx.dog == NULL))
    {
        return;
    }
    ActionControl_SyncPoseFromDog();
    distance_to_target = CalculateDistance(g_action_ctx.current_x, g_action_ctx.current_y,
                                           g_action_ctx.target.x, g_action_ctx.target.y);
    g_action_ctx.target_angle = CalculateTargetAngle(g_action_ctx.current_x, g_action_ctx.current_y,
                                                     g_action_ctx.target.x, g_action_ctx.target.y);
    angle_diff = CalculateAngleDiff(g_action_ctx.current_angle, g_action_ctx.target_angle);

    switch (g_action_ctx.state)
    {
    case ACTION_STATE_ROTATE:
        if (fabsf(angle_diff) <= g_action_ctx.angle_threshold)
        {
            g_action_ctx.state = ACTION_STATE_MOVE;
            g_action_ctx.dog->state = STAND_UP_;// 先停止旋转，准备进入移动状态
        }
        else
        {
            if (angle_diff > 0.0f)
            {
                if (angle_diff > PI_F)
                {
                    Motion_RotateRight();
                }
                else
                {
                    Motion_RotateLeft();
                }
            }
            else if (angle_diff < 0.0f)
            {
                if (angle_diff < -PI_F)
                {
                    Motion_RotateLeft();
                }
                else
                {
                    Motion_RotateRight();
                }
            }
            else
            {
                Motion_RotateLeft();
            }
        }
        break;

    case ACTION_STATE_MOVE:
        if (distance_to_target <= g_action_ctx.distance_threshold)
        {
            g_action_ctx.dog->plan.finish_flag = true;
            g_action_ctx.dog->state = STAND_UP_;// 到达目标点，停止运动并标记完成
        }
        else if (fabsf(angle_diff) > (g_action_ctx.angle_threshold * 1.5f))// 如果在移动过程中偏离目标方向过多，先调整方向
        {
            g_action_ctx.state = ACTION_STATE_ROTATE;// 重新进入旋转调整状态，优先纠正方向
        }
        else
        {
            Motion_MoveForward(distance_to_target);
        }
        break;
    default:
        g_action_ctx.state = ACTION_STATE_IDLE;
        Motion_Stop();
        break;
    }
}

uint8_t ActionControl_IsArrived(void)
{
    if (!g_ctx_initialized)
    {
        return 0U;
    }

    return (g_action_ctx.state == ACTION_STATE_ARRIVED) ? 1U : 0U;
}

ActionState_t ActionControl_GetState(void)
{
    return g_action_ctx.state;
}

void ActionControl_Reset(void)
{
    g_action_ctx.state = ACTION_STATE_IDLE;
    g_action_ctx.target.x = 0.0f;
    g_action_ctx.target.y = 0.0f;

    if (g_action_ctx.dog != NULL)
    {
        g_action_ctx.dog->plan.type = IDLE_;
        g_action_ctx.dog->plan.finish_flag = false;
        g_action_ctx.dog->state = STAND_UP_;
    }
}
