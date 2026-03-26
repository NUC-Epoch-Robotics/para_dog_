#include "planning.h"
#include "dog.h"
#include <math.h>
#include "cmsis_os.h"
#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

static float NormalizeAngleRad(float angle)
{
    const float two_pi = 2.0f * PI_F;

    while (angle >= PI_F)
    {
        angle -= two_pi;
    }
    while (angle < -PI_F)
    {
        angle += two_pi;
    }

    return angle;
}

static void DogAdjustHeadingTowardTarget(Dog *dog, float yaw_deadzone_rad)
{

    float vector[2] = {0, 0};
    float vector_angle = 0;
    float dvector_angle = 0;
    float dyaw = 0;
    uint32_t adjust_begin_time = osKernelSysTick();
    // 计算目标方向与当前朝向的角度差，并按最短路径调整
    vector[0] = dog->target_location.pos.x - dog->location.pos.x;
    vector[1] = dog->target_location.pos.y - dog->location.pos.y;
    vector_angle = atan2(vector[1], vector[0]); // -pi -> pi

    dvector_angle = vector_angle - NormalizeAngleRad(dog->location.yaw);
    if (fabsf(dvector_angle) > PI_F)
    {
        if (dvector_angle > 0)
        {
            dvector_angle -= 2.0f * PI_F;
        }
        else
        {
            dvector_angle += 2.0f * PI_F;
        }
    }
    dog->target_location.yaw = dog->location.yaw + dvector_angle;
    dyaw = dog->target_location.yaw - dog->location.yaw;
    while (fabsf(dyaw) > yaw_deadzone_rad && ((osKernelSysTick() - adjust_begin_time) < 5000)) // 5秒调整时间限制
    {
        if (dyaw > 0)
        {
            dog->state = TURN_LEFT;
        }
        else
        {
            dog->state = TURN_RIGHT;
        }
        dyaw = dog->target_location.yaw - dog->location.yaw;
        osDelay(100);
    }
}

static void DogAdjustDistanceTowardTarget(Dog *dog)
{
    float distance = sqrtf(powf(dog->target_location.pos.x - dog->location.pos.x, 2) +
                           powf(dog->target_location.pos.y - dog->location.pos.y, 2));

    if (distance > 50.0f && distance < 500.0f)
    {
        dog->state = WALK_FORWARD;
    }
    else if (distance > 500.0f)
    {
        dog->state = WALK_FORWARD;
    }
    else if (distance < 50.0f)
    {
        dog->state = STAND_UP_;
        dog->plan.finish_flag = true;
    }

    osDelay(100);
}

static void DogExecutePointToPointPlan(Dog *dog)
{
    while (dog->plan.finish_flag == false)
    {
        DogAdjustHeadingTowardTarget(dog, 0.1f); // 0.1f is an example value for the yaw deadzone
        DogAdjustDistanceTowardTarget(dog);
    }
}

void dog_Planning(Dog *dog)
{
    switch (dog->plan.type)
    {
    case IDLE_:
        // 保持原地不动
        dog->state = STAND_UP_;
        break;
    case POINT_TO_POINT:
        DogExecutePointToPointPlan(dog);
        break;
    }
}
