#include "planning.h"
#include "dog.h"
#include <math.h>
#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

typedef enum PLAN_STATUS
{
    PLAN_FAILURE = 0,
    PLAN_SUCCESS = 1,
    PLAN_RUNNING = 2
} PLAN_STATUS;

static void DogExecutePointToPointPlan(Dog *dog)
{
    static bool have_target = false;
    if (!have_target) // 检查目标设置与否
    {
        if (ActionControl_SetTarget(dog->target_location.pos.x, dog->target_location.pos.y))
        {
            have_target = true; // 目标设置成功
        }
        else
        {
            return; // 目标设置失败，跳过本次更新
        }
    }
    // 执行行动控制更新
    ActionControl_Update();
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
    case PICK_UP:
        break;
    case DROP_OFF:
        break;
    default:
        dog->state = STAND_UP_;
        break;
    }
}
