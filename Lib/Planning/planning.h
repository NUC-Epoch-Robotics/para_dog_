#ifndef PLANNING_H
#define PLANNING_H
#include <stdbool.h>
#include "Obstacle_Crossing.h"
typedef struct Dog Dog;

typedef enum plan_type_t
{
    IDLE_,
    POINT_TO_POINT,
    PICK_UP,
    DROP_OFF
} plan_type_t;

typedef struct PlanContext
{
    plan_type_t type;
    bool finish_flag;
} PlanContext;
#endif // PLANNING_H
