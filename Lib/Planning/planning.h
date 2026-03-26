#ifndef PLANNING_H
#define PLANNING_H
#include <stdbool.h>

typedef struct Dog Dog;

typedef enum plan_type_t
{
    IDLE_,
    POINT_TO_POINT,
    PICK_UP_BOX,
    DROP_OFF_BOX
} plan_type_t;

typedef struct plan_t
{
    plan_type_t type;
    bool finish_flag;
} plan_t;

void dog_Planning(Dog *dog);
TASK_COMPETITION
#endif // PLANNING_H
