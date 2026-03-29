#include "world.h"
#include "cmsis_os.h"

typedef enum BtStatus
{
    BT_FAILURE = 0,
    BT_SUCCESS = 1,
    BT_RUNNING = 2
} BtStatus;

typedef struct TaskTreeContext
{
    uint8_t phase;
    uint8_t has_target;
    uint8_t group_index;
    uint8_t pick_index;
    uint8_t pick_step;
    uint8_t picked_count;
    uint8_t fail_count[4][2];
    uint32_t navigate_start_tick;
    uint32_t group_start_tick;
} TaskTreeContext;

static TaskTreeContext g_task_tree_ctx;

static void TaskTreeClearTarget(TaskTreeContext *ctx)
{
    ctx->has_target = 0U;
    ctx->group_index = 0U;
    ctx->pick_index = 0U;
    ctx->pick_step = 0U;
    ctx->picked_count = 0U;
    ctx->navigate_start_tick = 0U;
    ctx->group_start_tick = 0U;
}

static uint8_t TaskTreeGetBoxIndexByDirectionStep(uint8_t direction, uint8_t step)
{
    if (direction == BOX_PICK_FRONT_TO_BACK)
    {
        return step;
    }
    return (uint8_t)(1U - step);
}

void task_switch_reset(void)
{
    g_task_tree_ctx.phase = 0U;
    for (uint8_t i = 0; i < 4U; i++)
    {
        for (uint8_t p = 0; p < 2U; p++)
        {
            g_task_tree_ctx.fail_count[i][p] = 0U;
        }
    }
    TaskTreeClearTarget(&g_task_tree_ctx);
}

static BtStatus TaskTreeSelectSeekTarget(Dog *dog, BoxGroup (*box_groups)[4], TaskTreeContext *ctx)
{
    float best_score = 3.4e38f;
    uint8_t best_group = 0U;
    uint8_t best_pick = 0U;
    uint8_t found = 0U;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        if ((*box_groups)[i].return_status)
        {
            continue;
        }

        for (uint8_t p = 0U; p < 2U; p++)
        {
            float dx = (*box_groups)[i].pick_pos[p][0].x - dog->location.pos.x;
            float dy = (*box_groups)[i].pick_pos[p][0].y - dog->location.pos.y;
            float dist_sq = dx * dx + dy * dy;
            float fail_penalty = (float)ctx->fail_count[i][p] * 400000.0f;
            float score = dist_sq + fail_penalty;

            if (score < best_score) // 综合考虑距离和失败次数选择目标
            {
                best_score = score;
                best_group = i;
                best_pick = p;
                found = 1U;
            }
        }
    }

    if (!found)
    {
        return BT_FAILURE;
    }

    ctx->has_target = 1U;
    ctx->group_index = best_group;
    ctx->pick_index = best_pick;
    ctx->pick_step = 0U;
    ctx->picked_count = 0U;
    ctx->navigate_start_tick = osKernelSysTick();
    ctx->group_start_tick = ctx->navigate_start_tick;

    dog->target_location.pos = (*box_groups)[best_group].pick_pos[best_pick][0];
    dog->plan.type = POINT_TO_POINT;
    dog->plan.finish_flag = false;

    return BT_SUCCESS;
}

static BtStatus TaskTreePickCurrentBox(Dog *dog, BoxGroup *target_group, TaskTreeContext *ctx)
{
    uint8_t box_index = TaskTreeGetBoxIndexByDirectionStep(ctx->pick_index, ctx->pick_step);

    if (!target_group->boxes[box_index].return_status)
    {
        target_group->boxes[box_index].return_status = true;
        if (ctx->picked_count < 2U)
        {
            ctx->picked_count++;
        }
    }

    dog->mission = PICK_UP_BOX;

    target_group->return_status = target_group->boxes[0].return_status &&
                                  target_group->boxes[1].return_status;

    if (target_group->return_status)
    {
        return BT_SUCCESS;
    }

    return BT_RUNNING;
}

static BtStatus TaskTreeNavigateSeekTarget(Dog *dog, BoxGroup (*box_groups)[4], TaskTreeContext *ctx)
{
    BoxGroup *target_group;
    uint32_t now = osKernelSysTick();
    BtStatus pick_status;

    if (!ctx->has_target)
    {
        return BT_FAILURE;
    }

    target_group = &(*box_groups)[ctx->group_index];

    // 组内超时：若已拾取至少1个箱子，直接进入运输阶段
    if ((now - ctx->group_start_tick) > 20000U)
    {
        if (ctx->picked_count > 0U)
        {
            target_group->return_status = target_group->boxes[0].return_status &&
                                          target_group->boxes[1].return_status;
            return BT_SUCCESS;
        }

        if (ctx->fail_count[ctx->group_index][ctx->pick_index] < 10U)
        {
            ctx->fail_count[ctx->group_index][ctx->pick_index]++;
        }
        TaskTreeClearTarget(ctx);
        dog->plan.type = IDLE_;
        return BT_RUNNING;
    }

    if (dog->plan.finish_flag) // 到达当前箱子位置后立即执行拾取
    {
        pick_status = TaskTreePickCurrentBox(dog, target_group, ctx);
        if (pick_status == BT_SUCCESS)
        {
            return BT_SUCCESS;
        }

        if (ctx->pick_step == 0U)
        {
            ctx->pick_step = 1U;
            dog->target_location.pos = target_group->pick_pos[ctx->pick_index][1];
            dog->plan.type = POINT_TO_POINT;
            dog->plan.finish_flag = false;
            ctx->navigate_start_tick = now;
            return BT_RUNNING;
        }

        return (ctx->picked_count > 0U) ? BT_SUCCESS : BT_RUNNING;
    }

    return BT_RUNNING;
}

static BtStatus SeekBoxTreeTick(Dog *dog, BoxGroup (*box_groups)[4], TaskTreeContext *ctx)
{
    BtStatus status;

    if (ctx->has_target == 0U)
    {
        status = TaskTreeSelectSeekTarget(dog, box_groups, ctx);
        if (status == BT_FAILURE)
        {
            return BT_FAILURE;
        }
    }

    status = TaskTreeNavigateSeekTarget(dog, box_groups, ctx);
    if (status == BT_SUCCESS)
    {
        return BT_SUCCESS;
    }
    if (status == BT_FAILURE)
    {
        return BT_FAILURE;
    }

    return BT_RUNNING;
}

static BtStatus DeliverTreeTick(Dog *dog, ReturnField (*return_field)[4], TaskTreeContext *ctx)
{
    (void)return_field;
    (void)ctx;
    dog->mission = DELIVER_BOX;
    // 预留：根据颜色选择 return_field 并规划路径
    return BT_SUCCESS;
}

static BtStatus RootTaskTreeTick(Dog *dog, BoxGroup (*box_groups)[4], ReturnField (*return_field)[4], TaskTreeContext *ctx)
{
    if (ctx->phase == 0U)
    {
        dog->mission = SEEK_BOX;
        if (SeekBoxTreeTick(dog, box_groups, ctx) == BT_SUCCESS) // SEEKBOX + PICKUPBOX
        {
            TaskTreeClearTarget(ctx);
            ctx->phase = 1U;
        }
        return BT_RUNNING;
    }

    if (ctx->phase == 1U)
    {
        dog->mission = DELIVER_BOX;
        if (DeliverTreeTick(dog, return_field, ctx) == BT_SUCCESS) // DELIVERBOX
        {
            ctx->phase = 0U;
            dog->mission = SEEK_BOX;
        }
        return BT_RUNNING;
    }

    ctx->phase = 0U;
    dog->mission = SEEK_BOX;
    return BT_RUNNING;
}

void task_switch(Dog *dog, BoxGroup (*box_groups)[4], ReturnField (*return_field)[4])
{
    if ((dog == NULL) || (box_groups == NULL))
    {
        return;
    }

    if (dog->mission == IDLE)
    {
        task_switch_reset();
        dog->mission = SEEK_BOX;
    }

    if (RootTaskTreeTick(dog, box_groups, return_field, &g_task_tree_ctx) == BT_FAILURE)
    {
        dog->mission = IDLE;
        task_switch_reset();
    }
}
