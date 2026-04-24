#include "world.h"
#include "cmsis_os.h"
#include "string.h"

void task_switch_reset(TaskTreeContext *ctx)
{
    memset(ctx, 0, sizeof(TaskTreeContext));
}

static void TaskTreeClearTarget(TaskTreeContext *ctx)
{
    ctx->seekbox_ctx.has_target = false;
    ctx->seekbox_ctx.navigate_stage.is_navigating = false;
}

static BtStatus TaskTreeSelectSeekTarget(Dog *dog, BoxGroup (*box_groups)[4], TaskTreeContext *ctx)
{
    float best_score = 3.4e38f;
    uint8_t best_group = 0U;
    uint8_t best_pick = 0U;
    uint8_t found = 0U;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        if ((*box_groups)[i].picked)
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

    ctx->seekbox_ctx.has_target = true;
    ctx->seekbox_ctx.group_index = best_group;
    ctx->seekbox_ctx.pick_box_stage.pick_index = best_pick;
    ctx->seekbox_ctx.pick_box_stage.pick_step = 0U;
    ctx->seekbox_ctx.pick_box_stage.picked_count = 0U;
    return BT_SUCCESS; // 找到目标，进入导航阶段
}

static BtStatus TaskTreePickCurrentBox(Dog *dog, BoxGroup *target_group, TaskTreeContext *ctx)
{
    uint32_t now = osKernelSysTick(); // 当前时间，用于超时处理
    if (!ctx->seekbox_ctx.has_target)
    {
        return BT_FAILURE; // 没有目标，无法拾取
    }
    if (!ctx->seekbox_ctx.navigate_stage.is_navigating)
    {
        dog->plan.type = PICK_UP; // 进入拾取阶段，planning模块将根据当前目标位置和箱体状态进行拾取控制
        ctx->seekbox_ctx.pick_box_stage.pick_start_tick = osKernelSysTick();
        return BT_RUNNING; // 拾取阶段进行中
    }
    else if (ctx->seekbox_ctx.navigate_stage.is_navigating)
    {
        if (dog->plan.finish_flag == true) // 拾取完成，更新状态并进入下一个子任务
        {
            dog->plan.finish_flag = false;                          // 置否，防止重复进入下一个子任务
            ctx->seekbox_ctx.pick_box_stage.picked_count++;         // 成功拾取一个箱子，增加已拾取数量
            if (ctx->seekbox_ctx.pick_box_stage.picked_count >= 2U) // 已经成功拾取两个箱子，认为组内任务完成，进入运输阶段
            {
                target_group->picked = 1U; // 更新箱体状态为已拾取，影响后续目标选择和任务流程
            }
            if (ctx->seekbox_ctx.pick_box_stage.pick_step == 0U) // 成功拾取第一个箱子，准备进入第二个箱子拾取阶段
            {
                ctx->seekbox_ctx.pick_box_stage.pick_step = 1U;        // 进入第二个箱子拾取阶段
                ctx->subtask = SEEK_NAVIGATE_TO_TARGET;                // 进入导航阶段，准备导航到第二个箱子位置
                ctx->seekbox_ctx.navigate_stage.is_navigating = false; // 重置任务运行状态
                return BT_RUNNING;
            }
            if (ctx->seekbox_ctx.pick_box_stage.pick_step == 1U) // 已经完成第二个箱子的拾取，准备进入第二个箱子拾取阶段
            {
                ctx->seekbox_ctx.navigate_stage.is_navigating = false; // 重置任务运行状态，准备进入下一个子任务
                return BT_SUCCESS;
            }
        }
        // 超时处理
    }
    return BT_FAILURE; // 拾取阶段进行中
}
static BtStatus TaskTreeNavigateSeekTarget(Dog *dog, BoxGroup (*box_groups)[4], TaskTreeContext *ctx)
{
    uint32_t now = osKernelSysTick();

    if (!ctx->seekbox_ctx.has_target)
    {
        TaskTreeClearTarget(ctx);          // 清除当前目标状态，准备进入下一个子任务
        ctx->subtask = SEEK_SELECT_TARGET; // 没有目标，回到选择目标阶段
        return BT_FAILURE;                 // 没有目标，无法导航
    }
    // 组内超时：若已拾取至少1个箱子，直接进入运输阶段
    if (ctx->seekbox_ctx.navigate_stage.is_navigating && ((now - ctx->seekbox_ctx.navigate_stage.navigate_start_tick) > 100000U)) // 正在导航过程中，持续检查是否超时
    {
        if (ctx->seekbox_ctx.pick_box_stage.picked_count > 0U) // 已经成功拾取至少1个箱子，认为组内任务完成，进入运输阶段
        {
            ctx->seekbox_ctx.navigate_stage.is_navigating = false; // 导航结束，重置导航状态，进入下一个子任务
            ctx->mission = DELIVER_BOX;                            // 直接进入交付阶段，跳过后续的导航和拾取子任务
            
            return BT_RUNNING;
        }
        return BT_FAILURE;
    }
    else if (!ctx->seekbox_ctx.navigate_stage.is_navigating) // 有目标，但还未开始导航，先设置目标位置并进入导航状态
    {
        if (ctx->seekbox_ctx.pick_box_stage.pick_step == 0U) // 如果是第一个箱子位置
        {
            dog->target_location.pos = (*box_groups)[ctx->seekbox_ctx.group_index].pick_pos[ctx->seekbox_ctx.pick_box_stage.pick_index][0]; // 设置目标位置为当前目标箱子位置，planning模块将根据这个目标位置进行路径规划和运动控制
            dog->plan.type = POINT_TO_POINT;                                                                                                // 改变plan类型，进入规划阶段，后续由planning模块根据目标位置规划路径并控制运动
            ctx->seekbox_ctx.navigate_stage.is_navigating = true;
            ctx->seekbox_ctx.navigate_stage.navigate_start_tick = osKernelSysTick(); // 更新记录导航开始时间，影响后续超时处理
            return BT_RUNNING;
        }
        else if (ctx->seekbox_ctx.pick_box_stage.pick_step == 1U) // 如果是第二个箱子位置
        {
            dog->target_location.pos = (*box_groups)[ctx->seekbox_ctx.group_index].pick_pos[ctx->seekbox_ctx.pick_box_stage.pick_index][1]; // 设置目标位置为当前目标箱子位置，planning模块将根据这个目标位置进行路径规划和运动控制
            dog->plan.type = POINT_TO_POINT;
            ctx->seekbox_ctx.navigate_stage.is_navigating = true;
            ctx->seekbox_ctx.navigate_stage.navigate_start_tick = osKernelSysTick(); // 更新记录导航开始时间，影响后续超时处理
            return BT_RUNNING;
        }
    }
    else if (ctx->seekbox_ctx.navigate_stage.is_navigating) // 已经在导航中，持续检查导航状态
    {
        if (dog->plan.finish_flag == true && dog->plan.type == POINT_TO_POINT) // 导航已完成，返回成功状态，进入下一个子任务
        {
            dog->plan.finish_flag = false;                       // 置否，防止重复进入下一个子任务
            dog->plan.type = IDLE_;                              // 导航完成，重置plan类型，准备进入下一个子任务
            if (ctx->seekbox_ctx.pick_box_stage.pick_step == 0U) // 成功导航到第一个箱子位置，准备进入第一个箱子拾取阶段
            {
                return BT_SUCCESS;
            }
            else if (ctx->seekbox_ctx.pick_box_stage.pick_step == 1U) // 成功导航到第二个箱子位置，准备进入第二个箱子拾取阶段
            {
                return BT_SUCCESS;
            }
        }
        return BT_RUNNING;
    }
    return BT_FAILURE; // 持续导航中
}

static BtStatus SeekBoxTreeTick(Dog *dog, BoxGroup (*box_groups)[4], TaskTreeContext *ctx)
{
    BtStatus status;
    switch (ctx->subtask)
    {
    case SEEK_SELECT_TARGET:
        // Implementation for selecting target
        status = TaskTreeSelectSeekTarget(dog, box_groups, ctx); // 是否成功选择目标
        if (status == BT_SUCCESS)
        {
            ctx->subtask = SEEK_NAVIGATE_TO_TARGET; // 选择目标成功，进入导航阶段
            return BT_RUNNING;
        }
        else if (status == BT_FAILURE)
        {
            return BT_FAILURE; // 没有目标可选，整个寻箱阶段失败，重置任务树准备重新选择目标
        }
        break;
    case SEEK_NAVIGATE_TO_TARGET:
        // Implementation for navigating to target
        status = TaskTreeNavigateSeekTarget(dog, box_groups, ctx); // 是否成功导航到目标位置
        if (status == BT_RUNNING)
        {
            return BT_RUNNING;
        }
        else if (status == BT_SUCCESS)
        {
            ctx->subtask = SEEK_PICK_BOX;                          // 导航成功，进入拾取阶段
            ctx->seekbox_ctx.navigate_stage.is_navigating = false; // 重置运行状态
            return BT_RUNNING;
        }
        else if (status == BT_FAILURE)
        {
            ctx->fail_count[ctx->seekbox_ctx.group_index][ctx->seekbox_ctx.pick_box_stage.pick_index]++; // 导航失败，增加失败次数，影响后续目标选择
            ctx->seekbox_ctx.navigate_stage.is_navigating = false;                                       // 重置导航状态
            return BT_FAILURE;
        }
        break;
    case SEEK_PICK_BOX:
        // Implementation for picking box
        status = TaskTreePickCurrentBox(dog, &(*box_groups)[ctx->seekbox_ctx.group_index], ctx);
        if (status == BT_SUCCESS)
        {
            if (ctx->seekbox_ctx.pick_box_stage.picked_count == 2U)
            {
                ctx->seekbox_ctx.navigate_stage.is_navigating = false;
                return BT_SUCCESS;
            }
            else
            {
                ctx->subtask = SEEK_NAVIGATE_TO_TARGET;                // 成功拾取当前箱子，准备进入下一个箱子导航阶段
                ctx->seekbox_ctx.navigate_stage.is_navigating = false; // 重置运行状态，准备进入下一个子任务
                return BT_RUNNING;
            }
        }
        else if (status == BT_RUNNING)
        {
            return BT_RUNNING;
        }
        else if (status == BT_FAILURE)
        {
            ctx->fail_count[ctx->seekbox_ctx.group_index][ctx->seekbox_ctx.pick_box_stage.pick_index]++; // 导航失败，增加失败次数，影响后续目标选择
            ctx->seekbox_ctx.navigate_stage.is_navigating = false;                                       // 重置导航状态
            return BT_FAILURE;
        }
        break;
    }
    return BT_FAILURE; // 默认返回失败，实际情况应该根据具体实现调整
}

static BtStatus DeliverTreeTick(Dog *dog, ReturnField (*return_field)[4], TaskTreeContext *ctx)
{
    (void)return_field;
    (void)ctx;
    dog->g_task_ctx.mission = DELIVER_BOX;
    // 预留：根据颜色选择 return_field 并规划路径
    return BT_SUCCESS;
}

static BtStatus RootTaskTreeTick(Dog *dog, BoxGroup (*box_groups)[4], ReturnField (*return_field)[4], TaskTreeContext *ctx)
{
    if (dog->g_task_ctx.mission == SEEK_BOX) // 寻箱阶段
    {
        BtStatus seek_status = SeekBoxTreeTick(dog, box_groups, ctx);
        if (seek_status == BT_RUNNING)
        {
            return BT_RUNNING; // 寻箱阶段进行中
        }
        else if (seek_status == BT_SUCCESS) // SEEKBOX + PICKUPBOX
        {
            dog->g_task_ctx.mission = DELIVER_BOX;
        }
        else if (seek_status == BT_FAILURE)
        {
            return BT_FAILURE;
        }
    }

    else if (dog->g_task_ctx.mission == DELIVER_BOX) // 交付阶段
    {
        // BtStatus deliver_status = DeliverTreeTick(dog, return_field, ctx);
        //        if(deliver_status ==BT_RUNNING)
        //        {
        //            return BT_RUNNING; // 交付阶段进行中
        //        }
        // else if (deliver_status == BT_SUCCESS) // DELIVERBOX
        // {
        //     task_switch_reset(); // 交付完成，重置任务树准备下一轮任务
        //     return BT_SUCCESS;
        // }
        // else if (deliver_status == BT_FAILURE)
        // {
        //     return BT_FAILURE;
        // }
    }
    dog->g_task_ctx.mission = SEEK_BOX;
    return BT_RUNNING;
}

void task_switch(Dog *dog, BoxGroup (*box_groups)[4], ReturnField (*return_field)[4])
{
    if ((dog == NULL) || (box_groups == NULL))
    {
        return;
    }

    if (dog->g_task_ctx.mission == IDLE) // 如果当前是空闲状态，进入任务树从seek_box阶段开始执行
    {
        task_switch_reset(&dog->g_task_ctx);
        dog->g_task_ctx.mission = SEEK_BOX;
    }

    if (RootTaskTreeTick(dog, box_groups, return_field, &dog->g_task_ctx) == BT_FAILURE)
    {
        dog->g_task_ctx.mission = IDLE; //
        dog->plan.type = IDLE_;
        task_switch_reset(&dog->g_task_ctx); // 某一环节失败，重置任务树
    }
}