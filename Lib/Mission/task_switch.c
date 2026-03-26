#include "world.h"
#include "cmsis_os.h"

void task_switch(Dog *dog, BoxGroup (*box_groups)[4], ReturnField *return_field)
{
    (void)return_field;

    if (dog == 0 || box_groups == 0)
    {
        return;
    }

    switch (dog->mission)
    {
    case SEEK_BOX:
    {
        // 找到最近且未归位组的最近 pick 位
        Coordinates nearest_pick_pos = {0.0f, 0.0f};
        float nearest_distance_sq = 3.4e38f;

        for (uint8_t i = 0; i < 4; i++)
        {
            if ((*box_groups)[i].return_status)
            {
                continue;
            }

            for (uint8_t p = 0; p < 2; p++)
            {
                float dx = (*box_groups)[i].pick_pos[p].x - dog->location.pos.x;
                float dy = (*box_groups)[i].pick_pos[p].y - dog->location.pos.y;
                float dist_sq = dx * dx + dy * dy;

                if (dist_sq < nearest_distance_sq)
                {
                    nearest_distance_sq = dist_sq;
                    nearest_pick_pos = (*box_groups)[i].pick_pos[p];
                }
            }
        }

        // 将最近的 pick 位作为目标点并切到点到点规划
        dog->target_location.pos = nearest_pick_pos;
        dog->plan.type = POINT_TO_POINT;
        dog->plan.finish_flag = false;
        osDelay(100);
        break;
    }

    case PICK_UP_BOX:
        // 预留：驱动吸盘臂吸取 box
        break;

    case DELIVER_BOX:
        // 预留：根据颜色选择 return_field 并规划路径
        break;

    case RETURN_BOX:
    case PICK_DOWN_BOX:
    case IDLE:
    default:
        break;
    }
}
