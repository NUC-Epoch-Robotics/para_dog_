#include "world.h"
#include <string.h>

static void box_group_init(BoxGroup (*box_groups)[4])
{
    static const Coordinates box_initpos[4][2] = {
        {{100.0f, 100.0f}, {700.0f, 300.0f}},
        {{300.0f, 100.0f}, {500.0f, 300.0f}},
        {{500.0f, 100.0f}, {300.0f, 300.0f}},
        {{700.0f, 100.0f}, {100.0f, 300.0f}}};
    static const Coordinates pick_template[2] = {
        {100.0f, 100.0f},
        {700.0f, 300.0f}};

    for (uint8_t i = 0; i < 4; i++)
    {
        (*box_groups)[i].group_id = i;
        (*box_groups)[i].return_status = false;

        for (uint8_t j = 0; j < 2; j++)
        {
            (*box_groups)[i].boxes[j].box_position = box_initpos[i][j];
            (*box_groups)[i].boxes[j].box_size = 250.0f;
            (*box_groups)[i].boxes[j].box_color = BOX_COLOR_NONE;
            (*box_groups)[i].boxes[j].return_status = false;
        }

        (*box_groups)[i].pick_pos[0] = pick_template[0];
        (*box_groups)[i].pick_pos[1] = pick_template[1];
    }
}

void world_init(WorldModel *world)
{
    if (world == NULL)
    {
        return;
    }

    memset(world, 0, sizeof(*world));
    box_group_init(&world->box_groups);
}
