#include "world.h"
#include "postrue_control.h"
#include <string.h>

static void box_group_init(BoxGroup (*box_groups)[4])
{
    if (box_groups == NULL)
    {
        Error_Handler();
    }
    static const Coordinates box_initpos[4][2] = {
        {{10.0f, 10.0f}, {10.0f, 30.0f}},
        {{30.0f, 10.0f}, {30.0f, 30.0f}},
        {{50.0f, 10.0f}, {50.0f, 30.0f}},
        {{70.0f, 10.0f}, {70.0f, 30.0f}}};
    static const Coordinates pick_initpos[4][2] = {
        {{10.0f, 60.0f}, {10.0f, 30.0f}},
        {{30.0f, 60.0f}, {30.0f, 30.0f}},
        {{50.0f, 60.0f}, {50.0f, 30.0f}},
        {{70.0f, 60.0f}, {70.0f, 30.0f}}};

    for (uint8_t i = 0; i < 4; i++)
    {
        (*box_groups)[i].group_id = i;
        (*box_groups)[i].return_status = false;

        for (uint8_t j = 0; j < 2; j++)
        {
            uint8_t box_id = (j == 0U) ? i : (uint8_t)(i + 4U);
            (*box_groups)[i].boxes[j].box_id = box_id;
            (*box_groups)[i].boxes[j].box_position = box_initpos[i][j];
            (*box_groups)[i].boxes[j].box_size = 250.0f;
            (*box_groups)[i].boxes[j].box_color = BOX_COLOR_NONE;
            (*box_groups)[i].boxes[j].return_status = false;
        }

        (*box_groups)[i].pick_pos[BOX_PICK_FRONT_TO_BACK][0] = pick_initpos[i][0];
        (*box_groups)[i].pick_pos[BOX_PICK_FRONT_TO_BACK][1] = pick_initpos[i][1];
        (*box_groups)[i].pick_pos[BOX_PICK_BACK_TO_FRONT][0] = pick_initpos[i][1];
        (*box_groups)[i].pick_pos[BOX_PICK_BACK_TO_FRONT][1] = pick_initpos[i][0];
    }
}
void return_field_init(ReturnField *return_field)
{
    if (return_field == NULL)
    {
        Error_Handler();
    }
    static const Coordinates return_positions[4] = {
        {100.0f, 500.0f},
        {300.0f, 500.0f},
        {500.0f, 500.0f},
        {700.0f, 500.0f}};
    static const Coordinates field_positions[4] = {
        {100.0f, 550.0f},
        {300.0f, 550.0f},
        {500.0f, 550.0f},
        {700.0f, 550.0f}};

    for (uint8_t i = 0; i < 4; i++)
    {
        return_field[i].field_color = (BoxColor)(i + 1U); // Assuming field colors correspond to box colors
        return_field[i].return_position = return_positions[i];
        return_field[i].field_position = field_positions[i];
        return_field[i].returned_num = 0;
    }
}
void world_init(WorldModel *world)
{
    if (world == NULL)
    {
       Error_Handler();
    }

    memset(world, 0, sizeof(*world));
    Dog_Init(&world->dog);
    box_group_init(&world->box_groups);
    return_field_init(world->return_field);
}
