#ifndef WORLD_H
#define WORLD_H
#include <stdbool.h>
#include <stdint.h>
#include "dog.h"
typedef enum BoxColor
{
    BOX_COLOR_NONE,
    BOX_COLOR_RED,
    BOX_COLOR_BLUE,
    BOX_COLOR_GREY,
    BOX_COLOR_GREEN
} BoxColor;

typedef enum BoxPickDirection
{
    BOX_PICK_FRONT_TO_BACK = 0,
    BOX_PICK_BACK_TO_FRONT = 1
} BoxPickDirection;

typedef struct SupplyBox
{
    uint8_t box_id;           // 0-7
    Coordinates box_position; // x/y-mm
    float box_size;
    BoxColor box_color;
    bool picked;        // 1 for picked, 0 for not picked
    bool return_status; // 1 for returned, 0 for not returned
} SupplyBox;
typedef struct BoxGroup
{
    uint8_t group_id; // 0-3
    SupplyBox boxes[2];
    Coordinates pick_pos[2][2]; // [方向][顺序]，每个方向依次经过两个box的pick坐标
    bool picked;        // 1 for picked, 0 for not picked
    bool return_status;
} BoxGroup;
typedef struct ReturnField
{
    BoxColor field_color;
    Coordinates return_position;
    Coordinates field_position; // x/y-mm，需场地实测
    uint8_t returned_num;       //<=2
} ReturnField;
// 物资箱周身轨道
// world_model包含物资箱和归还区的状态信息，供任务切换模块调用
typedef struct WorldModel
{
    Dog dog;
    BoxGroup box_groups[4];
    ReturnField return_field[4];
} WorldModel;
extern WorldModel world;
void world_init(WorldModel *world);

#endif /* WORLD_H */
