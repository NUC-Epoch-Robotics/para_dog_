#ifndef _TASK_SWITCH_H
#define _TASK_SWITCH_H
#include <stdbool.h>

typedef struct Dog Dog;
typedef struct BoxGroup BoxGroup;
typedef struct ReturnField ReturnField;
typedef enum Mission_Type
{
    IDLE,
    SEEK_BOX,
    PICK_UP_BOX,
    DELIVER_BOX,
    RETURN_BOX,
    PICK_DOWN_BOX

} Mission_Type;
typedef enum BtStatus
{
    BT_FAILURE = 0,
    BT_SUCCESS = 1,
    BT_RUNNING = 2
} BtStatus;
typedef enum CTX_SUBTASK
{
    SEEK_SELECT_TARGET = 0,
    SEEK_NAVIGATE_TO_TARGET = 1,
    SEEK_PICK_BOX = 2
} CTX_SUBTASK;
typedef struct NavigateStage{
    uint32_t navigate_start_tick;
    bool is_navigating;
    float target[2];
    float target_yaw;
}NavigateStage;
typedef struct PickBoxStage{
    uint32_t pick_start_tick;
    uint8_t pick_index;
    uint8_t pick_step;
    uint8_t picked_count;
} PickBoxStage;
 typedef struct Seek_Subtask
{
    uint8_t group_index;
    bool has_target;
    NavigateStage navigate_stage;
    PickBoxStage pick_box_stage;
} SeekBoxTask;
typedef struct Deliver_Subtask
{
    uint8_t return_field_index;
    bool has_target;
    NavigateStage navigate_stage;
} DeliverBoxTask;
typedef struct TaskTreeContext
{


    Mission_Type mission;
    CTX_SUBTASK subtask;
    SeekBoxTask seekbox_ctx;
    DeliverBoxTask deliver_ctx;
    uint8_t fail_count[4][2];

} TaskTreeContext;

void task_switch(Dog *dog, BoxGroup (*box_groups)[4], ReturnField (*return_field)[4]);

#endif /* _TASK_SWITCH_H */
