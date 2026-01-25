#ifndef AUTOCTRL_H
#define AUTOCTRL_H

#define Tracking 0//行为属性
#define Action 1
#include "stdint.h"
typedef enum{//任务节点枚举
    track_startpoint=0 
}task_node;

typedef struct task_Attribute{//任务属性结构体
    task_node node;//当前任务节点
     uint8_t task_type;//0跟踪1动作
}task_Attribute;
typedef enum {
    stairs
}offroad_motion;
typedef union{
    float target_location[3];//目标位置
    float target_yaw;
	offroad_motion target_motion;//目标动作
}target_union;
typedef struct auto_ctrlcenter{//自动控制中心结构体
    task_Attribute task_attr;//任务属性
	target_union target;
    uint8_t task_complete_flag;//任务完成标志0/1
}auto_ctrlcenter;
#endif

