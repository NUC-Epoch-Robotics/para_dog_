#ifndef OBSTACLE_CROSSING_H
#define OBSTACLE_CROSSING_H

#include "stdint.h"
#include <stdbool.h>

typedef struct Dog Dog;

// 行动状态枚举
typedef enum
{
    ACTION_STATE_IDLE,   // 空闲
    ACTION_STATE_ROTATE, // 旋转调整角度
    ACTION_STATE_MOVE,   // 直线移动
    ACTION_STATE_AVOID,  // 避障
    ACTION_STATE_ARRIVED // 已到达
} ActionState_t;

// 运动状态枚举（底层接口使用）
typedef enum
{
    MOTION_STATE_IDLE,
    MOTION_STATE_MOVING,
    MOTION_STATE_ROTATING,
    MOTION_STATE_COMPLETED
} MotionState_t;



void ActionControl_Init(void);
void ActionControl_InitWithDog(Dog *dog);

    /**
     * @brief 获取当前机器人位置（从雷达数据）
     * @param x 输出位置 X
     * @param y 输出位置 Y
     * @return 1 表示数据有效，0 表示无效
     */
    uint8_t ActionControl_GetPosition(float *x, float *y);

/**
 * @brief 获取当前机器人朝向角度（从 IMU）
 * @return 当前角度（度）
 */
float ActionControl_GetAngle(void);

/**
 * @brief 设置目标点
 * @param target_x 目标 X 坐标
 * @param target_y 目标 Y 坐标
 */
bool ActionControl_SetTarget(float target_x, float target_y);

/**
 * @brief 行动控制主循环（需要在主程序中周期性调用）
 *        根据雷达当前位置和目标坐标，自动计算机器人应该如何移动
 */
void ActionControl_Update(void);

/**
 * @brief 检查是否到达目标
 * @return 1 表示到达，0 表示未到达
 */
uint8_t ActionControl_IsArrived(void);

/**
 * @brief 获取当前行动状态
 * @return 当前状态枚举值
 */
ActionState_t ActionControl_GetState(void);

/**
 * @brief 重置控制器状态
 */
void ActionControl_Reset(void);

/**
 * @brief 行动控制器底层运动接口
 *        封装现有步态函数，符合运动控制开发规范
 */
void Motion_StandUp(void);                   // 站立动作
void Motion_MoveForward(float distance_mm);  // 前进指定距离（mm）
void Motion_MoveBackward(float distance_mm); // 后退指定距离（mm）
void Motion_RotateLeft(void);     // 左转指定角度（度）
void Motion_RotateRight(void);    // 右转指定角度（度）
void Motion_Stop(void);                      // 停止运动
void Motion_SetSpeed(float speed_level);     // 设置速度等级（0.0-1.0）
MotionState_t Motion_GetCurrentState(void);  // 获取当前运动状态
bool Motion_IsMotionCompleted(void);         // 检查动作是否完成

#endif // OBSTACLE_CROSSING_H
