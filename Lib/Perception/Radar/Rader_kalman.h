#ifndef __RADER_KALMAN_H__
#define __RADER_KALMAN_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 卡尔曼滤波器维度定义
#define KF_STATE_DIM    4       // 状态向量维度 [x, y, vx, vy]
#define KF_MEASURE_DIM  2       // 测量向量维度 [x, y]

// 雷达数据结构体
typedef struct {
    float x;                    // 滤波后 x 坐标
    float y;                    // 滤波后 y 坐标
    float vx;                   // x 方向速度
    float vy;                   // y 方向速度
    uint32_t timestamp;         // 时间戳
    bool is_valid;              // 数据有效性标志
} RadarData_t;

// 卡尔曼滤波器结构体
typedef struct {
    float state[4];                     // 状态向量
    float covariance[4][4];             // 状态协方差矩阵
    float process_noise[4][4];          // 过程噪声协方差
    float measurement_noise[2][2];      // 测量噪声协方差
    float state_transition[4][4];       // 状态转移矩阵
    float measurement_matrix[2][4];     // 测量矩阵
    float dt;                           // 时间间隔
    bool is_initialized;                // 初始化标志
} RadarKalmanFilter_t;

// 函数声明
void Kalman_Init(RadarKalmanFilter_t *kf, float dt);
void Kalman_Update(RadarKalmanFilter_t *kf, float measured_x, float measured_y);
void Kalman_GetPosition(RadarKalmanFilter_t *kf, float *x, float *y);
void Kalman_GetVelocity(RadarKalmanFilter_t *kf, float *vx, float *vy);
void Kalman_GetData(RadarKalmanFilter_t *kf, RadarData_t *data);
bool Kalman_IsValid(RadarKalmanFilter_t *kf);

// 数据处理接口
void RadarData_Process(uint32_t raw_x, uint32_t raw_y, RadarData_t *output);

// 获取滤波器实例
RadarKalmanFilter_t* Kalman_GetInstance(void);

#ifdef __cplusplus
}
#endif

#endif /* __RADER_KALMAN_H__ */
