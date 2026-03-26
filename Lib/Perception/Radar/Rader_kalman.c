#include "Rader_kalman.h"
#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"

// 单例滤波器
static RadarKalmanFilter_t s_kalman_filter;
static bool s_is_init = false;

// ========== 矩阵运算辅助函数 ==========

// 4x4 矩阵乘法：result = a * b（移除 const 以兼容结构体成员类型）
static void Matrix_Multiply_4x4(float a[4][4], float b[4][4], float result[4][4]) {
    float temp[4][4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                temp[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    memcpy(result, temp, sizeof(temp));
}

// 4x4 矩阵转置：dst = src^T（移除 const 以兼容结构体成员类型）
static void Matrix_Transpose_4x4(float src[4][4], float dst[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            dst[i][j] = src[j][i];
        }
    }
}

// 2x2 矩阵求逆：dst = src^-1，失败返回 false
static bool Matrix_Inverse_2x2(float src[2][2], float dst[2][2]) {
    float det = src[0][0] * src[1][1] - src[0][1] * src[1][0];
    if (fabsf(det) < 1e-6f) {
        return false;
    }
    
    dst[0][0] = src[1][1] / det;
    dst[0][1] = -src[0][1] / det;
    dst[1][0] = -src[1][0] / det;
    dst[1][1] = src[0][0] / det;
    
    return true;
}

// ========== 卡尔曼滤波核心函数 ==========

// 获取滤波器单例
RadarKalmanFilter_t* Kalman_GetInstance(void) {
    return &s_kalman_filter;
}

// 初始化卡尔曼滤波器
void Kalman_Init(RadarKalmanFilter_t *kf, float dt) {
    if (kf == NULL) {
        return;
    }
    
    memset(kf, 0, sizeof(RadarKalmanFilter_t));
    
    kf->dt = dt;
    kf->is_initialized = false;
    
    // 初始化状态向量 [x, y, vx, vy]
    kf->state[0] = 0.0f;
    kf->state[1] = 0.0f;
    kf->state[2] = 0.0f;
    kf->state[3] = 0.0f;
    
    // 初始化状态转移矩阵 F
    kf->state_transition[0][0] = 1.0f;
    kf->state_transition[0][2] = dt;
    kf->state_transition[1][1] = 1.0f;
    kf->state_transition[1][3] = dt;
    kf->state_transition[2][2] = 1.0f;
    kf->state_transition[3][3] = 1.0f;
    
    // 初始化测量矩阵 H (只测量位置)
    kf->measurement_matrix[0][0] = 1.0f;
    kf->measurement_matrix[1][1] = 1.0f;
    
    // 初始化协方差矩阵 P (单位矩阵)
    for (int i = 0; i < 4; i++) {
        kf->covariance[i][i] = 1.0f;
    }
    
    // 初始化测量噪声协方差 R
    kf->measurement_noise[0][0] = 0.5f;
    kf->measurement_noise[1][1] = 0.5f;
    
    // 初始化过程噪声协方差 Q
    kf->process_noise[0][0] = 0.1f;
    kf->process_noise[1][1] = 0.1f;
    kf->process_noise[2][2] = 0.1f;
    kf->process_noise[3][3] = 0.1f;
}

// 卡尔曼滤波更新（预测 + 更新）
void Kalman_Update(RadarKalmanFilter_t *kf, float measured_x, float measured_y) {
    if (kf == NULL) {
        return;
    }
    
    float predicted_state[4] = {0};
    float predicted_cov[4][4] = {0};
    float innovation[2] = {0};
    float innovation_cov[2][2] = {0};
    float innovation_cov_inv[2][2] = {0};
    float kalman_gain[4][2] = {0};
    float temp_matrix[4][4] = {0};
    
    // ========== 预测步骤 ==========
    
    // 状态预测：x̂ = F * x
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            predicted_state[i] += kf->state_transition[i][j] * kf->state[j];
        }
    }
    
    // 协方差预测：P = F * P * F^T + Q
    Matrix_Transpose_4x4(kf->state_transition, temp_matrix);
    Matrix_Multiply_4x4(kf->state_transition, kf->covariance, predicted_cov);
    Matrix_Multiply_4x4(predicted_cov, temp_matrix, predicted_cov);
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            predicted_cov[i][j] += kf->process_noise[i][j];
        }
    }
    
    // ========== 更新步骤 ==========
    
    // 计算新息：y = z - H * x̂
    innovation[0] = measured_x - kf->measurement_matrix[0][0] * predicted_state[0];
    innovation[1] = measured_y - kf->measurement_matrix[1][1] * predicted_state[1];
    
    // 计算新息协方差：S = H * P * H^T + R
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 4; k++) {
                for (int l = 0; l < 4; l++) {
                    innovation_cov[i][j] += kf->measurement_matrix[i][k] * 
                                            predicted_cov[k][l] * 
                                            kf->measurement_matrix[j][l];
                }
            }
            innovation_cov[i][j] += kf->measurement_noise[i][j];
        }
    }
    
    // 计算新息协方差的逆
    if (!Matrix_Inverse_2x2(innovation_cov, innovation_cov_inv)) {
        return;
    }
    
    // 计算卡尔曼增益：K = P * H^T * S^-1
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 4; k++) {
                kalman_gain[i][j] += predicted_cov[i][k] * kf->measurement_matrix[j][k];
            }
            kalman_gain[i][j] *= innovation_cov_inv[j][j];
        }
    }
    
    // 状态更新：x = x̂ + K * y
    for (int i = 0; i < 4; i++) {
        kf->state[i] = predicted_state[i];
        for (int j = 0; j < 2; j++) {
            kf->state[i] += kalman_gain[i][j] * innovation[j];
        }
    }
    
    // 协方差更新：P = (I - K * H) * P
    memset(temp_matrix, 0, sizeof(temp_matrix));
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 2; k++) {
                temp_matrix[i][j] += kalman_gain[i][k] * kf->measurement_matrix[k][j];
            }
        }
    }
    
    // I - K*H
    for (int i = 0; i < 4; i++) {
        temp_matrix[i][i] = 1.0f - temp_matrix[i][i];
    }
    
    Matrix_Multiply_4x4(temp_matrix, predicted_cov, kf->covariance);
    
    // 标记已初始化
    kf->is_initialized = true;
}

// 获取滤波后的位置
void Kalman_GetPosition(RadarKalmanFilter_t *kf, float *x, float *y) {
    if (kf == NULL || x == NULL || y == NULL) {
        return;
    }
    *x = kf->state[0];
    *y = kf->state[1];
}

// 获取滤波后的速度
void Kalman_GetVelocity(RadarKalmanFilter_t *kf, float *vx, float *vy) {
    if (kf == NULL || vx == NULL || vy == NULL) {
        return;
    }
    *vx = kf->state[2];
    *vy = kf->state[3];
}

// 获取完整数据
void Kalman_GetData(RadarKalmanFilter_t *kf, RadarData_t *data) {
    if (kf == NULL || data == NULL) {
        return;
    }
    data->x = kf->state[0];
    data->y = kf->state[1];
    data->vx = kf->state[2];
    data->vy = kf->state[3];
    data->timestamp = HAL_GetTick();
    data->is_valid = kf->is_initialized;
}

// 检查滤波器是否有效
bool Kalman_IsValid(RadarKalmanFilter_t *kf) {
    if (kf == NULL) {
        return false;
    }
    return kf->is_initialized;
}

// 数据处理接口（与 ReadData.c 对接）
void RadarData_Process(uint32_t raw_x, uint32_t raw_y, RadarData_t *output) {
    if (output == NULL) {
        return;
    }
    
    // 初始化滤波器（100ms 采样间隔，根据实际调整）
    if (!s_is_init) {
        Kalman_Init(&s_kalman_filter, 0.1f);
        s_is_init = true;
    }
    
    // 将 uint32_t 转换为 float（假设原始数据单位是毫米，转换为米）
    float measured_x = (float)raw_x / 1000.0f;
    float measured_y = (float)raw_y / 1000.0f;
    
    // 更新卡尔曼滤波
    Kalman_Update(&s_kalman_filter, measured_x, measured_y);
    
    // 获取滤波结果
    Kalman_GetData(&s_kalman_filter, output);
}
