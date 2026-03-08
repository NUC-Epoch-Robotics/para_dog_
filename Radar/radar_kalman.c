#include "radar_kalman.h"
#include <math.h>
#include <string.h>

#define RADAR_KALMAN_EPS 1e-6f /* 奇异性判定的极小阈值 */

/* 生成单位矩阵的便捷封装 */
static void set_identity(matrix_t *m)
{
    matrix_identity(m);
}

/* 将对角线填入常数，其余清零 */
static void set_diag(matrix_t *m, float diag)
{
    matrix_fill(m, 0.0f);
    for (size_t i = 0; i < m->rows && i < m->cols; ++i)
    {
        matrix_set(m, i, i, diag);
    }
}

/* 初始化缓冲区、形状与噪声协方差 */
void radar_kalman_init(radar_kalman_t *f, float q_var, float r_var, float dt)
{
    if (f == NULL)
    {
        return;
    }
    // 包装矩阵
    matrix_wrap(&f->x, 4, 1, f->x_buf);
    matrix_wrap(&f->x_pred, 4, 1, f->x_pred_buf);
    matrix_wrap(&f->P, 4, 4, f->P_buf);
    matrix_wrap(&f->P_pred, 4, 4, f->P_pred_buf);
    matrix_wrap(&f->Q, 4, 4, f->Q_buf);
    matrix_wrap(&f->R, 2, 2, f->R_buf);
    matrix_wrap(&f->F, 4, 4, f->F_buf);
    matrix_wrap(&f->H, 2, 4, f->H_buf);
    matrix_wrap(&f->I, 4, 4, f->I_buf);
    matrix_wrap(&f->K, 4, 2, f->K_buf);
    matrix_wrap(&f->S, 2, 2, f->S_buf);
    matrix_wrap(&f->y, 2, 1, f->y_buf);
    matrix_wrap(&f->tmp4x4, 4, 4, f->tmp4x4_buf);
    matrix_wrap(&f->tmp4x2, 4, 2, f->tmp4x2_buf);
    matrix_wrap(&f->tmp2x4, 2, 4, f->tmp2x4_buf);
    matrix_wrap(&f->tmp2x2, 2, 2, f->tmp2x2_buf);
    matrix_wrap(&f->tmp4x1, 4, 1, f->tmp4x1_buf);
    matrix_wrap(&f->tmp2x1, 2, 1, f->tmp2x1_buf);

    // 初始化状态为0
    matrix_fill(&f->x, 0.0f);
    matrix_fill(&f->x_pred, 0.0f);

    // 状态转移矩阵 F
    set_identity(&f->F);
    matrix_set(&f->F, 0, 2, dt);  // x += vx * dt
    matrix_set(&f->F, 1, 3, dt);  // y += vy * dt

    // 观测矩阵 H (测量x和y)
    matrix_fill(&f->H, 0.0f);
    matrix_set(&f->H, 0, 0, 1.0f);  // 测量x
    matrix_set(&f->H, 1, 1, 1.0f);  // 测量y

    // 单位矩阵
    set_identity(&f->I);

    // 协方差初始化
    set_diag(&f->P, 1.0f);
    set_diag(&f->P_pred, 1.0f);

    // 过程噪声 Q (对角线)
    set_diag(&f->Q, q_var);

    // 测量噪声 R (对角线)
    set_diag(&f->R, r_var);

    // 其他矩阵清零
    matrix_fill(&f->K, 0.0f);
    matrix_fill(&f->S, 0.0f);
    matrix_fill(&f->y, 0.0f);
    matrix_fill(&f->tmp4x4, 0.0f);
    matrix_fill(&f->tmp4x2, 0.0f);
    matrix_fill(&f->tmp2x4, 0.0f);
    matrix_fill(&f->tmp2x2, 0.0f);
    matrix_fill(&f->tmp4x1, 0.0f);
    matrix_fill(&f->tmp2x1, 0.0f);
}

/* 预测步骤 */
void radar_kalman_predict(radar_kalman_t *f, float dt)
{
    if (f == NULL)
    {
        return;
    }

    // 更新状态转移矩阵中的dt
    matrix_set(&f->F, 0, 2, dt);
    matrix_set(&f->F, 1, 3, dt);

    // x_pred = F * x
    matrix_mul(&f->F, &f->x, &f->x_pred);

    // P_pred = F * P * F^T + Q
    matrix_transpose(&f->F, &f->tmp4x4);         // tmp4x4 = F^T
    matrix_mul(&f->P, &f->tmp4x4, &f->P_pred);  // P_pred = P * F^T
    matrix_mul(&f->F, &f->P_pred, &f->P_pred);  // P_pred = F * (P * F^T)
    matrix_add(&f->P_pred, &f->Q, &f->P_pred);
}

/* 更新步骤 */
void radar_kalman_update(radar_kalman_t *f, float x_meas, float y_meas)
{
    if (f == NULL)
    {
        return;
    }

    // 测量向量 z = [x_meas, y_meas]
    float z_buf[2] = {x_meas, y_meas};
    matrix_t z;
    matrix_wrap(&z, 2, 1, z_buf);

    // y = z - H * x_pred
    matrix_mul(&f->H, &f->x_pred, &f->tmp2x1);
    matrix_sub(&z, &f->tmp2x1, &f->y);

    // S = H * P_pred * H^T + R
    matrix_mul(&f->H, &f->P_pred, &f->tmp2x4);
    matrix_transpose(&f->H, &f->tmp4x2);  // tmp4x2作为H^T
    matrix_mul(&f->tmp2x4, &f->tmp4x2, &f->S);
    matrix_add(&f->S, &f->R, &f->S);

    // K = P_pred * H^T * S^-1
    // 首先计算 S^-1
    // 简单2x2矩阵求逆
    float det = matrix_get(&f->S, 0, 0) * matrix_get(&f->S, 1, 1) - 
                matrix_get(&f->S, 0, 1) * matrix_get(&f->S, 1, 0);
    if (fabsf(det) < RADAR_KALMAN_EPS)
    {
        return;  // 奇异，跳过更新
    }
    float inv_det = 1.0f / det;
    matrix_set(&f->tmp2x2, 0, 0, matrix_get(&f->S, 1, 1) * inv_det);
    matrix_set(&f->tmp2x2, 0, 1, -matrix_get(&f->S, 0, 1) * inv_det);
    matrix_set(&f->tmp2x2, 1, 0, -matrix_get(&f->S, 1, 0) * inv_det);
    matrix_set(&f->tmp2x2, 1, 1, matrix_get(&f->S, 0, 0) * inv_det);

    // K = P_pred * H^T * S^-1
    matrix_mul(&f->P_pred, &f->tmp4x2, &f->tmp4x2);  // tmp4x2现在是 H^T
    matrix_mul(&f->tmp4x2, &f->tmp2x2, &f->K);

    // x = x_pred + K * y
    matrix_mul(&f->K, &f->y, &f->tmp4x1);
    matrix_add(&f->x_pred, &f->tmp4x1, &f->x);

    // P = (I - K * H) * P_pred
    matrix_mul(&f->K, &f->H, &f->tmp4x4);
    matrix_sub(&f->I, &f->tmp4x4, &f->tmp4x4);
    matrix_mul(&f->tmp4x4, &f->P_pred, &f->P);
}

/* 获取当前状态 */
void radar_kalman_get_state(const radar_kalman_t *f, float *x, float *y, float *vx, float *vy)
{
    if (f == NULL || x == NULL || y == NULL || vx == NULL || vy == NULL)
    {
        return;
    }
    *x = matrix_get(&f->x, 0, 0);
    *y = matrix_get(&f->x, 1, 0);
    *vx = matrix_get(&f->x, 2, 0);
    *vy = matrix_get(&f->x, 3, 0);
}