#include "imu_kalman.h"
#include "hwt605.h"
#include <math.h>
#include <string.h>

#define IMU_KALMAN_EPS 1e-6f /* 奇异性判定的极小阈值 */

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

/* 3x3 矩阵求逆，接近奇异时返回错误 */
static matrix_status_t matrix_inverse_3x3(const matrix_t *a, matrix_t *inv)
{
    if (a == NULL || inv == NULL)
    {
        return MATRIX_ERR_NULL;
    }
    if (a->rows != 3 || a->cols != 3 || inv->rows != 3 || inv->cols != 3)
    {
        return MATRIX_ERR_DIM;
    }

    const float *m = a->data;
    float det = m[0] * (m[4] * m[8] - m[5] * m[7]) -
                m[1] * (m[3] * m[8] - m[5] * m[6]) +
                m[2] * (m[3] * m[7] - m[4] * m[6]);

    if (fabsf(det) < IMU_KALMAN_EPS)
    {
        return MATRIX_ERR_OOB;
    }

    float inv_det = 1.0f / det;

    inv->data[0] = (m[4] * m[8] - m[5] * m[7]) * inv_det;
    inv->data[1] = -(m[1] * m[8] - m[2] * m[7]) * inv_det;
    inv->data[2] = (m[1] * m[5] - m[2] * m[4]) * inv_det;
    inv->data[3] = -(m[3] * m[8] - m[5] * m[6]) * inv_det;
    inv->data[4] = (m[0] * m[8] - m[2] * m[6]) * inv_det;
    inv->data[5] = -(m[0] * m[5] - m[2] * m[3]) * inv_det;
    inv->data[6] = (m[3] * m[7] - m[4] * m[6]) * inv_det;
    inv->data[7] = -(m[0] * m[7] - m[1] * m[6]) * inv_det;
    inv->data[8] = (m[0] * m[4] - m[1] * m[3]) * inv_det;

    return MATRIX_OK;
}

/* 初始化缓冲区、形状与噪声协方差 */
void imu_kalman_init(imu_kalman_t *f, float q_var, float r_var)
{
    if (f == NULL)
    {
        return;
    }

    matrix_wrap(&f->x, 3, 1, f->x_buf);
    matrix_wrap(&f->x_pred, 3, 1, f->x_pred_buf);
    matrix_wrap(&f->P, 3, 3, f->P_buf);
    matrix_wrap(&f->P_pred, 3, 3, f->P_pred_buf);
    matrix_wrap(&f->Q, 3, 3, f->Q_buf);
    matrix_wrap(&f->R, 3, 3, f->R_buf);
    matrix_wrap(&f->F, 3, 3, f->F_buf);
    matrix_wrap(&f->H, 3, 3, f->H_buf);
    matrix_wrap(&f->I, 3, 3, f->I_buf);
    matrix_wrap(&f->K, 3, 3, f->K_buf);
    matrix_wrap(&f->S, 3, 3, f->S_buf);
    matrix_wrap(&f->y, 3, 1, f->y_buf);
    matrix_wrap(&f->tmp3x3, 3, 3, f->tmp3x3_buf);
    matrix_wrap(&f->tmp3x1, 3, 1, f->tmp3x1_buf);

    matrix_fill(&f->x, 0.0f);
    matrix_fill(&f->x_pred, 0.0f);
    set_identity(&f->F);
    set_identity(&f->H);
    set_identity(&f->I);
    set_diag(&f->P, 1.0f);
    set_diag(&f->P_pred, 1.0f);
    set_diag(&f->Q, q_var);
    set_diag(&f->R, r_var);
    matrix_fill(&f->K, 0.0f);
    matrix_fill(&f->S, 0.0f);
    matrix_fill(&f->y, 0.0f);
    matrix_fill(&f->tmp3x3, 0.0f);
    matrix_fill(&f->tmp3x1, 0.0f);
}

/* 预测：用陀螺角速度 (deg/s) 积分 dt(s) */
void imu_kalman_predict(imu_kalman_t *f, const float gyro_dps[3], float dt_s)
{
    if (f == NULL || gyro_dps == NULL || dt_s <= 0.0f)
    {
        return;
    }

    /* x(k|k-1) = x(k-1) + omega*dt  (omega in deg/s, dt in s) */
    f->x_pred_buf[0] = f->x_buf[0] + gyro_dps[0] * dt_s;
    f->x_pred_buf[1] = f->x_buf[1] + gyro_dps[1] * dt_s;
    f->x_pred_buf[2] = f->x_buf[2] + gyro_dps[2] * dt_s;

    /* P(k|k-1) = F P F' + Q*dt^2 ; F=I */
    matrix_copy(&f->P, &f->P_pred);
    matrix_scalar(&f->Q, dt_s * dt_s, &f->tmp3x3);
    matrix_add(&f->P_pred, &f->tmp3x3, &f->P_pred);
}

/* 更新：用测得的欧拉角 (deg) 校正 */
void imu_kalman_update(imu_kalman_t *f, const float meas_deg[3])
{
    if (f == NULL || meas_deg == NULL)
    {
        return;
    }

    /* y = z - H x_pred ; H = I */
    f->tmp3x1_buf[0] = meas_deg[0];
    f->tmp3x1_buf[1] = meas_deg[1];
    f->tmp3x1_buf[2] = meas_deg[2];
    matrix_sub(&f->tmp3x1, &f->x_pred, &f->y);

    /* S = H P_pred H' + R ; H = I */
    matrix_add(&f->P_pred, &f->R, &f->S);

    /* K = P_pred * S^{-1} */
    if (matrix_inverse_3x3(&f->S, &f->tmp3x3) != MATRIX_OK)
    {
        return;
    }
    matrix_mul(&f->P_pred, &f->tmp3x3, &f->K);

    /* x = x_pred + K y */
    matrix_mul(&f->K, &f->y, &f->tmp3x1);
    matrix_add(&f->x_pred, &f->tmp3x1, &f->x);

    /* P = (I - K H) P_pred ; H = I */
    matrix_sub(&f->I, &f->K, &f->tmp3x3);
    matrix_mul(&f->tmp3x3, &f->P_pred, &f->P);
}

/* 便捷接口：单次融合 HWT 的角速度与姿态 */
void imu_kalman_step_from_hwt(imu_kalman_t *f, const struct wit *hwt, float dt_s)
{
    if (f == NULL || hwt == NULL)
    {
        return;
    }

    float gyro_dps[3] = {hwt->Gx, hwt->Gy, hwt->Gz};
    float meas_deg[3] = {hwt->fRoll, hwt->fPitch, hwt->fYaw};

    imu_kalman_predict(f, gyro_dps, dt_s);
    imu_kalman_update(f, meas_deg);
}

/* 读取融合角度 (0:roll,1:pitch,2:yaw)，单位度 */
float imu_kalman_get_angle_deg(const imu_kalman_t *f, uint8_t axis)
{
    if (f == NULL || axis > 2)
    {
        return 0.0f;
    }
    return f->x_buf[axis];
}
