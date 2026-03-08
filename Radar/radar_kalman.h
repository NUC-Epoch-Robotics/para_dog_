#ifndef RADAR_KALMAN_H
#define RADAR_KALMAN_H

#include "simple_matrix.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    matrix_t x;         /* 后验状态 [x, y, vx, vy] */
    matrix_t x_pred;    /* 先前状态 */
    matrix_t P;         /* 后验协方差 4x4 */
    matrix_t P_pred;    /* 先前协方差 4x4 */
    matrix_t Q;         /* 过程噪声 4x4 */
    matrix_t R;         /* 测量噪声 2x2 */
    matrix_t F;         /* 状态转移 4x4 */
    matrix_t H;         /* 观测矩阵 2x4 */
    matrix_t I;         /* 单位矩阵 4x4 */
    matrix_t K;         /* 卡尔曼增益 4x2 */
    matrix_t S;         /* 创新协方差 2x2 */
    matrix_t y;         /* innovation (z - Hx) 2x1 */
    matrix_t tmp4x4;    /* scratch 4x4 */
    matrix_t tmp4x2;    /* scratch 4x2 */
    matrix_t tmp2x4;    /* scratch 2x4 */
    matrix_t tmp2x2;    /* scratch 2x2 */
    matrix_t tmp4x1;    /* scratch 4x1 */
    matrix_t tmp2x1;    /* scratch 2x1 */

    float x_buf[4];
    float x_pred_buf[4];
    float P_buf[16];
    float P_pred_buf[16];
    float Q_buf[16];
    float R_buf[4];
    float F_buf[16];
    float H_buf[8];
    float I_buf[16];
    float K_buf[8];
    float S_buf[4];
    float y_buf[2];
    float tmp4x4_buf[16];
    float tmp4x2_buf[8];
    float tmp2x4_buf[8];
    float tmp2x2_buf[4];
    float tmp4x1_buf[4];
    float tmp2x1_buf[2];
} radar_kalman_t;

void radar_kalman_init(radar_kalman_t *f, float q_var, float r_var, float dt);
void radar_kalman_predict(radar_kalman_t *f, float dt);
void radar_kalman_update(radar_kalman_t *f, float x_meas, float y_meas);
void radar_kalman_get_state(const radar_kalman_t *f, float *x, float *y, float *vx, float *vy);

#ifdef __cplusplus
}
#endif

#endif /* RADAR_KALMAN_H */
