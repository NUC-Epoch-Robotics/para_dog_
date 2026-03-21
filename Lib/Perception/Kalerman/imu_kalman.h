#ifndef IMU_KALMAN_H
#define IMU_KALMAN_H

#include "simple_matrix.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct wit;

typedef struct
{
    matrix_t x;         /* posterior state [roll pitch yaw] */
    matrix_t x_pred;    /* prior state */
    matrix_t P;         /* posterior covariance */
    matrix_t P_pred;    /* prior covariance */
    matrix_t Q;         /* process noise */
    matrix_t R;         /* measurement noise */
    matrix_t F;         /* state transition (3x3 identity) */
    matrix_t H;         /* observation matrix (3x3 identity) */
    matrix_t I;         /* identity 3x3 */
    matrix_t K;         /* Kalman gain */
    matrix_t S;         /* innovation covariance */
    matrix_t y;         /* innovation (z - Hx) */
    matrix_t tmp3x3;    /* scratch 3x3 */
    matrix_t tmp3x1;    /* scratch 3x1 */

    float x_buf[3];
    float x_pred_buf[3];
    float P_buf[9];
    float P_pred_buf[9];
    float Q_buf[9];
    float R_buf[9];
    float F_buf[9];
    float H_buf[9];
    float I_buf[9];
    float K_buf[9];
    float S_buf[9];
    float y_buf[3];
    float tmp3x3_buf[9];
    float tmp3x1_buf[3];
} imu_kalman_t;

void imu_kalman_init(imu_kalman_t *f, float q_var, float r_var);
void imu_kalman_predict(imu_kalman_t *f, const float gyro_dps[3], float dt_s);
void imu_kalman_update(imu_kalman_t *f, const float meas_deg[3]);
void imu_kalman_step_from_hwt(imu_kalman_t *f, const struct wit *hwt, float dt_s);
float imu_kalman_get_angle_deg(const imu_kalman_t *f, uint8_t axis);

#ifdef __cplusplus
}
#endif

#endif /* IMU_KALMAN_H */
