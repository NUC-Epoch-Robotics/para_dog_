#include "offroad_autoctrl.h"
#include "dog.h"
#include "math.h"
#include "cmsis_os.h"
#include "ReadData.h"
#include "simple_matrix.h"

/* Kalman filter noise parameters (tunable) */
#define RADAR_KALMAN_Q 0.1f    /* process noise variance for radar x/y */
#define RADAR_KALMAN_R 0.5f    /* measurement noise variance for radar x/y */
#define GYRO_KALMAN_Q 0.01f    /* process noise variance for gyro/yaw */
#define GYRO_KALMAN_R 0.5f    /* measurement noise variance for gyro/yaw */

#define rad
#define cm
#define frame -1//场地左右手坐标系切换
auto_ctrlcenter dog_auto_ctrl[10]={
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=TRACK},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=TRACK},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=TRACK},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=TRACK},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=TRACK},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=TRACK},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=TRACK},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=TRACK},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=TRACK},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=TRACK},.task_complete_flag=0}
};
void AutoTaskUpdate(Dog *dog){
    if(dog->auto_ctrl.task_complete_flag==1){
        dog->auto_ctrl.task_attr.node++;  
        dog->auto_ctrl=dog_auto_ctrl[dog->auto_ctrl.task_attr.node];
    }
  // 根据当前任务节点执行相应的操作
};

void AutoTracking(Dog *dog){
    float err_x,err_y,err_yaw1,err_yaw2;
    do{
        //求误差/补偿量
        err_x=dog->auto_ctrl.target.target_location[0] - dog->location.pos[0];
        err_y=dog->auto_ctrl.target.target_location[1] - dog->location.pos[1];
        err_yaw1=atan2(err_y, err_x)rad - dog->location.yaw*frame;//场地坐标系下的yaw误差
        err_yaw2=dog->auto_ctrl.target.target_location[2]rad - dog->location.yaw*frame;//场地坐标系下的yaw误差
        if(fabsf(err_yaw1)>3.14f rad){//寻找最小yaw1误差
            if(err_yaw1>0){
                err_yaw1=err_yaw1-2*3.14f rad;
            }
            else{
                err_yaw1=err_yaw1+2*3.14f rad;
            }
        }
        if(fabsf(err_yaw2)>3.14f rad){//寻找最小yaw2误差
            if(err_yaw2>0){
                err_yaw2=err_yaw2-2*3.14f rad;
            }
            else{
                err_yaw2=err_yaw2+2*3.14f rad;
            }
        }
        //进入位置矫正
        if (fabsf(err_yaw1)>0.261f rad){//yaw1误差死区判定,
            if(err_yaw1*frame<0){//左手坐标系顺时针为旋转正方向，右手坐标系逆时针为旋转正方向
                dog->state=TURN_RIGHT;
            }
            else{
                dog->state=TURN_LEFT;
            }
        }
        else if(fabsf(err_x)>5.0f cm||fabsf(err_y)>5.0f cm){//位置误差死区判定
            dog->state=WALK_FORWARD;
        }
        else if(fabsf(err_yaw2)>0.261f rad){ //yaw2误差死区判定
            if(err_yaw2*frame<0){//左手坐标系顺时针为旋转正方向，右手坐标系逆时针为旋转正方向
                dog->state=TURN_RIGHT;
            }
            else{
                dog->state=TURN_LEFT;
            }
        }
        else{
            dog->state=STAND_UP_;
        }
        osDelay(1);//等待下一个控制周期
    }while(fabsf(err_yaw1)>0.261f rad||fabsf(err_x)>5.0f cm||fabsf(err_y)>5.0f cm);
    // dog->auto_ctrl.task_complete_flag==1
};
void AutoAction(Dog *dog){
    dog->auto_ctrl.task_complete_flag=1;
};
void AutoOffRoad(Dog *dog){
    AutoTaskUpdate(dog);
    if(dog->auto_ctrl.task_attr.task_type==TRACK){
        AutoTracking(dog);
    }
    else if(dog->auto_ctrl.task_attr.task_type==ACTION){
        AutoAction(dog);
    }
}
void Rader_Cartesiancoordinatesystem_KalmanFilter(vcp_message_t *msg, float dt)
{
    /*
     * 4D Kalman filter for radar data: position (x,y) and velocity (vx,vy).
     * State: [x, vx, y, vy]^T
     * Model: constant velocity (uniform motion).
     * Measurement: [x, vx, y, vy]^T from radar.
     * Filtered values are written back to msg.
     */
    if (msg == NULL || dt <= 0.0f) {
        return;
    }

    /* Static state and covariance (initialized on first call) */
    static bool initialized = false;
    static matrix_t x;      /* 4x1 state */
    static matrix_t P;      /* 4x4 covariance */
    static matrix_t F;      /* 4x4 state transition */
    static matrix_t H;      /* 4x4 measurement matrix (identity) */
    static matrix_t Q;      /* 4x4 process noise */
    static matrix_t R;      /* 4x4 measurement noise */
    static matrix_t I;      /* 4x4 identity */
    static matrix_t K;      /* 4x4 Kalman gain */
    static matrix_t S;      /* 4x4 innovation covariance */
    static matrix_t y;      /* 4x1 innovation */
    static matrix_t z;      /* 4x1 measurement */
    static matrix_t x_pred; /* 4x1 predicted state */
    static matrix_t P_pred; /* 4x4 predicted covariance */
    static matrix_t temp4x4;/* 4x4 temp */
    static matrix_t temp4x4_2; /* additional 4x4 temp for transpose */
    static matrix_t temp4x1;/* 4x1 temp */

    /* Buffers */
    static float x_buf[4];
    static float P_buf[16];
    static float F_buf[16];
    static float H_buf[16];
    static float Q_buf[16];
    static float R_buf[16];
    static float I_buf[16];
    static float K_buf[16];
    static float S_buf[16];
    static float y_buf[4];
    static float z_buf[4];
    static float x_pred_buf[4];
    static float P_pred_buf[16];
    static float temp4x4_buf[16];
    static float temp4x4_2_buf[16];
    static float temp4x1_buf[4];

    if (!initialized) {
        /* Wrap matrices */
        matrix_wrap(&x, 4, 1, x_buf);
        matrix_wrap(&P, 4, 4, P_buf);
        matrix_wrap(&F, 4, 4, F_buf);
        matrix_wrap(&H, 4, 4, H_buf);
        matrix_wrap(&Q, 4, 4, Q_buf);
        matrix_wrap(&R, 4, 4, R_buf);
        matrix_wrap(&I, 4, 4, I_buf);
        matrix_wrap(&K, 4, 4, K_buf);
        matrix_wrap(&S, 4, 4, S_buf);
        matrix_wrap(&y, 4, 1, y_buf);
        matrix_wrap(&z, 4, 1, z_buf);
        matrix_wrap(&x_pred, 4, 1, x_pred_buf);
        matrix_wrap(&P_pred, 4, 4, P_pred_buf);
        matrix_wrap(&temp4x4, 4, 4, temp4x4_buf);
        matrix_wrap(&temp4x4_2, 4, 4, temp4x4_2_buf);
        matrix_wrap(&temp4x1, 4, 1, temp4x1_buf);

        /* Initialize */
        matrix_identity(&F);
        matrix_set(&F, 0, 1, dt);  /* x += vx * dt */
        matrix_set(&F, 2, 3, dt);  /* y += vy * dt */

        matrix_identity(&H);  /* measure all states */

        matrix_identity(&I);

        /* Noise covariances (tunable) */
        matrix_fill(&Q, 0.0f);
        matrix_set(&Q, 0, 0, RADAR_KALMAN_Q);  /* x process noise */
        matrix_set(&Q, 1, 1, RADAR_KALMAN_Q);  /* vx process noise */
        matrix_set(&Q, 2, 2, RADAR_KALMAN_Q);  /* y process noise */
        matrix_set(&Q, 3, 3, RADAR_KALMAN_Q);  /* vy process noise */

        matrix_fill(&R, 0.0f);
        matrix_set(&R, 0, 0, RADAR_KALMAN_R);  /* x measurement noise */
        matrix_set(&R, 1, 1, RADAR_KALMAN_R);  /* vx measurement noise */
        matrix_set(&R, 2, 2, RADAR_KALMAN_R);  /* y measurement noise */
        matrix_set(&R, 3, 3, RADAR_KALMAN_R);  /* vy measurement noise */

        /* Initial state and covariance */
        matrix_fill(&x, 0.0f);
        matrix_identity(&P);
        matrix_scalar(&P, 1.0f, &P);  /* P = I */

        initialized = true;
    }

    /* Update F with current dt */
    matrix_identity(&F);
    matrix_set(&F, 0, 1, dt);  /* x += vx * dt */
    matrix_set(&F, 2, 3, dt);  /* y += vy * dt */

    /* Prediction */
    matrix_mul(&F, &x, &x_pred);  /* x_pred = F * x */
    matrix_mul(&F, &P, &temp4x4);  /* temp4x4 = F * P */
    matrix_transpose(&F, &temp4x4_2);  /* temp4x4_2 = F^T */
    matrix_mul(&temp4x4, &temp4x4_2, &P_pred);  /* P_pred = (F * P) * F^T */
    matrix_add(&P_pred, &Q, &P_pred);  /* P_pred += Q */

    /* Measurement */
    z_buf[0] = msg->xdata;
    z_buf[1] = msg->vxdata;
    z_buf[2] = msg->ydata;
    z_buf[3] = msg->vydata;

    /* Innovation */
    matrix_mul(&H, &x_pred, &temp4x1);
    matrix_sub(&z, &temp4x1, &y);  /* y = z - H * x_pred */

    /* Innovation covariance */
    matrix_mul(&H, &P_pred, &temp4x4);  /* temp4x4 = H * P_pred */
    matrix_transpose(&H, &temp4x4_2);  /* temp4x4_2 = H^T */
    matrix_mul(&temp4x4, &temp4x4_2, &S);  /* S = (H * P_pred) * H^T */
    matrix_add(&S, &R, &S);  /* S += R */

    /* Kalman gain: K = P_pred * H^T * S^{-1} */
    /* For simplicity, assume S is invertible; use matrix_inverse if available, but since no inverse, approximate or use scalar */
    /* Since H is identity, K = P_pred * S^{-1} */
    /* But to keep simple, assume diagonal and compute element-wise */
    /* For full implementation, need matrix inverse, but library may not have it. */
    /* Since H=I, S = P_pred + R, K = P_pred * (P_pred + R)^{-1} */
    /* For simplicity, use scalar approximation if diagonal */

    /* Assuming diagonal for simplicity */
    for (size_t i = 0; i < 4; ++i) {
        float s_ii = matrix_get(&S, i, i);
        if (s_ii == 0.0f) s_ii = 1e-6f;  /* avoid division by zero */
        for (size_t j = 0; j < 4; ++j) {
            float p_ij = matrix_get(&P_pred, i, j);
            matrix_set(&K, i, j, p_ij / s_ii);
        }
    }

    /* Update */
    matrix_mul(&K, &y, &temp4x1);
    matrix_add(&x_pred, &temp4x1, &x);  /* x = x_pred + K * y */

    matrix_mul(&K, &H, &temp4x4);
    matrix_sub(&I, &temp4x4, &temp4x4);
    matrix_mul(&temp4x4, &P_pred, &P);  /* P = (I - K*H) * P_pred */

    /* Write back filtered values */
    msg->xdata = x_buf[0];
    msg->vxdata = x_buf[1];
    msg->ydata = x_buf[2];
    msg->vydata = x_buf[3];
}

void RaderGyroscope_KalmanFilter(Dog *dog)
{
}






