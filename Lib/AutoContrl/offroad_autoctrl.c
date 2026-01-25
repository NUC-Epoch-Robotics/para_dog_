#include "offroad_autoctrl.h"
#include "dog.h"
#include "math.h"
#include "cmsis_os.h"
#define rad
#define cm
#define frame -1//场地左右手坐标系切换
auto_ctrlcenter dog_auto_ctrl[10]={
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=Tracking},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=Tracking},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=Tracking},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=Tracking},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=Tracking},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=Tracking},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=Tracking},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=Tracking},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=Tracking},.task_complete_flag=0},
    {.target.target_location={0.0f,0.0f,0.0f},.task_attr={.node=track_startpoint,.task_type=Tracking},.task_complete_flag=0}
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
    if(dog->auto_ctrl.task_attr.task_type==Tracking){
        AutoTracking(dog);
    }
    else if(dog->auto_ctrl.task_attr.task_type==Action){
        AutoAction(dog);
    }
}








