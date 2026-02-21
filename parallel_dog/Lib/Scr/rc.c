//
// Created by SlimeTommy on 25-10-27.
//
// remote_control.c
#include "rc.h"
#include <string.h>
#include "bsp_sbus.h"
extern int16_t g_sbus_channels[18];
void rc_remote_ctrl(Dog *dog){
	SBUS_Handle();
	if (g_sbus_channels[5]==192){
			dog->state=DAMPING_MODE;

		}
	if(g_sbus_channels[4]==1792){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
	}
	else if(g_sbus_channels[5]==1792&&g_sbus_channels[6]==1792&&g_sbus_channels[7]==992){
		if(g_sbus_channels[2]>=1100){
			dog->state=JUMP_FORWARD;
		}
		else if(g_sbus_channels[4]==192){
			dog->state=INJUMP;
		}
		else{
			dog->state=STAND_UP_; 
		}
	
	}
	else if(g_sbus_channels[5]==1792&&g_sbus_channels[6]==1792&&g_sbus_channels[7]==1792){
		if(g_sbus_channels[2]>=1200){
			dog->state=WALK_FORWARD;
			
		}
		else if (g_sbus_channels[2]<=900){
			dog->state=WALK_BACK;
		}
		else if (g_sbus_channels[0]<=900){
			dog->state=TURN_LEFT;
		}
		else if (g_sbus_channels[0]>=1100){
			dog->state=TURN_RIGHT;
		}
		else{
			dog->state=STAND_UP_;
		}
	}
	


}


