//
// Created by SlimeTommy on 2025/10/25.
//
#include "curve.h"
#include "math.h"



// 计算组合数 C(n, k)
float binomial_coefficient(uint8_t n, uint8_t k) {
    double result1 = 1.0;
    for (int i = 1; i <= k; ++i) { 
        result1 *= (n - i + 1) / (float)i;
    }
    return result1;
}
bezierPoint bezierCurve(bezierPoint const (*node)[4],uint8_t n,float t)
{
	if(n>3){
		n=3;
	}
	bezierPoint result;
	float bino=0;
	float term=0;
    for(uint8_t k=0;k<=n;k++)
    {
        bino = binomial_coefficient(n,k);
		term = bino*pow(t,k)*pow(1-t,n-k);
        result.x  += node[0][k].x*term;
        result.y  += node[0][k]. y*term;
    }
    return result;
}

//bezierPoint node[29][4]={//单腿bezier动作节点(x,y)
//	{{0,190.53},{-56,127},{ 56,127},{ 99,214}},
//	
//	{{0 ,190.53f},{12.5,100.0f},{25,190.53f},{100,190.53f}},	//STEP_FORE1
//	{{0,190.53},{-12.5,200.0f},{-25,190.53},{ 99,214}},		//KICK_BACK1
//	{{-25,190.53f},{-30,100.0f},{30,100.0f},{25,190.53f}},	//STEP_FORE2
//	{{25,190.53},{0,200.0f},{-25,190.53f},{ 99,214}},			//KICK_BACK2
//	
//	{{0 ,190.53f},{-12.5,100.0f},{-25,190.53f},{-100,190.53f}},	//STEP_BACK1
//	{{0,190.53},{12.5,200.0f},{25,190.53},{ 99,214}},				//KICK_DOWN1
//	{{25,190.53f},{30,100.0f},{-30,100.0f},{-25,190.53f}},		//STEP_BACK2
//	{{-25,190.53},{0,200.0f},{25,190.53f},{ 99,214}},				//KICK_DOWN2
//	{{0 ,190.53f},{12.5,100.0f},{25,190.53f},{100,190.53f}},		//TURNSTEP_FORE1
//	{{0,190.53},{-12.5,200.0f},{-25,190.53},{ 99,214}},			//TURNKICK_BACK1
//	{{-25,190.53f},{-30,100.0f},{30,100.0f},{25,190.53f}},		//TURNSTEP_FORE2
//	{{25,190.53},{0,200.0f},{-25,190.53f},{ 99,214}},				//TURNKICK_BACK2
//	
//	{{0 ,180.53f},{12.5,100.0f},{25,180.53f},{100,180.53f}},		//LOWSTEP_FORE1
//	{{0,180.53},{-12.5,200.0f},{-25,180.53},{ 99,214}},			//LOWKICK_BACK1
//	{{-25,180.53f},{-30,100.0f},{30,100.0f},{25,180.53f}},		//LOWSTEP_FORE2
//	{{25,180.53},{0,200.0f},{-25,180.53f},{ 99,214}},				//LOWKICK_BACK2
//	
//	{{-50,140},{-50,140},{0,0},{-0,0}}, 			// JUMP_PREPARE
//	{{-50,140},{-114.285f,320.0f},{0,0},{0,0}}, 		// JUMPING
//	{{-114.285f,320.0f},{75,210},{0,0},{0,0}},
//	{{75,210},{50,140},{0,0},{0,0.0}},//JUMP_BUFFER
//	
//	{{0,130},{0,0},{0,0},{-0,0}}, 			// INJUMP_PREPARE
//	{{0,320},{0,320},{35,279.79f},{70,100}}, 		// INJUMP_TAKEOFF
//	{{0,320},{0,130},{0,100.0},{0,0.0}}, 	// INJUMP_LANDING
//	
//	{{0 ,190.53f},{-12.5,100.0f},{-25,190.53f},{100,190.53f}},	//STEP_FORE1
//	{{0,190.53},{12.5,200.0f},{25,190.53},{ 99,214}},		//KICK_BACK1
//	{{25,190.53f},{30,100.0f},{-30,100.0f},{-25,190.53f}},	//STEP_FORE2
//	{{-25,190.53},{0,200.0f},{25,190.53f},{ 99,214}},			//KICK_BACK2
//	
//	{{0,190.53},{-25,200.0f},{-50,190.53},{ 99,214}},
//};
//uint8_t bezierDimension[29]={	
//		0,//STAND_UP_
//		2,2,3,2,//WALK_FORWARD
//		2,2,3,2,2,2,3,2,//TURN_RIGHT / TURN_LEFT
//		2,2,3,2,//LOWWALK_FORWARD
//		0,1,1,1,//JUMP_FORWARD
//		0,0,1,//INJUMP
//		2,2,3,2,//WALK_BACK
//	0
//};//曲线阶数
//float bezierFre[29]={	
//		1,//STAND_UP_
//		100 ,100 ,100 ,100,  //WALK_FORWARD
//		100,100,100,100,  //TURN_RIGHT
//		100,100,100,100,  //TURN_LEFT
//		50,50,50,50,//LOWWALK_FORWARD
//		1,50,50,50,//JUMP_FORWARD
//		1,1,50,//INJUMP
//		50 ,50 ,50 ,50,//WALK_BACK
//		1
//		};//取样频率
//float bezierT[29]  ={
//		1,//STAND_UP_
//		0.17,0.17,0.17,0.17,//WALK_FORWARD
//		0.15,0.15,0.15,0.15,//TURN_RIGHT
//		0.15,0.15,0.15,0.15,//TURN_LEFT
//		0.3,0.3,0.3,0.3, //LOWWALK_FORWARD	
//		1,0.1,0.1,0.2,//JUMP_FORWARD
//		1,1,0.2,//INJUMP
//		0.3,0.3,0.3,0.3,//WALK_BACK
//	1
//		};//s



















