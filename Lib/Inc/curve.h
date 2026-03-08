//
// Created by SlimeTommy on 2025/10/25.
//
#ifndef __CURVE_H
#define __CURVE_H
#include "stdint.h"
typedef struct
{
    float x;
    float y;
}bezierPoint;

typedef union {
	const bezierPoint  (*ctrl_pointc)[4];
	bezierPoint (*ctrl_point)[4];
}Bezier_ctrl_point;

bezierPoint bezierCurve(const bezierPoint (*node)[4],uint8_t n,float t);







#endif


