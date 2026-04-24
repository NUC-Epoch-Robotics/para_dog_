//
// Created by SlimeTommy on 2025/10/25.
//
#ifndef __CURVE_H
#define __CURVE_H
#include "stdint.h"
typedef struct
{
    float x;//mm
    float y;//mm
}bezierPoint;

typedef union {
	const bezierPoint  (*ctrl_pointc)[4];
	bezierPoint (*ctrl_point)[4];
}Bezier_ctrl_point;

bezierPoint bezierCurve(const bezierPoint (*node)[4],uint8_t n,float t);

/**
 * @brief 缩放贝塞尔曲线控制点
 * @param dest 目标数组，存储缩放后的控制点
 * @param src 源数组，原始控制点
 * @param count 控制点数量
 * @param scale_x X方向缩放比例
 * @param scale_y Y方向缩放比例（相对于中性y值）
 * @param neutral_y 中性y值，Y方向偏移以此为基准计算
 */
void bezierCurve_Scale(bezierPoint dest[], const bezierPoint src[], uint8_t count, 
                       float scale_x, float scale_y, float neutral_y);

#endif

