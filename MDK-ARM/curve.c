
#include "curve.h"
#include "stdint.h"
#include "math.h"
// 计算组合数 C(n, k)
float binomial_coefficient(int n, int k) {
    double result = 1.0;
    for (int i = 1; i <= k; ++i) {
        result *= (n - i + 1) / (float)i;
    }
    return result;
}



bezierPoint bezierCurve(bezierPoint *node,uint8_t n,uint8_t t)
{	
	bezierPoint result={0,0};
	for(uint8_t k=0;k<=n;k++)
	{
		float bino = binomial_coefficient(n,k);
		float term = bino*pow(t,k)*pow(1-t,n-k);
		result.x  += node[k].x*term;
		result.y  += node[k].y*term;
	} 
	return result;
}

















