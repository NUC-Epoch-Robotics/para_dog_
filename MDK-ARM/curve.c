
#include "curve.h"
#include "stdint.h"
#include "math.h"

// 计算组合数 C(n, k)
float binomial_coefficient(uint8_t n, uint8_t k)
{
	double result = 1.0;
	for (uint8_t i = 1; i <= k; ++i)
	{
		result *= (n - i + 1) / (float)i;
	}
	return (float)result;
}

bezierPoint bezierCurve(const bezierPoint (*node)[4], uint8_t n, float t)
{
	if (n > 3)
	{
		n = 3;
	}
	if (t < 0.0f)
	{
		t = 0.0f;
	}
	else if (t > 1.0f)
	{
		t = 1.0f;
	}

	bezierPoint result = {0.0f, 0.0f};
	for (uint8_t k = 0; k <= n; k++)
	{
		float bino = binomial_coefficient(n, k);
		float term = bino * powf(t, k) * powf(1.0f - t, n - k);
		result.x += node[0][k].x * term;
		result.y += node[0][k].y * term;
	}
	return result;
}

















