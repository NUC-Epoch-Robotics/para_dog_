#include "pathc.h"
#include "ReadData.h"
#include "dog.h"
void path_calculate(Dog *dog)
{
	static const path_point p[1]={
		{0,5}
	};
	float m,n;
	vcp_message_t msg;
	if(msg.xdata==0&&msg.ydata!=5)
	{
			dog->state=WALK_FORWARD;   
	}
}
