#include "PID.h"

PID::PID(float p, float i, float d)
{
	Kp=p;
	Ki=i;
	Kd=d;
	errSum=0;
	lastErr=0;
	last=0;
}

float PID::compute(short err)
{
	unsigned long now=millis();
	float timeChg=(float)now-(float)last;
	errSum+=err;
	float errD=(err-lastErr)/timeChg;
	float output=Kp*err+Ki*errSum+Kd*errD;
	lastErr=err;
	last=now;
	return output;
}
