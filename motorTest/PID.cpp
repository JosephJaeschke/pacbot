#include "PID.h"
#include <Arduino.h>

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
    unsigned long curr=millis();
    float timeChg=(float)curr-(float)last;
    errSum+=err;
    float errD=(err-lastErr)/timeChg;
    outPut=Kp*err+Ki*errSum*timeChg+Kd*errD;
    lastErr=err;
    last=curr;
    return outPut;
}
