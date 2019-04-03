#include "Sensor.h"

Sensor::Sensor(short p)
{
	pin=p;
	ema=0;
	ema_a=0.06;
	ema_ema=0;
	DEMA=0;
}

void Sensor::filter(float c)
{
	ema_func(c);
	ema_ema_func();
	DEMA=2*ema-ema_ema;
}

void Sensor::ema_func(float curr)
{
	ema=(ema_a*curr+(1-ema_a)*ema);
}

void Sensor::ema_ema_func()
{
	ema_ema=(ema_a*ema+(1-ema_a)*ema_ema);
}
