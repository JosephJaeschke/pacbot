#ifndef SENSOR_H
#define SENDOR_H 

class Sensor
{
  public:
  short pin;
  float ema_a;
  float ema_ema;
  float ema;
  float DEMA;
  float curr;
  
  Sensor(short);
  void filter(float);
  
  private:
  void ema_func(float);
  void ema_ema_func();
};

#endif
