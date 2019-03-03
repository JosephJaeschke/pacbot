#ifndef PID_H
#define PID_H

class PID
{
 private:
 float Kp,Ki,Kd,outPut;
 short errSum,lastErr;
 unsigned long last;
  
 public:
 PID(float,float,float);
 float compute(short);
};

#endif
