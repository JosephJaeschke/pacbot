#include "PID.h"

#define STBY 19
#define AIN2 22
#define AIN1 23
#define BIN2 11
#define BIN1 12
#define LEOB 6
#define LEOA 5
#define REOA 8
#define REOB 7
#define PWMA 10
#define PWMB 9
#define SPEED 50

volatile int leftCount=0, rightCount=0;

PID enc(3.8,0.07,0.3);

void leftEncoderEvent() 
{
  if(digitalRead(LEOA)==HIGH)
  {
    if(digitalRead(LEOB)==LOW)
    {
      leftCount++;
    }
    else
    {
      leftCount--;
    }
  } 
  else
  {
    if(digitalRead(LEOB)==LOW)
    {
      leftCount--;
    }
    else 
    {
      leftCount++;
    }
  }
}

void rightEncoderEvent() 
{
  if(digitalRead(REOA)==HIGH)
  {
    if(digitalRead(REOB)==LOW)
    {
      rightCount++;
    }
    else
    {
      rightCount--;
    }
  } 
  else
  {
    if(digitalRead(REOB)==LOW)
    {
      rightCount--;
    }
    else
    {
      rightCount++;
    }
  }
}

void setup() 
{

  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  pinMode(STBY,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(LEOA,INPUT);
  pinMode(LEOB,INPUT);
  pinMode(REOA,INPUT);
  pinMode(REOB,INPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(LEOA),leftEncoderEvent,CHANGE);
  attachInterrupt(digitalPinToInterrupt(REOA),rightEncoderEvent,CHANGE);
}

void loop() 
{
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
  digitalWrite(STBY,HIGH);
  while(true)
  {
      short error=-leftCount+rightCount;
      Serial.printf("%d\n",error);
      float diff=enc.compute(error);
      int adjust=SPEED-diff;
      adjust=constrain(adjust,0,255);
      analogWrite(PWMB,adjust);
      delay(10);
  }
}
