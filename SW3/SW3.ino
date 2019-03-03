#include "PID.h"
#include "Sensor.h"

#define STBY 4
#define AIN2 5
#define AIN1 6
#define BIN2 7
#define BIN1 8
#define LEOB 9
#define LEOA 10
#define REOA 11
#define REOB 12
#define PWMA A0
#define PWMB A1
#define IRBL A2
#define IRF A3
#define IRBR A4
#define IRFR A5
#define IRFL A6
#define SPEED 50

Sensor front(0,0);
Sensor fr(0,0);
Sensor fl(0,0);
Sensor br(0,0);
Sensor bl(0,0);

PID enc(0,0,0);

volatile int leftCount=0, rightCount=0;

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

void move()
{
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
  analogWrite(PWMA,SPEED);
  analogWrite(PWMB,SPEED);
  digitalWrite(STBY,HIGH);
}

void turnCW()
{
	digitalWrite(STBY,LOW);
	digitalWrite(AIN1,HIGH);
	digitalWrite(AIN2,LOW);
	digitalWrite(BIN1,LOW);
	digitalWrite(BIN2,HIGH);
	long encCount=leftCount;
	digitalWrite(STBY,HIGH);
	while(leftCount-encCount<86) //check!!!
	{
	}
	digitalWrite(STBY,LOW);
}

void turnCCW()
{
	digitalWrite(STBY,LOW);
	digitalWrite(BIN1,HIGH);
	digitalWrite(BIN2,LOW);
	digitalWrite(AIN1,LOW);
	digitalWrite(AIN2,HIGH);
	long encCount=rightCount;
	digitalWrite(STBY,HIGH);
	while(rightCount-encCount<86) //check!!!
	{
	}
	digitalWrite(STBY,LOW);
}

void stop()
{
	digitalWrite(STBY,LOW);
}

void sense()
{
	for(short i=0;i<10;i++)
	{
		front.update(analogRead(IRF));
		fr.update(analogRead(IRFR));
		fl.update(analogRead(IRFL));
		br.update(analogRead(IRBR));
		bl.update(analogRead(IRBL));
	}
	
}

void IR_Debug()
{
	Serial.printf(
}

void setup()
{
  pinMode(13,OUTPUT);
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
  pinMode(IRBL,INPUT);
  pinMode(IRF,INPUT);
  pinMode(IRBR,INPUT);
  pinMode(IRFR,INPUT);
  pinMode(IRFL,INPUT);
  attachInterrupt(digitalPinToInterrupt(LEOA),leftEncoderEvent,CHANGE);
  attachInterrupt(digitalPinToInterrupt(REOA),rightEncoderEvent,CHANGE);
  PID enc(
  Serial.begin(9600);
}

void loop()
{
  //Serial.printf("%d, %d\n",leftCount,rightCount);
  //Serial.printf("%d\n",analogRead(IRFL));
  digitalWrite(13,HIGH);
  delay(200);
  digitalWrite(13,LOW);
  delay(400);
}
