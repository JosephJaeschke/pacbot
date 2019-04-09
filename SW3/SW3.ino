#include <Wire.h>
#include "PID.h"
#include "Sensor.h"
#include "SoftwareSerial.h"

#define STBY 20
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
#define IRBL A7
#define IRF A1
#define IRBR A3
#define IRFR A2
#define IRFL A0
#define MPU 0x68
#define SPEED 30
#define CIRC 3.14159265359*38.5 //from two years ago
#define TICKSPROT 174 //ticks per rotation (from two years ago)

int16_t AcX=0,AcY=0,AcZ=0,GyX=0,GyY=0,GyZ=0,temperature=0;

Sensor front(IRF);
Sensor fr(IRFR);
Sensor fl(IRFL);
Sensor br(IRBR);
Sensor bl(IRBL);

PID enc(3.8,0.0,0.0);

volatile int leftCount=0, rightCount=0;
int prevR=0,prevL=0;

char tmp_str[7];

char* convert_int16_to_str(int16_t i)
{
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

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

float distance()
{
  float distR=abs((float)(rightCount-prevR)*CIRC/TICKSPROT);
  float distL=abs((float)(leftCount-prevL)*CIRC/TICKSPROT);
  return ((distR+distL)/2)/10;
}

void moveOne()
{
  digitalWrite(STBY,LOW);
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
  digitalWrite(STBY,HIGH);
  while(distance()<15.8)
  {
    short error=-leftCount+rightCount;
    Serial1.printf("Error: %d\n",error);
    float diff=enc.compute(error);
    int adjust = SPEED-diff;
    adjust = constrain(adjust,0,100);
    analogWrite(PWMB, adjust);
    delay(10);
  }
  digitalWrite(STBY,LOW);
  prevR=rightCount;
  prevL=leftCount;
  delay(500);
}

void turnCW()
{
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
  analogWrite(PWMB,SPEED);
  digitalWrite(STBY,HIGH);
  rightCount=0;
  leftCount=0;
  while(rightCount < 88 && leftCount > -88)//double check
  {
    Serial1.printf("Left Enc: %d Right Enc: %d \n", leftCount, rightCount);
  }
  digitalWrite(STBY,LOW);
  delay(100);
  rightCount=0;
  leftCount=0;
  prevR=0;
  prevL=0;
  delay(100);
}

void turnCCW()
{
	digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,HIGH);
  analogWrite(PWMB,SPEED);
  digitalWrite(STBY,HIGH);
  rightCount=0;
  leftCount=0;
  while(rightCount > -93 &&leftCount < 93)//double check
  {
    Serial1.printf("Left Enc: %d Right Enc: %d \n", leftCount, rightCount);
  }
  digitalWrite(STBY,LOW);
  delay(100);
  rightCount=0;
  leftCount=0;
  prevR=0;
  prevL=0;
  delay(100);
}

void halt()
{
	digitalWrite(STBY,LOW);
}

void sense()
{
  for(int i=0;i<10;i++)
  {
    front.filter(analogRead(front.pin));
    fr.filter(analogRead(fr.pin));
    fl.filter(analogRead(fl.pin));
    br.filter(analogRead(br.pin));
    bl.filter(analogRead(bl.pin));
  }
}

void readIMU()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);
  AcX=(Wire.read()<<8|Wire.read());    
  AcY=(Wire.read()<<8|Wire.read());  
  AcZ=Wire.read()<<8|Wire.read();  
  temperature=Wire.read()<<8|Wire.read();
  GyX=(Wire.read()<<8|Wire.read());  
  GyY=(Wire.read()<<8|Wire.read());  
  GyZ=(Wire.read()<<8|Wire.read());  
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
  Serial.begin(9600);
  Serial1.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  digitalWrite(13,HIGH);
  analogWrite(PWMA,SPEED);
  analogWrite(PWMB,SPEED);
  delay(200);
}

void loop()
{
  if (Serial1.available() > 0)
  {
    char xbeeIn = (char)Serial1.read();
    Serial.write(xbeeIn);
    switch (xbeeIn) 
    {
      case '1':
        Serial1.write("Moving Forward");
        moveOne();
        Serial1.write("Done \n");
        break;
      case '2':
        Serial1.write("Turning cw");
        turnCW();
        Serial1.write("Done \n");
        break;
      case '3':
        Serial1.write("Turning ccw");
        turnCCW();
        Serial1.write("Done \n");
        break;
    } 
  }
}
