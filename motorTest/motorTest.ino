#define LEOA 5
#define LEOB 6
#define REOA 8
#define REOB 7

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

void setup() 
{
  /*
  pinMode(23,OUTPUT);
  pinMode(22,OUTPUT); //Right Motor
  pinMode(10,OUTPUT);
  
  pinMode(9,OUTPUT);
  pinMode(11,OUTPUT); //Left Motor
  pinMode(12,OUTPUT);
  
  pinMode(19,OUTPUT);
  */
  pinMode(13,OUTPUT);
  pinMode(LEOA,INPUT);
  pinMode(LEOB,INPUT);
  pinMode(REOA,INPUT);
  pinMode(REOB,INPUT);
  attachInterrupt(digitalPinToInterrupt(LEOA),leftEncoderEvent,CHANGE);
  attachInterrupt(digitalPinToInterrupt(REOA),rightEncoderEvent,CHANGE);
}

void loop() 
{
  digitalWrite(13,HIGH);
  delay(200);
  Serial.printf("%d %d\n",leftCount,rightCount);
  digitalWrite(13,LOW);
  delay(100);
}
