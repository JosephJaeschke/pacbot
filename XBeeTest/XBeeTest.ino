void setup()
{
  Serial1.begin(115200);
  Serial.begin(9600);
  pinMode(13,OUTPUT);
}

void loop()
{
  if(Serial1.available()>0)
  {
    char in=(char)Serial1.read();
    Serial.print(in);
    if(in=='0')
    {
      digitalWrite(13,LOW);
    }
    else if(in=='1')
    {
      digitalWrite(13,HIGH);
    }
    delay(200);
  }
}

