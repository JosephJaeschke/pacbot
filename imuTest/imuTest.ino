#include <Wire.h>

int16_t AcX=0,AcY=0,AcZ=0,GyX=0,GyY=0,GyZ=0,temperature=0;

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i)
{ // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
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

  Serial.begin(9600);
  Serial1.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  digitalWrite(13,HIGH);
  delay(200);
}

void loop()
{

  readIMU();
 
  Serial.print("aX = "); Serial.print(convert_int16_to_str(AcX));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(AcY));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(AcZ));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(GyX));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(GyY));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(GyZ));
  Serial.print("\n");
  delay(333);
}