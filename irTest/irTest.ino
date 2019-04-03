#include "Sensor.h"

#define IRBL A4
#define IRF A1
#define IRBR A3
#define IRFR A2
#define IRFL A0

Sensor front(IRF);
Sensor fr(IRFR);
Sensor fl(IRFL);
Sensor br(IRBR);
Sensor bl(IRBL);

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  Serial.begin(9600);
}

void loop() {
  front.filter(analogRead(front.pin));
  fr.filter(analogRead(fr.pin));
  fl.filter(analogRead(fl.pin));
  br.filter(analogRead(br.pin));
  bl.filter(analogRead(bl.pin));
  Serial.printf("%.2f | %.2f | %.2f | %.2f | %.2f\n",front.DEMA,fr.DEMA,fl.DEMA,br.DEMA,bl.DEMA); 
  delay(200);
}
