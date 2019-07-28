#include "GyroData.h"
#include "CircularList.h"
#define MAXLIST 10

  CircularList cl(MAXLIST);
  GyroData *gd = new GyroData();


void setup(){
  // put you setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Inici HOLA");
  Serial.println(MAXLIST);
  int i;

  for (i=0; i< 30; i++) {
    cl.add(1*i, 2*i, 3*i, 4*i, 5*i, 6*i, 7*i);
  }

//  cl.last(gd);
  cl.mean(gd);
  Serial.println(gd->getTimeData());
  Serial.println(gd->getGyroX());
  Serial.println(gd->getGyroY());
  Serial.println(gd->getGyroZ());
  Serial.println(gd->getAccelX());
  Serial.println(gd->getAccelY());
  Serial.println(gd->getAccelZ());

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10000);
}
