#include "CircularList.h"
#include <stdlib.h>
#include <Arduino.h>
#include "Debug.h" 

  CircularList::CircularList() {
    CircularList(MAXLIST);
  };
  
  CircularList::CircularList(int n) {
    int i;

#ifdef DEBUG
    Serial.println("Entra CircularList ");
#endif
    maxList = n;
    gList = new GyroData [maxList];
    GyroData* g;
//    *gList =  malloc(sizeof(GyroData*)*maxList);

    count = current = lastId = 0;
//    *gList =  malloc(sizeof(GyroData*)*maxList);
//    for (i=0; i < maxList; i++) {
//      gList[i]= new GyroData();
//    }
#ifdef DEBUG
    Serial.println("maxList ");
    Serial.println(maxList);
#endif
#ifdef DEBUG
    Serial.println("Surt CircularList ");
#endif
  };
  
  CircularList::~CircularList() {
    int i;
/*
    for (i=0; i < maxList; i++) {
      delete gList[i];
    };
*/
    delete  [] gList;
  };
  
  void CircularList::add (GyroData gd) {
    add(gd.getTimeData(), gd.getGyroX(), gd.getGyroY(), gd.getGyroZ(), 
        gd.getAccelX(), gd.getAccelY(), gd.getAccelZ());

  };

  void CircularList::add (float td, float gx, float gy, float gz, float ax, float ay, float az) {
    gList[current].setTimeData(td);
    gList[current].setGyroX(gx);
    gList[current].setGyroY(gy);
    gList[current].setGyroZ(gz);
    gList[current].setAccelX(ax);
    gList[current].setAccelY(ay);
    gList[current].setAccelZ(az);
    lastId = current;
    if (count < 10) { count++;};
    current = (++current % 10);
#ifdef DEBUG  
    Serial.print("count ");
    Serial.println(count);
    Serial.print("current ");
    Serial.println(current);
#endif
    
  }

  bool CircularList::last(GyroData* gd) {
    if (count == 0) {
      return false;
    };
    gd->setTimeData(gList[lastId].getTimeData());
    gd->setGyroX(gList[lastId].getGyroX());
    gd->setGyroY(gList[lastId].getGyroY());
    gd->setGyroZ(gList[lastId].getGyroZ());
    gd->setAccelX(gList[lastId].getAccelX());
    gd->setAccelY(gList[lastId].getAccelY());
    gd->setAccelZ(gList[lastId].getAccelZ());
    
    return true;
  };

  void CircularList::erase() {
    int i;

    for (i=0; i < maxList; i++) {
      gList[i].set (0, 0, 0, 0, 0, 0, 0);
      count = current = lastId = 0;
    };
    
  };

  void CircularList::mean(GyroData *gd) {
    int i;
    int countAux = count;
    float sumgx, sumgy, sumgz, sumax, sumay, sumaz;
    sumgx = sumgy = sumgz = sumax = sumay = sumaz= 0.0;
    
#ifdef DEBUG  
    Serial.print("count ");
    Serial.println(count);
//    Serial.print("sumgy 0");
//    Serial.println(sumgy);
#endif  
    for (i=0; i < count; i++) {
      sumgx += gList[i].getGyroX();
      sumgy += gList[i].getGyroY();
#ifdef DEBUG  
    Serial.print("sumgy ");
    Serial.println(sumgy);
#endif  
      sumgz += gList[i].getGyroZ();
      sumax += gList[i].getAccelX();
      sumay += gList[i].getAccelY();
      sumaz += gList[i].getAccelZ();      
    }
    if (count == 0) {
      countAux = 1;
    }
//    gd->setTimeData(gList[lastId]->getTimeData());
    gd->setTimeData(countAux);
    gd->setGyroX(sumgx/countAux);
    gd->setGyroY(sumgy/countAux);
    gd->setGyroZ(sumgz/countAux);
    gd->setAccelX(sumax/countAux);
    gd->setAccelY(sumay/countAux);
    gd->setAccelZ(sumaz/countAux);  
  };

  
