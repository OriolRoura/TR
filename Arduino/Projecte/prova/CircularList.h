#include "GyroData.h"

#ifndef __CIRCULARLIST__
#define _CIRCULARLIST__
#define MAXLIST 10

class CircularList {
  int count;
  int maxList;
  int current;
  int lastId;

  GyroData* gList = 0;
  public:  
    CircularList();
    CircularList(int n);
    ~CircularList();
    void add (GyroData gd);
    void add (float td, float gx, float xy, float xz, float ax, float ay, float az);
    bool last(GyroData *gd);
    void erase();
    void mean(GyroData *gd);
   
  
  
};

#endif
