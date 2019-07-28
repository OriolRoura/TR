#ifndef __GYRODATA__
#define __GYRODATA__
class GyroData {
  float timeData, gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
public:
  GyroData();
  
  void set(float timeData, float gyroX, float gyroY, float gyroZ, 
           float accelX, float accelY, float accelZ);
  void set(float data[]);
  void setTimeData(float data);
  void setGyroX(float data);
  void setGyroY(float data);
  void setGyroZ(float data);
  void setAccelX(float data);
  void setAccelY(float data);
  void setAccelZ(float data);
  
  void get(float data[]);
  float getTimeData();
  float getGyroX();
  float getGyroY();
  float getGyroZ();
  float getAccelX();
  float getAccelY();
  float getAccelZ();
};


#endif
