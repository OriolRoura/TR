#include <Arduino.h>
#include "GyroData.h"
#include "Debug.h"

GyroData::GyroData() {
#ifdef DEBUG
    Serial.println("Entra GyroData");
#endif  
  timeData=gyroX=gyroY=gyroZ=accelX=accelY=accelZ=0;
#ifdef DEBUG
    Serial.println("Surt GyroData");
#endif
};

void GyroData::set(float _timeData, float _gyroX, float _gyroY, float _gyroZ, 
                   float _accelX, float _accelY, float _accelZ) {
  timeData = _timeData;
  gyroX = _gyroX;
  gyroY = _gyroY;
  gyroZ = _gyroZ;
  accelX = _accelX;
  accelY = _accelY;
  accelZ = _accelZ;
};

void GyroData::set(float data[]) {
  setTimeData(data[0]);
  setGyroX(data[1]);
  setGyroY(data[2]);
  setGyroZ(data[3]);
  setAccelX(data[4]);
  setAccelY(data[5]);
  setAccelZ(data[6]);
}

void GyroData::get(float data[]) {
  data[0] = getTimeData();
  data[1] = getGyroX();
  data[2] = getGyroY();
  data[3] = getGyroZ();
  data[4] = getAccelX();
  data[5] = getAccelY();
  data[6] = getAccelZ();
  
}; 

void GyroData::setTimeData(float data) {
  timeData = data;
};

void GyroData::setGyroX(float data) {
  gyroX = data;
};

void GyroData::setGyroY(float data) {
  gyroY = data;
};

void GyroData::setGyroZ(float data) {
  gyroZ = data;
};

void GyroData::setAccelX(float data) {
  accelX = data;
};

void GyroData::setAccelY(float data) {
  accelY = data;
};

void GyroData::setAccelZ(float data) {
  accelZ = data;
};

float GyroData::getTimeData() {
  return timeData;
};

float GyroData::getGyroX() {
  return gyroX;
};

float GyroData::getGyroY() {
  return gyroY;
};

float GyroData::getGyroZ() {
  return gyroZ;
};

float GyroData::getAccelX() {
  return accelX;
};

float GyroData::getAccelY() {
  return accelY;
};

float GyroData::getAccelZ() {
  return accelZ;
};
