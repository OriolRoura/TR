/*
  SD card datalogger

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <SD.h>

#define POLSADOR 9

bool pass = false;
void checkSettings();

MPU6050 mpu;

const int chipSelect = 4;

long tiempo_prev, dt;

float XAxis_prev, YAxis_prev, ZAxis_prev;
float gyrosc_x, gyrosc_y, gyrosc_z;

void setup() {
  pinMode(POLSADOR,INPUT_PULLUP);
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))//inicialitzaciÃ³ del giroscop
  {
    Serial.println("no s'ha trobat el sensor!");
    delay(500);
  }
  

  checkSettings();

  Serial.print("inisialitzant SD.");

  // inicialitzaciÃ³ de la SD i comprovacio que estÃ  colÂ·locada
  if (!SD.begin(chipSelect)) {
    Serial.println("error en la carraga de la targeta");
    // don't do anything more:
    return;
  }
  Serial.println("targeta inisialitzada.");
}

void loop() {
 if (buttonPressed(POLSADOR)) { 
    if(pass == true){
      pass = false; 
    }
    else{
      pass = true;
      tiempo_prev=millis();
    }
 }
 if(pass == true){
  // guardem el log en un String
  String dataString = "";

  // read accel and gyr:
  //Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();
  Vector rawGyro = mpu.readRawGyro();
  //Vector normGyro = mpu.readNormalizeGyro();

  dt = millis()-tiempo_prev;
  tiempo_prev=millis();
  //Serial.println(rawGyro.XAxis);
  gyrosc_x = (rawGyro.XAxis)*dt/1000.0+XAxis_prev;
  gyrosc_y = (rawGyro.YAxis)*dt/1000.0+YAxis_prev;
  gyrosc_z = (rawGyro.ZAxis)*dt/1000.0+ZAxis_prev;
  XAxis_prev=gyrosc_x;
  YAxis_prev=gyrosc_y;
  ZAxis_prev=gyrosc_z;
  
   //dataString = String(rawAccel.XAxis)+";"+String(rawAccel.YAxis)+";"+String(rawAccel.ZAxis)+";"+String(normAccel.XAxis)+";"+String(normAccel.YAxis)+";"+String(normAccel.ZAxis);
  
  dataString = String (millis())+";"+String(gyrosc_x)+";"+String(gyrosc_y)+";"+String(gyrosc_z)+";";
  dataString += String(normAccel.XAxis)+";"+String(normAccel.YAxis)+";"+String(normAccel.ZAxis);
   

  // obrim el fitxer de daades
  File dataFile = SD.open("datalog.csv", FILE_WRITE);

  // guardem les dades en el fitxer
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error obrint datalog.txt");
  }
 }
 delay(100);
}


//mostra de la configuraciÃ³ del giroscop
void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  
  Serial.print(" * Accelerometer:         ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("2 g"); break;
  }
  switch(mpu.getScale())
  {   
    case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
    case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
  }
  mpu.setAccelOffsetX(-813);
  mpu.setAccelOffsetY(-452);
  mpu.setAccelOffsetZ(889);
  Serial.print("offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  
  Serial.print(" * Gyroscope offsets: ");
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getGyroOffsetZ());
  
  Serial.println();
}

bool buttonPressed(int pin) {
   if(digitalRead(pin) == LOW){
    while(digitalRead(pin) == LOW){
      delay(100);
    }
    return true;
   } else {
    return false;
   }
}
