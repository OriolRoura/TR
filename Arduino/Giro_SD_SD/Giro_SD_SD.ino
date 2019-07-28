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
#include <GyroData.h>

#define POLSADOR 9

bool pass = false;
void checkSettings();

MPU6050 mpu;


const int chipSelect = 4;

void setup() {
  pinMode(POLSADOR,INPUT_PULLUP);
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))//inicialització del giroscop
  {
    Serial.println("no s'ha trobat el sensor!");
    delay(500);
  }
  

  checkSettings();

  Serial.print("inisialitzant SD.");

  // inicialització de la SD i comprovacio que està col·locada
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
    }
 }
 if(pass == true){
  // guardem el log en un String
  String dataString = "";

  // read accel and gyr:
  //Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();
  //Vector rawGyro = mpu.readRawGyro();
  Vector normGyro = mpu.readNormalizeGyro();
  
  //dataString = String(rawAccel.XAxis)+";"+String(rawAccel.YAxis)+";"+String(rawAccel.ZAxis)+";"+String(normAccel.XAxis)+";"+String(normAccel.YAxis)+";"+String(normAccel.ZAxis);
  dataString = String (millis())+";"+String(normGyro.XAxis)+";"+String(normGyro.YAxis)+";"+String(normGyro.ZAxis);
  dataString += String(normAccel.XAxis)+";"+String(normAccel.YAxis)+";"+String(normAccel.ZAxis)+";";



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


//mostra de la configuració del giroscop
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
  mpu.setAccelOffsetX(0);
  mpu.setAccelOffsetY(0);
  mpu.setAccelOffsetZ(0);
  Serial.print("offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  mpu.setGyroOffsetX(0);
  mpu.setGyroOffsetY(0);
  mpu.setGyroOffsetZ(0);
  
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
