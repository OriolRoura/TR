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
 #include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include <Wire.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <SPI.h>

#define FALLO 1000
#define RL -500
#define BEL_PIN 3
#define POLSADOR 9
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define DRETA_PIN 5
#define ESQUERRA_PIN 7
#define GRAUS 5
#define GRAUS2 0.3
#define INTER 500
#define COINCIDENCIES 250
#define INTERVAL 50
int Iguals;
int da;
unsigned int ctlButton = 0;
long esquerraMillis, dretaMillis=0;
long millisOld;
bool primeraDreta, primeraEsquerra;
bool pass = false;
bool controlDreta = false;
bool controlEsquerra = false;
bool controlFre = false;
bool dretaEnces, esquerraEnces;

MPU6050 mpu;


void checkSettings();

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

  float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

float gyroXOld;
float accelXOld;
float accelYOld;
float accelZOld;
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

const int chipSelect = 4;


void setup() {
  pinMode(DRETA_PIN, OUTPUT);
   pinMode(INTERRUPT_PIN, INPUT);
  pinMode(POLSADOR,INPUT_PULLUP);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

 
  // Open serial communications and wait for port to open:
//  Serial.begin(9600);
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    mpu.setRate (5);
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        
        mpu.CalibrateAccel(15);
        mpu.CalibrateGyro (15);
        
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
 digitalWrite(DRETA_PIN,HIGH);
delay(1000);
  digitalWrite(DRETA_PIN,LOW);
}

void loop() {
 if ( buttonPressed(POLSADOR)) { 
    if(pass == true){
      pass = false; 
    }
    else{
      pass = true;
    }
 }

     if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

    }
 if(pass == true){
  // guardem el log en un String
  String dataString = "";
             mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            gyroX=ypr[0] * 180/M_PI;
            gyroY=ypr[1] * 180/M_PI;
            gyroZ=ypr[2] * 180/M_PI;


            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            accelX=aaWorld.x;
            accelY=aaWorld.y;
            accelZ=aaWorld.z;



// ---------------------------------------------------------------  
    if( abs(accelX + accelXOld)>=FALLO and abs(accelY + accelYOld)>=FALLO and abs(accelZ + accelZOld)>=FALLO){
      accelX=accelXOld;
      accelY=accelYOld;
      accelZ=accelZOld;      
    }
    
    if(millis()>=millisOld+INTERVAL){
      millisOld=millis();
      if ( abs (gyroXOld-gyroX) >= 300 ){     //arreglar el salt als 180º #3
        gyroXOld = gyroX;
      }
      
      if(gyroX > (gyroXOld+GRAUS)){
        controlDreta = true;
        controlEsquerra = false;
        Iguals=0;
      }
      if(gyroX < (gyroXOld+GRAUS2)){
        Iguals++;
      }
  
      if(gyroX < (gyroXOld - GRAUS)){
        controlDreta = false;
        controlEsquerra = true;;
        Iguals=0;
      }
      gyroXOld=gyroX;
    }
      
    if(gyroX > (gyroXOld-GRAUS2)){
      Iguals++;
    }
    if (accelX<= (accelXOld-RL)){
      controlFre = true;
      da = 0;
    }
    if(accelX >= (accelXOld-RL)){
      da++;
    }
    if(da >=   COINCIDENCIES){
      controlFre = false;
      da = 0;
    }
    
    if(Iguals>= COINCIDENCIES){
      controlDreta = false;
      controlEsquerra = false;
      Iguals=0;
    }
    
    accelXOld=accelX;
    accelYOld=accelY;
    accelZOld=accelZ;
// --------------------------------------------------------------------------  
  dataString = String (millis())+";"+String(gyroX)+";"+String(gyroY)+";"+String(gyroZ)+";";
  dataString += String(accelX)+";"+String(accelY)+";"+String(accelZ);

   Serial.print(gyroX);
   Serial.print("        ");
   Serial.println(gyroXOld);
  

//control dret intermitent

    if (controlDreta==true){
      if (primeraDreta==true){    
        primeraDreta = false;
        dretaMillis=millis();     
        digitalWrite(DRETA_PIN,HIGH);
        dretaEnces=true; 
      } else {
        if (millis() >= dretaMillis + INTER){   
          if (dretaEnces==true){
            digitalWrite(DRETA_PIN,LOW);
            dretaEnces=false; 
          } else {
            digitalWrite(DRETA_PIN,HIGH);
            dretaEnces=true; 
          }
          dretaMillis=millis();  
        }
      }
      
    } else {
      primeraDreta = true;
      digitalWrite(DRETA_PIN,LOW);
    }
    
//control intermitent esquerra    
    
    if (controlEsquerra==true){
      if (primeraEsquerra==true){    
        primeraEsquerra = false;
        esquerraMillis=millis();     
        digitalWrite(ESQUERRA_PIN,HIGH);
        esquerraEnces=true; 
      } else {
        if (millis() >= esquerraMillis + INTER){   
          if (esquerraEnces==true){
            digitalWrite(ESQUERRA_PIN,LOW);
            esquerraEnces=false; 
          } else {
            digitalWrite(ESQUERRA_PIN,HIGH);
            esquerraEnces=true; 
          }
          esquerraMillis=millis();  
        }
      }
    }

//control llum fre
    
    if (controlFre==true){
      digitalWrite(BEL_PIN,HIGH); 
    } else {
      digitalWrite(BEL_PIN,LOW); 
    }
 }
// delay(100);
}



bool buttonPressed(unsigned short pin) {
  int btnStatus;
  if (pin > 31) {
     return false;
  };
  btnStatus = ctlButton & 1 << pin;
  if (btnStatus == 0) {
     if (digitalRead(pin) == LOW) {
       ctlButton = ctlButton | 1 << pin;
       delay(100);
       return true;
     }
  }
  else {
     if (digitalRead(pin) == HIGH) {
       ctlButton = ctlButton & (1 << pin ^ 0xFFFF);
     };
  };
  return false;
};
