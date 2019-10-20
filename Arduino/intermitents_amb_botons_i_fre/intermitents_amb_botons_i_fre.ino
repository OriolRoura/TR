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
                                  //es defineixen totes les constants
#define FALLO 1000                //es defineix la constant de marje per a donar error
#define POLSADOR 9                //es defineix la constant del polsador per a comeençar a prendre mostres
#define POLSDRET 4                //es defineix la constant del polsador per a actibar el llum intermitent dret
#define POLSESQUERR 12            //es defineix la constant del polsador per a actibar el llum intermitent esquerra
#define INTERRUPT_PIN 2           //es defineix la constant per a saber si el giroscop esta actibat o no
#define DRETA_PIN 5               //es defineix la constant del llum dret
#define ESQUERRA_PIN 7            //es defineix la constant del llum esquerra
#define BEL_PIN 3                 //es defineix la constant del llum de fre 
#define GRAUS 5                   //es defineix la constant del limit minim per a que es detecti un gir
#define GRAUS2 0.3                //es defineix la constant del limit minim per a que es mantingui un gir
#define GRAUS3 0.1                //es defineix la constant del limit mínim per a actibar el fre
#define INTER 500                 //es defineix la constant del temps de durada d'una intermitencia durant el funcionament dels intermitents
#define COINCIDENCIES 250         //es defineix la constant del nombre maxim de cicles que es fan donant un resultat semblant
#define INTERVAL 300              //es defineix la constant del temps que passa entre una presa de dades i una altre
#define ALFA 0.05                 //es defineix la constant de la formula de la mitjana de pas baix

                                  // s'inclueixen totes les llibreries utilitzades
 #include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include <Wire.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <SPI.h>
#include "PasBaix.h"

                                  // es defineixen totes les variables
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
bool polsDret = false;
bool polsEsquerr = false;
bool dretaEnces, esquerraEnces;
bool fre = false;
float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
float gyroXOld;
float gyroYOld;
float accelXOld;
float accelYOld;
float accelZOld;
float gyroOld;
PasBaix pbX(ALFA);
PasBaix pbY(ALFA);

MPU6050 mpu;


void checkSettings();
                                  //variables de control del sensor
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

                                  //vaiables d'orientació
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

  


uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indica quan hi ha hagut una interrupcio del MPU. Hi ha dades del giroscop disponibles
void dmpDataReady() {
    mpuInterrupt = true;
}



void setup() {
                                  //s'inicialitzen els pulsadors
  pinMode(POLSADOR,INPUT_PULLUP);
  pinMode(POLSDRET,INPUT_PULLUP);
  pinMode(POLSESQUERR,INPUT_PULLUP);
                                  //s'inicialit LES constants DE GIR
  pinMode(DRETA_PIN, OUTPUT);
  pinMode(ESQUERRA_PIN, OUTPUT);
                                  //s'inicialitza la constant INTERRUPT_PIN
   pinMode(INTERRUPT_PIN, INPUT);

                                
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

                                  //s'inicialitza  la comunicació entre el giroscop i l'arduino
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
                                  // s'inicialitza el MPU (una de les llibreries)
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    mpu.setRate (5);
    pinMode(INTERRUPT_PIN, INPUT);

                                  // es berifica la connecció
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

                                  // s'inicialitza el DPM
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

/*    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip*/

                                  // es calibra el giroscop
    if (devStatus == 0) {
        
        mpu.CalibrateAccel(15);
        mpu.CalibrateGyro (15);
        
                                  // s'encen el DPM
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

                            // detecta .............
                            // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

                                  //prepara el DPM per començar el loop
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

                                  //agafa la mezura dels paquets que seran embiats
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
                                  // intermitencia per a saber que l'inicialització ha acabat
  digitalWrite(DRETA_PIN,HIGH);
  digitalWrite(ESQUERRA_PIN,HIGH);

  delay(1000);
  digitalWrite(DRETA_PIN,LOW);
  digitalWrite(ESQUERRA_PIN,LOW);
}

void loop() {
                                  // si es pulsa el polsador...
 if (buttonPressed(POLSADOR)) { 
   
    if(pass == true){
      pass = false; 
    }
    else{
      pass = true;
    }
 }
                                  // si el giroscop no està funcionant finalitzem el programa
     if (!dmpReady) return;

                                  // Capturem les dades del giroscop
    
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
                                  // si  l'interruptor de inici s'ha apretat
 if(pass == true){
                                  // guardem el log en un String
  String dataString = "";
             mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            gyroX=ypr[0] * 180/M_PI;
            gyroY=ypr[1] * 180/M_PI;
            gyroZ=ypr[2] * 180/M_PI;


                                  // converteix les dades l'accelerometre i el giròscop
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            accelX=pbX.calc(aaWorld.x);
            accelY=pbY.calc(aaWorld.y);
            accelZ=aaWorld.z;



// ---------------------------------------------------------------
                                  // si es prem el polsador dret els intermitents drets s'encenen
    if (buttonPressed(POLSDRET)){
      polsDret=true;
      polsEsquerr=false;
    }
    if (polsDret==true){
      controlDreta=true;
    }
                                  // si es prem el polsador esquerra els intermitents esquerra s'encenen
    if (buttonPressed(POLSESQUERR)){
      polsEsquerr=true;
      polsDret=false;
    }
                                  // si es prem el polsador esquerra els intermitents esquerra s'encenen
    if (polsEsquerr==true){
      controlEsquerra=true;
    }
                                  //si en el accelX + accelXOld es major a la constant FALLO, accelY + accelYOld es major a la constant FALLO,
                                  //accelZ + accelZOld o es major a la constant FALLO, s'agafen les dades de l'anterior loop  de accelX, accelY i accelZ
    if( abs(accelX + accelXOld)>=FALLO and abs(accelY + accelYOld)>=FALLO and abs(accelZ + accelZOld)>=FALLO){
      accelX=accelXOld;
      accelY=accelYOld;
      accelZ=accelZOld;      
    }
                                  //arreglar el salt als 180º #3
    if(millis()>=millisOld+INTERVAL){
      millisOld=millis();
      if ( abs (gyroXOld-gyroX) >= 300 ){    
        gyroXOld = gyroX;
      }
      

                                  //es comproba si esta habent-hi un gir cap a la dreta
      if(gyroX > (gyroXOld+GRAUS)){
        polsDret =false;
        polsEsquerr=false;
        controlDreta = true;
        controlEsquerra = false;
        Iguals=0;
      }
      if(gyroX < (gyroXOld+GRAUS2)){
        Iguals++;
      }
                                  //es comproba si esta habent-hi un gir cap a l'esquerra
      if(gyroX < (gyroXOld - GRAUS)){
        polsEsquerr = false;
        polsDret =false;
        controlDreta = false;
        controlEsquerra = true;;
        Iguals=0;
      }
      gyroXOld=gyroX;
    }
      
    if(gyroX > (gyroXOld-GRAUS2)){
      Iguals++;
    }



                                  //es comproba si esta habent-hi un fre
     if((gyroY < gyroOld-GRAUS3) && (fre==true)){
      controlFre = true;
      da = 0;
    }
    if ((accelX>1000 ) && (accelY<-1000 )){
      fre=true;
      gyroYOld=gyroY;
    }
    else{
      fre=false;
    }
    
    if(Iguals>= COINCIDENCIES){
      controlDreta = false;
      controlEsquerra = false;
      Iguals=0;
    }
                                  //s'actualitzen les bariables: accelXOld, accelYOld, accelZOld per a el proccim cicle
    accelXOld=accelX;
    accelYOld=accelY;
    accelZOld=accelZ;
    
// --------------------------------------------------------------------------  
                                  //es coloquen totes les dades en un string
  dataString = String (millis())+";"+String(gyroX)+";"+String(gyroY)+";"+String(gyroZ)+";";
  dataString += String(accelX)+";"+String(accelY)+";"+String(accelZ);


                                  //control de l'intermitent dret

   Serial.print(gyroX);
   Serial.print("        ");
   Serial.print(gyroXOld);
   Serial.print("        ");
   Serial.print(polsEsquerr);
   Serial.print("        ");
   Serial.println(controlEsquerra);
    bool cc;
    if (cc=(dretaEnces == true || controlDreta==true )){      // hem posat la bariable cc= per que sino el compilador falla
      
        if (millis() >= dretaMillis + INTER ){
          if (dretaEnces == true){
            digitalWrite(DRETA_PIN,LOW);
            dretaEnces=false;
          } else { 
            digitalWrite(DRETA_PIN,HIGH);
            dretaEnces=true;

          }
          dretaMillis=millis();
        }
    } 
    
                                  //control de l'intermitent esquerra    
    
    if (controlEsquerra==true || esquerraEnces == true){
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


                                  //control del llum fre
    
    if (controlFre==true){
      digitalWrite(BEL_PIN,HIGH); 
    } else {
      digitalWrite(BEL_PIN,LOW); 
    }
 }
// delay(100);
}




                                  //defineix la barible putton pressed
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
