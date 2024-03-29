

                                  // s'inclueixen totes les llibreries utilitzades
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include <Wire.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <SPI.h>
#include "RF24.h"
#include "comsStruct.h"
#include "PasBaix.h"

ComsStruct cs;
                                  //es defineixen totes les constants
#define RL 2000                   //es defineix la constant del nombre maxim de cicles que es fan donant un resultat semblant en el fre
#define FALLO 1000                //es defineix la constant de marje per a donar error
#define POLSADOR 9                //es defineix la constant del polsador per a comeençar a prendre mostres
#define INTERRUPT_PIN 2           //es defineix la constant per a saber si el giroscop esta actibat o no
#define GRAUS 5                   //es defineix la constant del limit minim per a que es detecti un gir
#define GRAUS2 0.3                //es defineix la constant del limit minim per a que es mantingui un gir
#define GRAUS3 0.1                //es defineix la constant del limit mínim per a actibar el fre
#define COINCIDENCIES 250         //es defineix la constant del nombre maxim de cicles que es fan donant un resultat semblant en el gir
#define INTERVAL 300              //es defineix la constant del temps que passa entre una presa de dades i una altre
#define ALFA 0.05                 //es defineix la constant de la formula de la mitjana de pas baix
#define DRETA_PIN 5               //es defineix la constant del llum dret
#define ESQUERRA_PIN 7            //es defineix la constant del llum esquerra


                                  // es defineixen totes les variables
unsigned int ctlButton = 0;
long millisOld;
bool pass = false;
bool controlDreta = false;
bool controlEsquerra = false;
bool controlFre = false;
bool fre = false;
int Iguals;
int da;
float gyroXOld;
float gyroYOld;
float accelXOld;
float accelYOld;
float accelZOld;
float gyroOld;
PasBaix pbX(ALFA);
PasBaix pbY(ALFA);

MPU6050 mpu;

RF24 radio(4,10);
byte addresses[][6] = {"1Node","2Node"};

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
  double checksum;




// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

const int chipSelect = 4;


void setup() {

                                  //s'inicialitza la constant INTERRUPT_PIN
    pinMode(INTERRUPT_PIN, INPUT);
                                    //s'inicialitza el pulsador
  pinMode(POLSADOR,INPUT_PULLUP);


  
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

    // supply your own gyro offsets here, scaled for min sensitivity
    /*mpu.setXGyroOffset(12);
    mpu.setYGyroOffset(172);
    mpu.setZGyroOffset(55);
    mpu.setZAccelOffset(889); // 889 factory default for my test chip

    // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
        mpu.CalibrateAccel(15);
        mpu.CalibrateGyro (15);*/
        
                                  // es calibra el giroscop

      if (devStatus == 0) {
        
        mpu.CalibrateAccel(15);
        mpu.CalibrateGyro (15);

        
                                  // s'encen el DPM
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
                                  //s'inicialitza la conneció radio amb la placa receptora
    radio.begin();
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);

                                  // intermitencia per a saber que l'inicialització ha acabat
  digitalWrite(DRETA_PIN,HIGH);
  digitalWrite(ESQUERRA_PIN,HIGH);

  delay(1000);
  digitalWrite(DRETA_PIN,LOW);
  digitalWrite(ESQUERRA_PIN,LOW);


}

                                  // si es pulsa el polsador...
void loop() {
  
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
        controlDreta = true;
        controlEsquerra = false;
        Iguals=0;
      }
      if(gyroX < (gyroXOld+GRAUS2)){
        Iguals++;
      }
  
                                  //es comproba si esta habent-hi un gir cap a l'esquerra
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
    
    if(accelY >= (accelYOld-RL)){
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
    
                                  //s'actualitzen les bariables: accelXOld, accelYOld, accelZOld per a el proccim cicle
    accelXOld=accelX;
    accelYOld=accelY;
    accelZOld=accelZ;
// --------------------------------------------------------------------------  
    
                                  //es coloquen totes les dades en un string
    cs.set (millis(),gyroX,gyroY,gyroZ,accelX,accelY,accelZ,controlEsquerra,controlFre,controlDreta); 
                                  //si el tamany es massa gran, escriu...
    if (!radio.write( &cs,sizeof(cs))){
      Serial.println(F("failed"));
    }  
  }    
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
