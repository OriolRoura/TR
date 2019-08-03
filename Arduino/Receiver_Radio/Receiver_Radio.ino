
/*
* Getting Started example sketch for nRF24L01+ radios
* This is a very basic example of how to send data from one node to another
* Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"
#include "ComsStruct.h"

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(9,10);
//RF24 radio(4,10);
/**********************************************************/

byte addresses[][6] = {"1Node","2Node"};

ComsStruct cs;

void setup() {
  Serial.begin(115200);
  Serial.println(F("RF24/Receiver"));
  Serial.println(sizeof (cs));
  
  radio.begin();

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  
  // Start the radio listening for data
  radio.startListening();
}

void loop() {
  
    unsigned long got_time;
    double id;
    float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
    bool lightLeft, lightBreak, ligthRight;
    String dataString;

    
    if( radio.available()){
                                                                    // Variable for the received timestamp
      while (radio.available()) {                                   // While there is data ready
        radio.read(&cs, sizeof( cs) );             // Get the payload
      }
      cs.get(&id, &gyroX, &gyroY, &gyroZ, &accelX, &accelY, &accelZ, &lightLeft, &lightBreak, &ligthRight);
      dataString = String(id)+";"+String(gyroX)+";"+String(gyroY)+";"+String(gyroZ)+";"+String(accelX)+";"+String(accelY)+";"+String(accelZ)+";"+String(lightLeft)+";"+String(lightBreak)+";"+String(ligthRight);
      Serial.println(dataString);
 }






} // Loop
