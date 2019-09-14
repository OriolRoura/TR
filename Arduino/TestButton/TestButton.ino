#define BUTTON_PIN1 9
#define BUTTON_PIN2 10


unsigned int ctlButton = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(BUTTON_PIN1, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP);
  delay(100);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (buttonPressed(BUTTON_PIN1)) {
    Serial.println("Pressed 1");
  }
  if (buttonPressed(BUTTON_PIN2)) {
    Serial.println("Pressed 2");
  }
}


bool buttonPressed(unsigned short pin) {
  unsigned int btnStatus;
  if (pin > 15) {
     return false;
  };
  btnStatus = ctlButton & 1 << pin;
  if (btnStatus == 0) {
     if (digitalRead(pin) == LOW) {
       ctlButton = ctlButton | 1 << pin;
       delay(50);
       return true;
     }
  }
  else {
     if (digitalRead(pin) == HIGH) {
       ctlButton = ctlButton & (1 << pin ^ 0xFFFF);
       delay(50);
     };
  };
  return false;
};
