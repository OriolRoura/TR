#define BUTTON_PIN 9

unsigned int ctlButton = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(100);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (buttonPressed(BUTTON_PIN)) {
    Serial.println("Pressed");
  }
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
