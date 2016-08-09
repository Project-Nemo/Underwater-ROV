int ledDelay8 = 1000;
int ledDelay7 = 800;
int ledDelay6 = 250;
int ledDelay5 = 0;

int ledState8 = HIGH;
int ledState7 = HIGH;
int ledState6 = HIGH;
int ledState5 = HIGH;

unsigned long previousMicros8 = 0;
unsigned long previousMicros7 = 0;
unsigned long previousMicros6 = 0;
unsigned long previousMicros5 = 0;

void setup() {
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
}

void loop() {
  unsigned long currentMicros = micros();
  // FOR PIN 8
  changeLEDState(currentMicros, previousMicros8, ledDelay8, 8);
  digitalWrite(8, ledState8);
  // FOR PIN 7
  changeLEDState(currentMicros, previousMicros7, ledDelay7, 7);
  digitalWrite(7, ledState7);
//  // FOR PIN 6
  changeLEDState(currentMicros, previousMicros6, ledDelay6, 6);
  digitalWrite(6, ledState6);
//  // FOR PIN 5
  changeLEDState(currentMicros, previousMicros5, ledDelay5, 5);
  digitalWrite(5, ledState5);
}

void changeLEDState(unsigned long currentMicros, unsigned long previousMicros, int ledDelay, int pin){
  if (currentMicros - previousMicros >= ledDelay) {
    // save the last time you blinked the LED
    switch(pin){
      case 8:
        previousMicros8 = currentMicros;
        if (ledState8 == LOW) {
          ledState8 = HIGH;
        } else {
          ledState8 = LOW;
        } 
        ledDelay8 = 1000 - ledDelay8;
        break;
      case 7:
        previousMicros7 = currentMicros;
        if (ledState7 == LOW) {
          ledState7 = HIGH;
        } else {
          ledState7 = LOW;
        } 
        ledDelay7 = 1000 - ledDelay7;
        break;
      case 6:
        previousMicros6 = currentMicros;
        if (ledState6 == LOW) {
          ledState6 = HIGH;
        } else {
          ledState6 = LOW;
        } 
        ledDelay6 = 1000 - ledDelay6;
        break;
      case 5:
        previousMicros5 = currentMicros;
        if (ledState5 == LOW) {
          ledState5 = HIGH;
        } else {
          ledState5 = LOW;
        } 
        ledDelay5 = 1000 - ledDelay5;
        break;
    }
  }
}

