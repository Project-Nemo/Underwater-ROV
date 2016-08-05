int ledDelay = 10;
unsigned long previousMicros = 0;
int ledState = LOW;

void setup() {
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
}

void loop() {
  digitalWrite(8, HIGH);
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= ledDelay) {
    // save the last time you blinked the LED
    previousMicros = currentMicros;
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
      ledDelay = 10;
    } else {
      ledState = LOW;
      ledDelay = 1000 - ledDelay;
    } 
  }
  digitalWrite(7, ledState);
}
