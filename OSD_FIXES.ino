#include <TVout.h>
#include <fontALL.h>
#include <video_gen.h>

TVout tv;
char * data = "          ";


void setup() {
  Serial.begin(9600);
  setupOSD();
  doSomething("Test");
}



void loop() {
  delay(1000);
  /*if(Serial.available()) {
    char * position = data;
    while(Serial.available()) {
      *position++ = Serial.read();
    }
    *position = '\0';
    
    doSomething(data);
  }*/

  if(Serial.available() > 1) {
    //int number = Serial.read();
    //int number1 = Serial.read();
    sprintf(data, "%d", readInt());
    doSomething(data);
  }
}


int16_t readInt() {
  return (Serial.read() << 8) | Serial.read();
}

/*
int16_t writeInt(int16_t number) {
  Serial.write(number >> 8);
  Serial.write(number & 0xFF);
}
*/

void setupOSD() {
  tv.begin(PAL, 136, 96);
  tv.delay(500);
  initOverlay();
  tv.select_font(font6x8);
}

void doSomething(char * outputString) {
  tv.fill(0);
  tv.print(30, 8, outputString);
}

