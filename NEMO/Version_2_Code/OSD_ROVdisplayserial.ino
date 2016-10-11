#include <TVout.h>
#include <fontALL.h>
#include <video_gen.h>

// OSD chip cannot handle variables, so need to use defines
#define W 136    // width of screen for osd
#define H 96     // height of screen for osd

#define HORIZON_START_X 5   // start x position for on screen display
#define HORIZON_START_Y 80  // start y position for on screen display
#define HORIZON_LENGTH 120
#define TOP_X 2
#define TOP_Y 8
#define BATT_LENGTH 10
#define BATT_HEIGHT 4
#define LINE_LENGTH 16
#define CENTER_X 60

TVout tv;   // On Screen Display screen variable

char * data = "          ";

// constants
int LowBattVolts = 96;
int LowPodVolts = 96;
int HighTemp = 50;

// Data to show on display
int angledeg = 0;
int pod_volts = 0;
int batt_volts = 0;
int temp = 0;
int depth = 0;

void setup() {
  Serial.begin(9600);
  setupOSD();
} 



void loop() {
  delay(1000);
  if(Serial.available() > 1) {
    angledeg = Serial.read();
    angledeg = angledeg - 90; // Value sent will be 90 degress greater than actual angle
    //Serial.println(angle, DEC);
    pod_volts = Serial.read();
    //Serial.println(pod_volts, DEC);
    batt_volts = Serial.read();
//    //Serial.println(batt_volts, DEC);
//    temp = Serial.read();
//    //Serial.println(temp, DEC);
//    depth = Serial.read();
//    //Serial.println(depth, DEC);
  }
  tv.fill(0); 
  drawGraph();
  displayROVDepth();
  displayROVTempHigh();
  displayHorizon();
  displayROVBatteryData();
  displayPodBatteryData();
}


int16_t readInt() {
  return (Serial.read() << 8) | Serial.read();
}

void setupOSD() {
  tv.begin(PAL, W, H);
  tv.delay(500);
  initOverlay();
  tv.select_font(font4x6);
}

// Initialize ATMega registers for video overlay capability.
// Must be called after tv.begin().
void initOverlay() {
  TCCR1A = 0;
  // Enable timer1.  ICES0 is set to 0 for falling edge detection on input capture pin.
  TCCR1B = _BV(CS10);
  // Enable input capture interrupt
  TIMSK1 |= _BV(ICIE1);
  // Enable external interrupt INT0 on pin 2 with falling edge.
  EIMSK = _BV(INT0);
  EICRA = _BV(ISC01);
}

// Required to reset the scan line when the vertical sync occurs
ISR(INT0_vect) {
  display.scanLine = 0;
}

void displayHorizon(){  
  float angle = (angledeg/180.0) *PI;
  int y = sin(abs(angle)) * LINE_LENGTH;
  int x = cos(abs(angle)) * LINE_LENGTH;
  if (angledeg == 0){
    drawLine(CENTER_X - LINE_LENGTH, HORIZON_START_Y, CENTER_X + LINE_LENGTH, HORIZON_START_Y);
  } else if (angledeg > 0){   
    drawLine(CENTER_X + x , HORIZON_START_Y + y, CENTER_X - x, HORIZON_START_Y - y);
  } else if (angledeg < 0){   
    drawLine(CENTER_X - x, HORIZON_START_Y + y, CENTER_X + x, HORIZON_START_Y - y);
  }
}

// ROV Battery Update
void displayROVBatteryData() {
  printMsg(TOP_X, TOP_Y, "ROV");
  drawBattery(22, TOP_Y, BATT_LENGTH, BATT_HEIGHT);
  fillBattery(22, TOP_Y, batt_volts, BATT_HEIGHT); 

  // if ROV battery low, print message
//  if (batt_volts < LowBattVolts) {
//    printMsg(44, TOP_Y + 8, "ROV BATTERY LOW");
//  }
}

// Pod Battery Update
void displayPodBatteryData() {
  printMsg(TOP_X, TOP_Y + 8, "POD");
  drawBattery(22, TOP_Y + 8, BATT_LENGTH, BATT_HEIGHT);
  fillBattery(22, TOP_Y + 8, pod_volts, BATT_HEIGHT); 
 
  // if pod battery low, print message
//  if (pod_volts < LowPodVolts) { 
//    tv.print(44, top_y, "POD BATTERY LOW");
//  }
}

// If temp to high, print message
void displayROVTempHigh() {
 //printMsg(TOP_X, TOP_Y + 16, "TEMP");
//  char * msg = "        ";
//  sprintf(msg, "%d", temp);
//  printMsg(20, TOP_Y + 16, msg); 
 // if (temp > HighTemp) {
  //  printMsg(44, TOP_Y + 16, "ROV TEMP HIGH");
  //}
}

void displayROVDepth(){
  printMsg(TOP_X, TOP_Y + 24, "DEPTH:");
//  char * msg = "        ";
//  sprintf(msg, "%d", depth);
//  printMsg(24, TOP_Y + 24, msg);       
}

void drawGraph() {
  drawLine(HORIZON_START_X, HORIZON_START_Y, HORIZON_LENGTH, HORIZON_START_Y);
  drawCircle(60, HORIZON_START_Y, 15);
}

void drawBattery(int x0, int y0, int x1, int y1) {
  tv.draw_rect(x0, y0, x1, y1, 1, 0);  // x, y, width, height, no fill
}

void fillBattery(int x0, int y0, int x1, int y1){
  tv.draw_rect(x0, y0, x1, y1, 1, 1);  // x, y, width, height, fill
}

void drawLine(int x0, int y0, int x1, int y1){
  tv.draw_line(x0, y0, x1, y1, 1);
}

void drawCircle(int x, int y, int radius){
   tv.draw_circle(x, y, radius, WHITE, -1);
}

void printMsg(int x, int y, char * msg){
  tv.print(x, y, msg);
}

