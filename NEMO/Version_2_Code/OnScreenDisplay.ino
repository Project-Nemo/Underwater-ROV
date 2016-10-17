#include <TVout.h>
#include <fontALL.h>
#include <video_gen.h>

#define W 136    // width of screen for osd
#define H 96     // height of screen for osd

TVout tv;   // On Screen Display screen variable

// constants
int LowBattVolts = 96;
int LowPodVolts = 96;
int HighTemp = 50;

// On scren display params for displaying values
int horizon_start_x = 5;     // start x position for on screen display
int horizon_start_y = 80;    // start y position for on screen display
int centre_x = W/2;
int top_x = 2;
int top_y = 2;
int batt_length_x = 10;
int batt_length_y = 4;
int line_length = 16;

// Data to show on display
int angle = 0;
int pod_volts = 0;
int batt_volts = 0;
int temp = 0;
int depth = 0;

void setup() {
  Serial.begin(9600);
  setupOSD();
}

void loop() {
  delay(2000);
  if(Serial.available() > 0) {
    angle = Serial.read();
    angle = angle - 90; // Value sent will be 90 degress greater than actual angle
    Serial.println(angle, DEC);
    pod_volts = Serial.read();
    Serial.println(pod_volts, DEC);
    batt_volts = Serial.read();
    Serial.println(batt_volts, DEC);
    temp = Serial.read();
    Serial.println(temp, DEC);
    int depth = Serial.read();
    Serial.println(depth, DEC);

    updateOnScreenDisplay();
  }
  delay(2000);
}

// setup on screen display
void setupOSD() {
  tv.begin(PAL, W, H);
  tv.delay(500);
  initOverlay();
  tv.select_font(font4x6);
  tv.fill(0);
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

void updateOnScreenDisplay() {
  tv.fill(0);
  drawGraph();
  displayHorizon();
  displayROVBatteryData();
  displayPodBatteryData();
  displayROVTempHigh();
  displayROVDepth();
}

void displayHorizon(){  
  int x = sin(abs(angle)) * line_length;
  int y = cos(abs(angle)) * line_length;
   if (angle == 0.0){
    tv.draw_line(centre_x - line_length, horizon_start_y, centre_x + line_length, horizon_start_y, 1);
  } else if (angle > 0.0){   
    tv.draw_line(centre_x + x , horizon_start_y + y, centre_x - x, horizon_start_y - y, 1);
  } else if (angle < 0.0){   
    tv.draw_line(centre_x - x, horizon_start_y + y, centre_x + x, horizon_start_y - y, 1);
  }
}

// ROV Battery Update
void displayROVBatteryData() {
  tv.print(top_x, top_y, "ROV");
  drawBattery(22, top_y, batt_length_x, batt_length_y);
  tv.draw_rect(22, top_y, batt_volts, 4, 1, 1); 

  // if ROV battery low, print message
  if (batt_volts < LowBattVolts) {
    tv.print(44, top_y + 8, "ROV BATTERY LOW");
  }
}

// Pod Battery Update
void displayPodBatteryData() {
  int diff_y = 8;
  tv.print(top_x, top_y + diff_y, "POD");
  drawBattery(22, top_y + diff_y, batt_length_x, batt_length_y);
  tv.draw_rect(22, top_y + diff_y, pod_volts, batt_length_x, 1, 1); 
  
  // if pod battery low, print message
  if (pod_volts < LowPodVolts) { 
    tv.print(44, top_y, "POD BATTERY LOW");
  }
}

// If temp to high, print message
void displayROVTempHigh() {
  int diff_y = 16;
  tv.print(top_x, top_y + diff_y, "TEMP");
  tv.print(20, top_y + diff_y, temp); 
  if (temp > HighTemp) {
    tv.print(44, top_y + diff_y, "ROV TEMP HIGH");
  }
}

void displayROVDepth(){
  int diff_y = 24;
  tv.print(top_x, top_y + diff_y, "DEPTH");
  tv.print(24, top_y + diff_y, depth);       
  tv.print(32, top_y + diff_y, "m");
}

void drawGraph() {
  tv.draw_line(horizon_start_x, horizon_start_y, 120, horizon_start_y, 1);
  tv.draw_circle(60, horizon_start_y, 15, WHITE, -1);
}

void drawBattery(int x0, int y0, int x1, int y1) {
  tv.draw_rect(x0, y0, x1, y1, 1, 0);  // x, y, width, height, outline colour, fill
}
