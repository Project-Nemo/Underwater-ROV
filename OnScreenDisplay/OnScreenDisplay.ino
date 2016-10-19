#include <TVout.h>
#include <fontALL.h>
#include <video_gen.h>
#include <avr/pgmspace.h>

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

const char rov[] PROGMEM  = "ROV";
const char pod[] PROGMEM  =  "POD";
const char depth[] PROGMEM  = "DEPTH";
const char temp[] PROGMEM  = "TEMP:";
const char rovbattlow[] PROGMEM  ="ROV BATTERY LOW";
const char podbattlow[] PROGMEM  ="POD BATTERY LOW";
const char* const string_table[] PROGMEM = {rov, pod, temp,depth, rovbattlow, podbattlow}; //To move variables to SRAM 
char buffer[15];

TVout tv;   // On Screen Display screen variable

// Data to show on display
int temp_val = 0;
int depth_val = 0;
int angledeg = 0;
int pod_volts = 0;
int batt_volts = 0;

void setup() {
  Serial.begin(9600);
  setupOSD();
  delay(1000);
} 

void loop() {
 Serial.end();
 Serial.begin(9600);
 delay(1000);
  if(Serial.available() > 1) { 
    angledeg = Serial.read();
    pod_volts = Serial.read();
    batt_volts = Serial.read();
    temp_val = Serial.read();
    depth_val = Serial.read();
  }  
  tv.fill(0); 
  displayHorizon();
  displayROVDepth();
  displayROVTemp();
  displayBatteryData();
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
  tv.draw_line(HORIZON_START_X, HORIZON_START_Y, HORIZON_LENGTH, HORIZON_START_Y, 1);
  tv.draw_circle(60, HORIZON_START_Y, 15, WHITE, -1);
  angledeg = angledeg - 90; // Value sent will be 90 degress greater than actual angle
  float angle = (angledeg/180.0) *PI;
  int x = cos(abs(angle)) * LINE_LENGTH;
  int y = sin(abs(angle)) * LINE_LENGTH;
  if (angledeg == 0){
    tv.draw_line(CENTER_X - LINE_LENGTH, HORIZON_START_Y, CENTER_X + LINE_LENGTH, HORIZON_START_Y, 1);
  } else if (angledeg < 0){   
    tv.draw_line(CENTER_X + x , HORIZON_START_Y + y, CENTER_X - x, HORIZON_START_Y - y, 1);
  } else if (angledeg > 0){   
    tv.draw_line(CENTER_X - x, HORIZON_START_Y + y, CENTER_X + x, HORIZON_START_Y - y, 1);
  }
}

// ROV Battery Update
void displayBatteryData() {
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[0]))); //Retrieving variables and copying to buffer
  tv.print(TOP_X, TOP_Y, buffer);
  drawBattery(22, TOP_Y, BATT_LENGTH, BATT_HEIGHT);
  fillBattery(22, TOP_Y, batt_volts, BATT_HEIGHT); 
  
  // if ROV battery low, print message
  if (batt_volts <= 2) {     
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
    tv.print(44, TOP_Y + 8, buffer);
  }
  
 // Pod Battery Update
 strcpy_P(buffer, (char*)pgm_read_word(&(string_table[1])));
  tv.print(TOP_X, TOP_Y + 8, buffer);
  drawBattery(22, TOP_Y + 8, BATT_LENGTH, BATT_HEIGHT);
  fillBattery(22, TOP_Y + 8, pod_volts, BATT_HEIGHT); 
  
  if (pod_volts <= 1) { 
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[5])));
    tv.print(44, TOP_Y + 8, buffer);
  }
}

void displayROVTemp() {
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[2])));
  tv.print(TOP_X, TOP_Y + 16, buffer);
  tv.print(24, TOP_Y + 16, temp_val); 
}

void displayROVDepth(){
 strcpy_P(buffer, (char*)pgm_read_word(&(string_table[3])));
 tv.print(TOP_X, TOP_Y + 24, buffer);
 tv.print(28, TOP_Y + 24, depth_val);       
}

void drawBattery(int x0, int y0, int x1, int y1) {
  tv.draw_rect(x0, y0, x1, y1, 1, 0);  // x, y, width, height, no fill
}

void fillBattery(int x0, int y0, int x1, int y1){
  tv.draw_rect(x0, y0, x1, y1, 1, 1);  // x, y, width, height, fill
}

