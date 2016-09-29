/*
  ROVPS2Control_Masterv8.ino
  Hamish Trolove - 30 March 2016
  www.techmonkeybusiness.com
  This sketch takes control commands from a PS2 handset and transmits the
  commands using Bill Porter's EasyTransfer Library over a 9600 baud serial
  link (100m tether).

  This sketch is designed for an Arduino Nano with only one Serial Port.

  Pin assignments are:

  3.3V output to PS2 red Pin
  Pin D10 to PS2 yellow pin
  Pin D11 to PS2 orange pin
  Pin D12 to PS2 brown pin
  Pin D13 to PS2 blue pin

  Pin D2 to LED Camera Photo Trigger Indicator
  Pin D3 to LED Camera Record Indicator
  Pin D4 to LED Main Lights Indicator
  Pin D5 to LED ROV Battery Low Voltage Warning
  Pin D6 to LED ROV Interior high temperature warning

  Communications
  Serial Connection: Topside D1 (TX) to ROV D0 (RX)
  Serial Connection: Topside D0 (RX) to ROV D1 (TX)
  Connect the GND on both

  A 16x2 LCD screen is connected as follows
  VSS to GND
  VDD to 5V output of MC78T05CT regulator
  VO to sweep arm of 10kohm variable resistor
  RS to Arduino Nano pin A0
  RW to GND
  E to Arduino Nano pin A1
  D4 to Arduino Nano pin A2
  D5 to Arduino Nano pin A3
  D6 to Arduino Nano pin A4
  D7 to Arduino Nano pin A5
  A to 5V output of MC78T05CT regulator
  K to GND via a 330ohm resistor

  5V is supplied from a regulator to the 1Kohm pull up resistors
  for PS2 as well as the LCD screen and it's backlight

  The coding pulls on the PSX library developed by Bill Porter.
  See www.billporter.info for the latest from Bill Porter and to
  download the library.

  The controls for the ROV are;
  Left Stick - X-axis = Roll, Y-axis = Up/down
  Right Stick - X-axis = Yaw, Y-axis = forward/back
  Direction button pad left = LED Main lights On/Off toggle
  Direction button pad up = turn camera upwards
  Direction button pad down = turn camera downwards
  Direction button pad right = Change reading on display
  Triangle = Start/Stop video recording
  Circle = Take photo
*/


#include <PS2X_lib.h> // Bill Porter's PS2 Library
#include <EasyTransfer.h> // Bill Porter's Easy Transfer Library
#include <LiquidCrystal.h>
#include <TVout.h>
#include <fontALL.h>

#define W 136          // on screen display width and height
#define H 96

TVout tv;
PS2X ps2x;  //The PS2 Controller Class
EasyTransfer ETin, ETout;  //Create the two Easy transfer Objects for
// Two way communication

LiquidCrystal lcd(A0, A1, A2, A3, A4, A5); //Pins for the LCD display

const int grnLEDpin = 4;  //green LED is on Digital pin 4
const int redLEDpin = 3;  //red LED is on Digital pin 3.
const int yelLEDpin = 2;  //yellow LED is on Digital pin 2
const int VwarnLEDpin = 5;  //Voltage warning LED is on Pin D5
const int TwarnLEDpin = 6;  //ROV temp warning LED is on Pin D6
const int LowBatVolts10 = 96;  //This is for holding the value of the
const int LowPodVolts10 = 96; 
//Low Battery Voltage warning Voltage threshold x10.

int ForwardVal = 0;  //Value read off the PS2 Right Stick up/down.
int YawLeftVal = 0;  //Value read off the PS2 Right Stick left/right
int UpVal = 0; //Value read off the PS2 Left Stick up/down
int RollLeftVal = 0; // Value read off the PS2 Left Stick left/right
float ROVTMP = 0;  //Variable to hold the converted ROV interior temperature.
int DispOpt = 0; //Variable to signal which value to show on the display

long PhotoSignalRunTime = 0; //A variable to carry the time since photo triggered.
volatile boolean PhotoActive = false;  // A flag to show that the camera signal has been sent.

unsigned char originx = 5;     // start x position for on screen display
unsigned char originy = 80;    // start y position for on screen display
unsigned char centrex = 60;
int linelen = 16;
float angledeg = -90.0 ;

struct RECEIVE_DATA_STRUCTURE {
  int BattVolt;  //Battery Voltage message from the ROV.
  int ROVTemp; //ROV interior temperature back from the ROV
  int ROVDepth; //ROV depth reading (m)
  int ROVHDG;  //ROV direction (Degrees)
  // accelerometer x, y, z values
  int AccX;
  int AccY;
  int AccZ;
  // gyroscope x, y, z values
  int GyroX;
  int GyroY;
  int GyroZ;
  int AccRoll;
  int AccPitch;
  int PodPower;
  int PodState;

  // for tuning PID
  int cP;
  int cD;
  int cI;
  int pidScale;
  int pidShift;
  int low_bound;
  int high_bound;

  String sensorData;
};

struct SEND_DATA_STRUCTURE {
  int upLraw;  //Variables to carry the actual raw data for the ESCs
  int upRraw;
  int HLraw;
  int HRraw;
  int CamPitch; //Angle of the camera servo.
  volatile boolean CamPhotoShot; // Camera photo trigger signal
  volatile boolean CamRec;  //Camera record function toggle
  volatile boolean LEDHdlts; //LED headlights on/off toggle

  // for tuning PID
  int cP;
  int cD;
  int cI;
  int pidScale;
  int pidShift;
  int low_bound;
  int high_bound;
  volatile boolean changed;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;

void setup() {
  ps2x.config_gamepad(13, 11, 10, 12, false, false);
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?)
  //We have disabled the pressure sensitivity and rumble in this instance and
  //we know the controller type so we have not bothered with the error checks
  pinMode(grnLEDpin, OUTPUT);  //Sets the grnLEDpin to output
  pinMode(redLEDpin, OUTPUT);  //Sets the redLEDpin to output
  pinMode(yelLEDpin, OUTPUT);  //Sets the yelLEDpin to output.
  pinMode(VwarnLEDpin, OUTPUT);  //Sets the low voltage warning pin to output
  pinMode(TwarnLEDpin, OUTPUT);  //Sets the overtemperature warning pin to output.
  txdata.CamRec = false;  //Sets the Camera default to not recording
  txdata.CamPhotoShot = false; //Sets the Camera default to no phototaken
  txdata.CamPitch = 90; //Sets the Camera Pitch to be level
  lcd.begin(16, 2);
  lcd.clear();  //make sure screen is clear.
  lcd.setCursor(0, 0); //Move cursor to top left corner
  lcd.print("Initialising");

  delay(10000);    //The 10 second delay to allow opportunity to upload new programs.
  Serial.begin(9600); //Begin Serial to talk to the Slave Arduino
  ETin.begin(details(rxdata), &Serial); //Get the Easy Transfer Library happening through the Serial
  ETout.begin(details(txdata), &Serial);
  lcd.clear();  //make sure screen is clear again.
  lcd.setCursor(0, 0); //Move cursor to top left corner
  lcd.print("Ready");

  // setup on screen display
  tv.begin(PAL, W, H);
  tv.delay(500);
  initOverlay();
  tv.select_font(font6x8);
  
}

void loop() {
  txdata.changed = false; // TODO: PID TUNING HELPER
  
  ps2x.read_gamepad(); //This needs to be called at least once a second
  // to get data from the controller.
  if (ps2x.Button(PSB_PAD_UP)) { //Pressed and held
    txdata.CamPitch = txdata.CamPitch + 2; //increase the camera pitch
  }

  if (ps2x.ButtonPressed(PSB_PAD_LEFT)) { //Pressed
    txdata.LEDHdlts = !txdata.LEDHdlts; //Toggle the LED light flag
  }


  if (ps2x.Button(PSB_PAD_DOWN)) { //Pressed and Held
    txdata.CamPitch = txdata.CamPitch - 2; //decrease the camera pitch
  }
  txdata.CamPitch = constrain(txdata.CamPitch, 20, 160); //Constrain the camera pitch
  //to within range servo can handle.

  if (ps2x.Button(PSB_PAD_RIGHT)) { //Pressed and Held
    DispOpt = DispOpt + 1; //step through the data to display.
    if (DispOpt == 2) { //At the moment there are only two items of
      //data to display.  This will need to be changed as extra data is added
      //This just resets the data to be displayed to the start of the list
      DispOpt = 0;
    }
  }

  if (ps2x.ButtonPressed(PSB_GREEN)) {//Triangle pressed
    txdata.CamRec = !txdata.CamRec; //Toggle the Camera recording Status
  }


  if (ps2x.ButtonPressed(PSB_RED)) { //Circle pressed
    txdata.CamPhotoShot = true;  //Set to indicate photo shot taken.
  }

  //Analogue Stick readings
  ForwardVal = ps2x.Analog(PSS_RY);
  YawLeftVal = ps2x.Analog(PSS_RX);
  UpVal = ps2x.Analog(PSS_LY);
  RollLeftVal = ps2x.Analog(PSS_LX);

  //Translate the Stick readings to servo instructions
  //Readings from PS2 Controller Sticks are from 0 to 255
  //with the neutral being 128.  The zero positions are to
  //the left for X-axis movements and up for Y-axis movements.

  //Variables to carry the actual raw data for the ESCs
  txdata.upLraw = (128 - UpVal) - (128 - RollLeftVal) / 2; //This will be up to a value of 192
  txdata.upRraw = (128 - UpVal) + (128 - RollLeftVal) / 2; //This will be up to a value of 192
  txdata.HLraw = -(128 - ForwardVal) + (128 - YawLeftVal); //This will be up to a value of 256
  txdata.HRraw = -(128 - ForwardVal) - (128 - YawLeftVal); //This will be up to a value of 256

  //Scale the values to be suitable for ESCs and Servos
  // These values will be able to be written directly to the ESCs and Servos
  txdata.upLraw = map(txdata.upLraw, -193, 193, 0, 179);
  txdata.upRraw = map(txdata.upRraw, -193, 198, 0, 179);
  txdata.HLraw = map(txdata.HLraw, -256, 256, 0, 179);
  txdata.HRraw = map(txdata.HRraw, -256, 256, 0, 179);


  
  // Send the message to the serial port for the ROV Arduino
  ETout.sendData();

  //Based on Bill Porter's example for the Two Way Easy Transfer Library
  //We will include a loop here to make sure the receive part of the
  //process runs smoothly.
  for (int i = 0; i < 5; i++) {
    ETin.receiveData();

    if (rxdata.BattVolt < LowBatVolts10) { //The factor of 10 is included to
      // match the factor of 10 used in the reported value which is an int multiplied
      //by 10 to give 0.1 precision to the value.  Make sense?
      digitalWrite(VwarnLEDpin, HIGH); //If the battery voltage too low,
      //trigger the warning LED
    } else {
      digitalWrite(VwarnLEDpin, LOW); //Otherwise if voltage above the
      //defined low voltage threshhold
      //leave the LED off.
    }
    ROVTMP = (rxdata.ROVTemp * 0.004882814 - 0.5) * 100; //converts the 0-1024
    //data value into temperature.
    if (ROVTMP > 50) {
      digitalWrite(TwarnLEDpin, HIGH); //If the Interior temp too high (over 50 degC),
      //trigger the warning LED
    } else {
      digitalWrite(TwarnLEDpin, LOW); //Otherwise if interior temperature within the
      //acceptable level, leave the LED off.
    }

    if (DispOpt == 1) {
      lcd.clear();  //A nice clean screen with no remnants from previous
      //messages.
      lcd.setCursor(0, 0); //Top left hand corner
      lcd.print("ROV Volts:");
      lcd.setCursor(0, 1); //Bottom left corner
      lcd.print("ROV Temp:");
      lcd.setCursor(11, 0);
      lcd.print(float(rxdata.BattVolt) / 10, 1); //factor of 10 used to get
      //extra precision from Integer value and then displayed to 1 decimal place.
      lcd.setCursor(11, 1);
      lcd.print(ROVTMP); // Display the ROV temperature
    } else {
      lcd.clear();  //A nice clean screen with no remnants from previous
      //messages.      lcd.setCursor(0,0); //Top left hand corner
      lcd.print("Depth:");
      lcd.setCursor(0, 1); //Bottom left corner
      lcd.print("Heading:");
      lcd.setCursor(11, 0);
      lcd.print(rxdata.ROVDepth); //Display ROV depth in metres
      lcd.setCursor(11, 1);
      lcd.print(rxdata.ROVHDG);  //Display ROV heading in degrees.
    }
    delay(18);
  }

  // Signalling the probable status of the camera using LEDs.

  if (txdata.CamPhotoShot && !PhotoActive) {
    PhotoSignalRunTime = millis();  //Set the start time for the signal
    digitalWrite(grnLEDpin, HIGH);
    PhotoActive = true;  //record that the photo has been triggered
  }
  if (txdata.CamPhotoShot && PhotoActive && millis() - PhotoSignalRunTime > 2000) { //See if the trigger
    // signal has been running for two seconds
    digitalWrite(grnLEDpin, LOW);
    txdata.CamPhotoShot = false; //Set the camera trigger to off
    PhotoActive = false;  // record that the photosignal has finished.
  }

  digitalWrite(redLEDpin, txdata.CamRec); //Light the redLED based on camera recording status flag
  digitalWrite(yelLEDpin, txdata.LEDHdlts); //Light the LED based on headlights status flag

  // onscreen display
  updateOnScreenDisplay();
  delay(18);

  // adjust PID values
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'x') {
      changeParams();
    }
  }
}

void changeParams(){
  txdata.changed = true;
  Serial.print("CD: ");
  Serial.println(rxdata.cD);
  while (!Serial.available()) {
  }
  txdata.cD = Serial.parseFloat();
  Serial.print("CI: ");
  Serial.println(rxdata.cI);
  while (!Serial.available()) {
  }
  txdata.cI = Serial.parseFloat();
  Serial.print("CP: ");
  Serial.print(rxdata.cP);
  while (!Serial.available()) {
  }
  txdata.cP = Serial.parseFloat();
  Serial.print("PIDScale ");
  Serial.print(rxdata.pidScale);
  while (!Serial.available()) {
  }
  txdata.pidScale = Serial.parseFloat();
  Serial.print("PIDShift ");
  Serial.print(rxdata.pidShift);
  while (!Serial.available()) {
  }
  txdata.pidShift = Serial.parseFloat();
  Serial.print("Low Bound ");
  Serial.print(rxdata.low_bound);
  while (!Serial.available()) {
  }
  txdata.low_bound = Serial.parseFloat();
  Serial.print("High Bound ");
  Serial.print(rxdata.high_bound);
  while (!Serial.available()) {
  }
  txdata.high_bound = Serial.parseFloat();
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

void updateOnScreenDisplay(){
  // write all display code in here
  tv.fill(0);
  drawAngle(rxdata.AccRoll);
  drawROVBattery(rxdata.BattVolt);
  drawPodBattery(rxdata.PodPower);
  ROVBattLow(rxdata.BattVolt);
  PodBattLow(rxdata.PodPower);
  ROVTempHigh(rxdata.ROVTemp);
}
  //--Artificial Horizon--//
void drawAngle(int angledata){
   //USE GYRO OR ROVHDG VARIABLE Need to see what unit is output to format for Angle assignment
  angledeg = angledata; //may need to modify this equation
  float angle = angledeg * PI / 180.0;
  tv.fill(0);
  drawGraph();
  if (angle == 0.0){
    tv.draw_line(centrex- linelen, originy, centrex+linelen, originy, 1);
  }else if (angle > 0.0){
    float x = sin(angle) * (double)linelen;
    float y = cos(angle) * (double)linelen;
    tv.draw_line(centrex + (int)x , originy + (int)y, centrex - (int)x, originy - (int)y, 1);
  }else if (angle < 0.0){
    float x = sin(abs(angle)) * (double)linelen;
    float y = cos(abs(angle)) * (double)linelen;
    tv.draw_line(centrex - (int)x, originy + (int)y, centrex + (int)x, originy - (int)y, 1);
  }
  angledeg =angledeg +1.0;
  if (angledeg > 90.0){
    angledeg = -90.0;
  }
}
  //--ROV Battery Update--//
void drawROVBattery(int BattVolt){
  tv.draw_rect(originx,15,10,5,1, -1);
  tv.draw_rect(originx,15,(int)BattVolt,5,1,1); // Will need to rescale BattVolt
}
  // --Pod Battery Update--//
void drawPodBattery(int PodPower){
  tv.draw_rect(originx,8,10,5,1, -1);
  tv.draw_rect(originx,8,(int)PodPower,5,1,1); // Will need to rescale PodPower
}

// if rov battery low, print message// 
void ROVBattLow(int BattVolt){
  
  if (BattVolt < LowBatVolts10) {
      tv.print(20,20, "ROV BATTERY LOW");
    } else {
      tv.print(20,20, "");
  }
}
  // if pod battery low, print message//
  
void PodBattLow(int PodPower){
  if (PodPower < LowPodVolts10) { //Change LowBatVolts10
      tv.print(20,30, "POD BATTERY LOW");
    } else {
      tv.print(20,30, "");
  }
}
  // if temp to high, print message//
void ROVTempHigh(int Temp){
  ROVTMP = (Temp * 0.004882814 - 0.5) * 100; //converts the 0-1024
    //data value into temperature.
    if (ROVTMP > 50) {
      tv.print(20,40, "ROV TEMP HIGH");
    } else {
      tv.print(20,40, "");
    }
  }
    
void drawGraph() {
  tv.draw_line(originx, originy, 120, originy, 1);
  tv.draw_circle(60, originy, 15, WHITE, -1);
}
