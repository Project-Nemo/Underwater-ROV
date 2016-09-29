/*
  ROVPS2Control_Slavev8.ino
  Hamish Trolove - 30 March 2016
  www.techmonkeybusiness.com
  This sketch takes commands sent to it from the Master unit with
  the PS2 Controller attached and converts it to motor commands,
  servo commands, light controls etc.  The data is sent from
  the handset (Master) to the ROV(Slave) using Bill Porter's EasyTransfer
  Library over a 9600 baud serial link (100m tether).
  The MS5803_14 library is from Luke Miller http://github.com/millerlp

  Data sent from the Master are raw settings for the ESC control.

  This sketch is designed for an Arduino Nano with only one Serial Port.

  The pin assignments are;
  D13 = RED LED pin.
  D12 = Headlight Control
  D11 = Jumper pin
  D8 = ESC Vertical Left
  D7 = ESC Vertical Right
  D6 = ESC Horizontal Left
  D5 = ESC Horizontal Right
  D4 = Camera Pitch Servo
  D3 = Video Trigger
  D2 = Photo Trigger

  A7 = Voltage Divider connection
  A6 = TMP36 temperature sensor output pin

  i2c bus
  GND pins on MS5803-14BA and BMP180 sensors to Nano GND pin
  Vcc pins on MS5803-14BA and BMP180 sensors to Nano 3.3V pin
  SDA pins on MS5803-14BA and BMP180 sensors to Nano A4 pin
  SCL pins on MS5803-14BA and BMP180 sensors to Nano A5 pin

  5V = Supply to the TMP36 temperature sensor.

  Communications
  // SLAVE to MASTER
  Serial Connection: Topside D1 (TX) to ROV D0 (RX)
  Serial Connection: Topside D0 (RX) to ROV D1 (TX)
  // SLAVE to POD
  Serial Connection: Pod D7 (RX) to Slave D8 (TX)
  Serial Connection: Pod D8 (TX) to Slave D7 (RX)
  Connect the GND on both

  Please note that the ESCs will all have been programmed by this
  point in the project.

  The onboard voltage, heading, depth, and internal temperature
  data is sent through the Serial link back to the Master
  for display on a 16x2 LCD screen.

  The heading is from an HMC5883L Digital Compass (i2c address 0x1E)
  and the depth from a MS5803-14BA high pressure sensor (i2c address 0x76)

  See also: HoryzonTrigger.ino, ROVPS2Control_Masterv0.ino,
  ROVDoNothing.ino, ROVSubBv0.ino, DigitalCompassv2.ino,
  PTLoggerv4.ino and TMP36_Temperature_Sensor.ino.


*/

#include <Wire.h>  //i2c library for the digital compass and depth sensor
#include <Servo.h>
#include <EasyTransfer.h> // Bill Porter's Easy Transfer Library
#include <SoftEasyTransfer.h>
#include <MS5803_14.h> //Library for the MS5803-14BA
#include <SoftwareSerial.h>
#include <ACCEL_GYRO_KALMAN.h>

EasyTransfer ETin, ETout;  //Create the two Easy transfer Objects for
// Two way communication

// Two way communication between research pod and slave
SoftwareSerial podSerial(9, 10);
SoftEasyTransfer SETin, SETout;

MS_5803 sensor = MS_5803(512);

Servo ESCVL;  // Create Servo Object ESC Vertical Left
Servo ESCVR;  // Create Servo Object ESC Vertical Right
Servo ESCHL;  // Create Servo Object ESC Horizontal Left
Servo ESCHR;  // Create Servo Object ESC Horizontal Right
Servo CamAng; // Create Servo Object for the Camera Pitch Servo.

GYRO gyroscope;
ACCEL accelerometer;
float accPitch = 0;
float accRoll = 0;

const int RedLEDpin = 13; // The indicator LED pin is 13.
const int HeadLts = 12; // The Headlight Control is on pin 12
const int CamRecTrig = 3; //Camera video recorder trigger is on pin D3
const int CamPhotTrig = 2; //Camera photo trigger is on pin D2

const int hmc5883Address = 0x1E; //0011110b, I2C 7bit address for compass
const byte hmc5883ModeRegister = 0x02;
const byte hmcContinuousMode = 0x00;
const byte hmcDataOutputXMSBAddress = 0x03;

volatile boolean CamRecd;  //Camera record function toggle
volatile boolean CamPhoto;  //Camera photo function toggle

const int Voltpin = A7; // analogue pin used to read the battery voltage
const int Temppin = A6; // analogue pin used to read the TMP36 Temp sensor
//Analogue pins A4 and A5 are taken by the i2c bus.

int volts;    // variable to read the voltage from the analog pin
int x, y, z; //triple axis data for the digital compass.
int angle; //calculated horizontal heading angle.

// station keeping variables for PID
double oldAngle = 0, angSum = 0, PID = 0;      // PID variables
double cP = 0.9, cI = 0, cD = 0.1;                 // PID constants
double PIDScale = 1, PIDShift = 90;             // output scaling
int leftVal;
int rightVal;
int low_bound = -10;
int high_bound = 10;

float MS5803Press;  //Pressure from the MS5803 Sensor.
float MS5803Temp;  //Temperature from the MS5803 Sensor.

const float RefVolts = 5.0; // 5-Volt board reference voltage on Nano
const float ResistFactor = 319.68; //Calculated from 1023.0*(R2/(R1 + R2)
//where R1 = 2200 ohms and R2 = 1000 ohms for a 15V max voltage.
long TriggerHoldTm = 0;  // the time since the camera button was triggered
long TriggerHoldDuration = 150;  //The time in milliseconds to hold the camera triggers LOW.

struct RECEIVE_DATA_STRUCTURE {
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

struct SEND_DATA_STRUCTURE {
  int BattVolt;  //Battery Voltage message to the Master.
  int ROVTemp; //ROV interior temperature back to Master
  int ROVDepth; //ROV depth reading (m)
  int ROVHDG;  //ROV direction (Degrees)
  int AccX;
  int AccY;
  int AccZ;
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

  // testing
  String sensorData;
};

struct RESEARCH_POD_RECEIVE_DATA {
  String SensorData;
  int PodPower;
  int PodState;
};

struct RESEARCH_POD_SEND_DATA {
  int ROVPressure;  // ROV depth reading
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;
RESEARCH_POD_RECEIVE_DATA podDataIn;
RESEARCH_POD_SEND_DATA podDataOut;

void setup()
{
  pinMode(RedLEDpin, OUTPUT);
  pinMode(HeadLts, OUTPUT);
  pinMode(CamRecTrig, OUTPUT);
  pinMode(CamPhotTrig, OUTPUT);
  digitalWrite(HeadLts, LOW); //Set the Headlights to Off
  CamRecd = false;  //Sets the Camera default to not recording
  CamPhoto = false; // No photos triggered.
  digitalWrite(RedLEDpin, LOW);
  digitalWrite(CamRecTrig, HIGH); //Both camera functions are controlled
  digitalWrite(CamPhotTrig, HIGH); // by making the pin low.

  ESCVL.attach(8, 600, 2250); //attach the ESCVL to pin 8
  ESCVR.attach(7, 600, 2250); //attach the ESCVR to pin 7
  ESCHL.attach(6, 600, 2250); //attach the ESCHL to pin 6
  ESCHR.attach(5, 600, 2250); //attach the ESCHR to pin 5
  //Due to problems with the ESC recognising the maximum
  //position at the default settings, the figures after
  //the pin number are the microsecond signals for the
  //minimum and maximum that the ESC will recognise.
  // 600 and 2250 work.
  CamAng.attach(4); //Attach the camera Pitch Servo to pin 4

  //  throttle = 90;  //Set throttle to the neutral position.
  ESCVL.write(90);  //Set the ESCVL signal to the neutral position.
  ESCVR.write(90);  //Set the ESCVL signal to the neutral position.
  ESCHL.write(90);  //Set the ESCVL signal to the neutral position.
  ESCHR.write(90);  //Set the ESCVL signal to the neutral position.
  CamAng.write(90); //Set the camera servo pitch to be level.

  Wire.begin(); // Start the i2c communication
  //Initialise the Digital Compass
  Wire.beginTransmission(hmc5883Address);  //Begin communication with compass
  Wire.write(hmc5883ModeRegister);  //select the mode register
  Wire.write(hmcContinuousMode); //continuous measurement mode
  Wire.endTransmission();
  // Initialize the MS5803 sensor.
  sensor.initializeMS_5803();

  delay(10000);   //Ten second delay
  //The ESC should now be initialised and ready to run.

  Serial.begin(9600); //Begin Serial to talk to the Master Arduino
  ETin.begin(details(rxdata), &Serial); //Get the Easy Transfer Library happening through the Serial
  ETout.begin(details(txdata), &Serial);

  podSerial.begin(9600);
  // Begin Serial communication with research pod
  SETin.begin(details(podDataIn), &podSerial);
  SETout.begin(details(podDataOut), &podSerial);

  initialise_IMU(); // setup the accelerometer and gyroscope

  //The camera starts in record mode probably due to Arduino startup signals
  //and so this needs to be stopped.  The sequence below sends a toggle to
  //the camera to stop it from recording.  obviously this will leave a small
  //waste video file, but we will need to live with that.

  digitalWrite(CamRecTrig, LOW); //Trip the photo trigger.
  delay(100);
  digitalWrite(CamRecTrig, HIGH);
}

void loop() {
  // Send the message to the serial port for the ROV Arduino
  ETout.sendData();
  SETout.sendData();

  //Based on Bill Porter's example for the Two Way Easy Transfer Library
  //We will include a loop here to make sure the receive part of the
  //process runs smoothly.
  for (int i = 0; i < 5; i++) {
    ETin.receiveData();
    // We'll do something properly with the returned data at a later s
    ESCVL.write(rxdata.upLraw);  //Set the ESCVL signal to the defined throttle position.
    ESCVR.write(rxdata.upRraw);  //Set the ESCVR signal to the defined throttle position.
    ESCHL.write(rxdata.HLraw);  //Set the ESCHL signal to the defined throttle position.
    ESCHR.write(rxdata.HRraw);  //Set the ESCHR signal to the defined throttle position.
    CamAng.write(rxdata.CamPitch); //Set the camera servo pitch to the defined angle.
    digitalWrite(HeadLts, rxdata.LEDHdlts); //Light the headlights based on the Message data
    delay(18);
  }

  //The camera settings are status flags so we will need to trigger the events based on
  //changes in the data.
  if (rxdata.CamRec && !CamRecd) { //If the signal is to trigger video recording
    //and the Camera is not already triggered drop the camera recording pin to LOW.
    CamRecTrigger(); //Run the camera triggering signal
    CamRecd = true; //update the flag.
  }
  if (!rxdata.CamRec && CamRecd) { //If the camera no longer needing to trigger
    // then signal the camera and turn off the flag.
    CamRecTrigger(); //Run the camera triggering signal
    CamRecd = false; //update the flag.
  }

  if (rxdata.CamPhotoShot && !CamPhoto) { //If the camera is required to fire a shot trigger
    // the camera photo pin.
    digitalWrite(CamPhotTrig, LOW); //Trip the photo trigger pin.
    TriggerHoldTm = millis();  //Reset the time that a camera trigger was used.
    CamPhoto = true; //update the flag.
  }

  if (!rxdata.CamPhotoShot && CamPhoto) { //If the camera photo signal ceases
    // reset the camera flag.
    CamPhoto = false; //update the flag.
  }


  if (millis() - TriggerHoldTm > TriggerHoldDuration) { //If camera button held long enough release it.
    // hopefully this little routine will speed up the sketch processing.
    digitalWrite(CamRecTrig, HIGH);
    digitalWrite(CamPhotTrig, HIGH); //Just make them both inactive
  }

  delay(18);  //This delay is added to give the ROV a chance to
  //return data
  volts = analogRead(Voltpin) / ResistFactor * RefVolts * 10; //Read the voltage
  //from the battery through the voltage divider.  Factor of 10 used
  //to help achieve an integer with 0.1V accuracy.
  txdata.BattVolt = volts; //Send back the onboard battery voltage.
  txdata.ROVTemp = analogRead(Temppin); //This reads the pin keeps it as a 0-1024 value.

  //Read the digital compass
  //Tell the HMC5883L where to begin reading the data
  Wire.beginTransmission(hmc5883Address);
  Wire.write(hmcDataOutputXMSBAddress);  //Select register 3, X MSB register
  Wire.endTransmission();

  //Read data from each axis
  Wire.requestFrom(hmc5883Address, 6);
  if (6 <= Wire.available()) {
    x = Wire.read() << 8; //X msb
    x |= Wire.read();   //X lsb
    z = Wire.read() << 8; //Z msb
    z |= Wire.read();   //Z lsb
    y = Wire.read() << 8; //Y msb
    y |= Wire.read();   //Y lsb
  }

  angle = atan2(-y, x) / M_PI * 180;
  if (angle < 0) {
    angle = angle + 360;
  }
  txdata.ROVHDG = angle;  //ROV direction (Degrees)

  //Reading and MS5803-14BA Sensor
  // Use readSensor() function to get pressure and temperature reading from the MS5803.
  sensor.readSensor();
  MS5803Press = sensor.pressure();  //Pressure in mbar absolute
  MS5803Temp = sensor.temperature();  //Although we have gathered this
  //it won't be used at this stage.

  txdata.ROVDepth = (MS5803Press - 1013) / 98.1; //ROV depth reading (m)
  podDataOut.ROVPressure = MS5803Press;

  if(SETin.receiveData()){
    Serial.println(podDataIn.SensorData);  // FOR TESTING
    txdata.sensorData = podDataIn.SensorData;
  }

  if(rxdata.changed){
    cD = rxdata.cD;
    cP = rxdata.cP;
    cI = rxdata.cI;
    PIDShift = rxdata.pidShift;
    PIDScale = rxdata.pidScale;
    low_bound = rxdata.low_bound;
    high_bound = rxdata.high_bound;
    // reset PID
    oldAngle = 0;
    angSum = 0;
    PID = 0; 
  }

  // read acceleration and gyroscope values
  txdata.cD = cD;
  txdata.cP = cP;
  txdata.cI = cI;
  txdata.pidShift = PIDShift;
  txdata.pidScale = PIDScale;
  txdata.low_bound = low_bound;
  txdata.high_bound = high_bound;

  read_IMU();
  pid();
  if(isNotControllingROV(rxdata.upLraw, rxdata.upRraw, rxdata.HLraw, rxdata.HRraw, 0)){
       stationKeepRoll();
  }
}

void changeParams(){
  Serial.print("CD: ");
  Serial.println(cD);
  while (!Serial.available()) {
  }
  cD = Serial.parseFloat();
  Serial.print("CI: ");
  Serial.println(cI);
  while (!Serial.available()) {
  }
  cI = Serial.parseFloat();
  Serial.print("CP: ");
  Serial.print(cP);
  while (!Serial.available()) {
  }
  cP = Serial.parseFloat();
  Serial.print("PID ");
  Serial.print(PID);
  while (!Serial.available()) {
  }
  PID = Serial.parseFloat();
  Serial.print("PIDScale ");
  Serial.print(PIDScale);
  while (!Serial.available()) {
  }
  PIDScale = Serial.parseFloat();
  Serial.print("PIDShift ");
  Serial.print(PIDShift);
  while (!Serial.available()) {
  }
  PIDShift = Serial.parseFloat();
}


void CamRecTrigger() {
  digitalWrite(CamRecTrig, LOW); //Trip the recorder toggle.
  TriggerHoldTm = millis();  //Reset the time that a camera trigger was used.
}

boolean isNotControllingROV(int VL, int VR, int HL, int HR, int desired){
  // high is upper limit, and low is lower (checks if values +/- bound of desired value)
  int bound = 2;
  int high = desired + bound;
  int low = desired - bound;
  return low <= VL && VL <= high && low <= VR && VR <= high && low <= HL && HL <= high && low <= HR && HR <= high;
}

void initialise_IMU() {
  // Initialize ADXL345
  while(!accelerometer.begin()){
    delay(500);
  }
  
  Serial.println("Hi there");
  // Initialize L3G4200D
  while(!gyroscope.begin(SCALE_250DPS, DATARATE_400HZ_50)){
    delay(500);
  }

  Serial.println("Begin Calibration");
  // Must call calibrate twice because for some reason, once doesn't work
  gyroscope.calibrate(250);
  gyroscope.calibrate(250);
  Serial.println("Finish Calibration");
}

void pid(){
  Serial.println("Calculating PID");
  double roll = txdata.AccRoll;
  
  //  After you've read the angle from 0 (roll)
  angSum += roll;
  PID = cP*roll + cI*angSum + cD*(roll - oldAngle);
  oldAngle = roll;

  leftVal = PIDScale*PID + PIDShift;  

  // transform right thruster value to equivalent opposite value of left thruster
  int diff = 90 - leftVal;
  rightVal = 90 + diff;
}

  //    - <- -> +
  //      Z Axis
  //        |
  //  |-----------|
  //  |  Camera   |
  //  |           |
  //  | LT     RT | - Y Axis
  //  |           |
  //  |           |
  //  |-----------|
  //
  // Top view of ROV, where IMU is placed so that z-axis is in line with camera
  // LT = Left Thruster - Vertical
  // RT = Right Thruster - Vertical
void stationKeepRoll() {
  // if the angle is between -10 and 10
  // Send values to thruster
  if(txdata.AccRoll < low_bound || txdata.AccRoll > high_bound){
   ESCVL.write(leftVal);
   ESCVR.write(rightVal);
  }
}

void read_IMU(){
  Vector acc = accelerometer.read_normalised();
  Vector gyr = gyroscope.read_normalised();

  // Calculate Pitch & Roll from accelerometer (deg)
  accRoll = (atan2(acc.x_axis, sqrt(acc.y_axis*acc.y_axis + acc.z_axis*acc.z_axis))*180.0)/M_PI;
  accPitch  = -(atan2(acc.y_axis, acc.z_axis)*180.0)/M_PI;

  txdata.AccX = acc.x_axis;
  txdata.AccY = acc.y_axis;
  txdata.AccZ = acc.z_axis;
  txdata.GyroX = gyr.x_axis;
  txdata.GyroY = gyr.y_axis;
  txdata.GyroZ = gyr.z_axis;
  txdata.AccPitch = accPitch;

  // if the ptich is  negative value, the roll is on the left side
  // roll is now any value between 0 to 90 to 0.
  // negative rolling to the left, positive to the right
  if(accPitch > 0){
    // will give a value between -90 and 0, -90 being the horizontal on the left and 0 being the neutral
    txdata.AccRoll = accRoll - 90; 
  } else {
    // will give a value between 0 and 90, 0 being the neutral and 90 being the horizontal on the right
    txdata.AccRoll = -accRoll + 90; 
  }

  Serial.println("");
  Serial.print("Accel Pitch: ");
  Serial.println(accPitch);
  Serial.print("Accel Roll: ");
  Serial.println(txdata.AccRoll);
  Serial.print("Acc: ");
  Serial.print(acc.x_axis);
  Serial.print(", ");
  Serial.print(acc.y_axis);
  Serial.print(", ");
  Serial.println(acc.z_axis);
  Serial.print("Gyro: ");
  Serial.print(gyr.x_axis);
  Serial.print(", ");
  Serial.print(gyr.y_axis);
  Serial.print(", ");
  Serial.println(gyr.z_axis);
  Serial.println("");
}
