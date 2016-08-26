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

#define DATA_LENGTH 10

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

const int RedLEDpin = 13; // The indicator LED pin is 13.
const int HeadLts = 12; // The Headlight Control is on pin 12
const int CamRecTrig = 3; //Camera video recorder trigger is on pin D3
const int CamPhotTrig = 2; //Camera photo trigger is on pin D2

const int hmc5883Address = 0x1E; //0011110b, I2C 7bit address for compass
const byte hmc5883ModeRegister = 0x02;
const byte hmcContinuousMode = 0x00;
const byte hmcDataOutputXMSBAddress = 0x03;

const byte gyro_ctrl_reg1 = 0x20;
const byte gyro_ctrl_reg2 = 0x21;
const byte gyro_ctrl_reg3 = 0x22;
const byte gyro_address = 0x69;   // Gryo device address

// for acc
const byte acc_ctrl_reg = 0x2d;
const byte acc_address = 0x53;  // ADXL345 device address

volatile boolean CamRecd;  //Camera record function toggle
volatile boolean CamPhoto;  //Camera photo function toggle

const int Voltpin = A7; // analogue pin used to read the battery voltage
const int Temppin = A6; // analogue pin used to read the TMP36 Temp sensor
//Analogue pins A4 and A5 are taken by the i2c bus.

int volts;    // variable to read the voltage from the analog pin
int x, y, z; //triple axis data for the digital compass.
int angle; //calculated horizontal heading angle.

// variables for storing accelerometer values
int ax, ay, az;
// variables for storing gryoscope values
int gx, gy, gz;

// Overall rotation values (for stable readings)
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;  
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

// IMU data arrays to allow moving averages to be calculated
int axData[DATA_LENGTH];
int ayData[DATA_LENGTH];
int azData[DATA_LENGTH];
int gxData[DATA_LENGTH];
int gyData[DATA_LENGTH];
int gzData[DATA_LENGTH];

//  Base accel and gryo values for calibration offset
float    base_x_accel;
float    base_y_accel;
float    base_z_accel;
float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;

// station keeping variables for PID
double oldAngle = 0, angSum = 0, PID = 0;      // PID variables
double cP = 1, cI = 1, cD = 1;                 // PID constants
double PIDScale = 1, PIDShift = 0;             // output scaling

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
  int PodPower;   // power of pod in watts
  int PodState;   // 1 if functioning correctly, 0 otherwise
};

struct RESEARCH_POD_RECEIVE_DATA {
  int PodPower;    // watts 
  int PodState;       // 1 if pod functioning correctly, 0 otherwise
};

struct RESEARCH_POD_SEND_DATA {
  int ROVDepth;  // ROV depth reading
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

  powerOnIMU(); // setup the accelerometer and gyroscope
  delay(5000); // wait for imu to be ready
  calibrateSensors();
  setLastReadAngle(millis(), 0, 0, 0, 0, 0, 0);

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
  podDataOut.ROVDepth = txdata.ROVDepth;

  if(SETin.receiveData()){
    txdata.PodPower = podDataIn.PodPower;
    txdata.PodState = podDataIn.PodState;
  }

  readIMUData();  
  calculateAccelAndGryoAngles();

  // TODO sationkeeping code trigger here
  
}


void CamRecTrigger() {
  digitalWrite(CamRecTrig, LOW); //Trip the recorder toggle.
  TriggerHoldTm = millis();  //Reset the time that a camera trigger was used.
}

void powerOnIMU() {
  //Turning on the accelerometer
  writeRegister(acc_address, acc_ctrl_reg, 0);
  writeRegister(acc_address, acc_ctrl_reg, 16);
  writeRegister(acc_address, acc_ctrl_reg, 8);
  //Turning on the gyroscope
  writeRegister(gyro_address, gyro_ctrl_reg1, 15);
  writeRegister(gyro_address, gyro_ctrl_reg2, 0);
  writeRegister(gyro_address, gyro_ctrl_reg3, 8);
}

void getAccValues() {

  byte xMSB = readRegister(acc_address, 0x33); //51
  byte xLSB = readRegister(acc_address, 0x32); //50
  ax = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(acc_address, 0x35); //53
  byte yLSB = readRegister(acc_address, 0x34); //52
  ay = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(acc_address, 0x37); //55
  byte zLSB = readRegister(acc_address, 0x36); //54
  az = ((zMSB << 8) | zLSB);
}

void getGyroValues() {

  byte xMSB = readRegister(gyro_address, 0x29); //41
  byte xLSB = readRegister(gyro_address, 0x28); //40
  gx = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(gyro_address, 0x2B); //43
  byte yLSB = readRegister(gyro_address, 0x2A); //42
  gy = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(gyro_address, 0x2D); //45
  byte zLSB = readRegister(gyro_address, 0x2C); //44
  gz = ((zMSB << 8) | zLSB);
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address) {

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while (!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}

void readIMUData() {
  getAccValues();   // read the accelerometer values
  getGyroValues();  // read the gyroscope values

  // caluclate moving average values and send them to master
  txdata.AccX = movingAverage(axData, ax - base_x_accel);
  txdata.AccY = movingAverage(ayData, ay - base_y_accel);
  txdata.AccZ = movingAverage(azData, az - base_z_accel);
  txdata.GyroX = movingAverage(gxData, gx - last_gyro_x_angle);
  txdata.GyroY = movingAverage(gyData, gy - last_gyro_y_angle);
  txdata.GyroZ = movingAverage(gzData, gz - last_gyro_z_angle);
}

double movingAverage(int data[], int newVal) {
  double total = 0;
  for (int i = 1; i < DATA_LENGTH; i++) {
    data[i - 1] = data[i];
    total = total + data[i - 1];
  }
  data[DATA_LENGTH - 1] = newVal;
  total = total + newVal;
  return total / DATA_LENGTH;;
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
  // use moving average to avoid reacting to transients
  double roll = txdata.GyroY;   
  //  After you've read the angle from 0 (roll)
  angSum += roll;
  PID = cP*roll + cI*angSum + cD*(roll - oldAngle);
  oldAngle = roll;
  
  //  Output adjustment
  //  <output> -> PIDScale*PID + PIDShift;
  
  int leftVal = PIDScale*PID + PIDShift;   // TODO: NEED TO COME UP WITH APPROPRIATE TRANSFROMATION HERE
  int diff = 90 - leftVal;
  int rightVal = 90 + diff;
  
  // Send values to thruster
   ESCVL.write(leftVal);   
   ESCVR.write(rightVal);  
}

void calibrateSensors() {  
  Serial.println("Starting Calibration"); 
  
  // Read and average the raw values from the IMU
  for (int i = 0; i < DATA_LENGTH; i++) {
    getAccValues();   
    getGyroValues();
    axData[i] = ax;
    ayData[i] = ay;
    azData[i] = az;
    gxData[i] = gx;
    gyData[i] = gy;
    gzData[i] = gz;
    delay(100);
  }

  // Discard the first set of values read from the IMU
  getAccValues();   
  getGyroValues();
  axData[0] = ax;
  ayData[0] = ay;
  azData[0] = az;
  gxData[0] = gx;
  gyData[0] = gy;
  gzData[0] = gz;
    
  // Store the raw calibration values globally
  base_x_accel = movingAverage(axData, ax);
  base_y_accel = movingAverage(ayData, ay);
  base_z_accel = movingAverage(azData, az);
  base_x_gyro = movingAverage(gxData, gx);
  base_y_gyro = movingAverage(gyData, gy);
  base_z_gyro = movingAverage(gzData, gz);
  
  Serial.println("Finished Calibration");
}

void setLastReadAngle(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

void calculateAccelAndGryoAngles() {
  int error;
  double dT;
  int valueIndex = DATA_LENGTH - 1;
  
  // Get the time of reading for rotation computations
  unsigned long t_now = millis();

  float FS_SEL = 131;
  float gyro_x = (gxData[valueIndex])/FS_SEL;
  float gyro_y = (gyData[valueIndex])/FS_SEL;
  float gyro_z = (gzData[valueIndex])/FS_SEL;
  
  
  // Get raw acceleration values
  // float G_CONVERT = 16384;
  float accel_x = axData[valueIndex];
  float accel_y = ayData[valueIndex];
  float accel_z = azData[valueIndex];
  
  // Get angle values from accelerometer
  float RADIANS_TO_DEGREES = 180/3.14159;
  //  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_z = 0;
  
  // Compute the (filtered) gyro angles
  float dt =(t_now - last_read_time)/1000.0;
  float gyro_angle_x = gyro_x*dt + last_x_angle;
  float gyro_angle_y = gyro_y*dt + last_y_angle;
  float gyro_angle_z = gyro_z*dt + last_z_angle;
  
  // Compute the drifting gyro angles
  float unfiltered_gyro_angle_x = gyro_x*dt + last_gyro_x_angle;
  float unfiltered_gyro_angle_y = gyro_y*dt + last_gyro_y_angle;
  float unfiltered_gyro_angle_z = gyro_z*dt + last_gyro_z_angle;
  
  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.96;
  float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
  float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
  float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
  
  // Update the saved data with the latest values
  setLastReadAngle(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
  
  // Send the data to the serial port
  Serial.print(F("DEL:"));              //Delta T
  Serial.print(dt, DEC);
  Serial.print(F("#ACC:"));              //Accelerometer angle
  Serial.print(accel_angle_x, 2);
  Serial.print(F(","));
  Serial.print(accel_angle_y, 2);
  Serial.print(F(","));
  Serial.print(accel_angle_z, 2);
  Serial.print(F("#GYR:"));
  Serial.print(unfiltered_gyro_angle_x, 2);        //Gyroscope angle
  Serial.print(F(","));
  Serial.print(unfiltered_gyro_angle_y, 2);
  Serial.print(F(","));
  Serial.print(unfiltered_gyro_angle_z, 2);
  Serial.print(F("#FIL:"));             //Filtered angle
  Serial.print(angle_x, 2);
  Serial.print(F(","));
  Serial.print(angle_y, 2);
  Serial.print(F(","));
  Serial.print(angle_z, 2);
  Serial.println(F(""));
  
  // Delay so we don't swamp the serial port
  delay(5);
}
