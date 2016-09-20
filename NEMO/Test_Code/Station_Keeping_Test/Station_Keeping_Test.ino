/*
  D8 = ESC Vertical Left
  D7 = ESC Vertical Right
  D6 = ESC Horizontal Left
  D5 = ESC Horizontal Right
*/

#include <Wire.h>  //i2c library for the digital compass and depth sensor
#include <Servo.h>
#include <ACCEL_GYRO_KALMAN.h>

Servo ESCVL;  // Create Servo Object ESC Vertical Left
Servo ESCVR;  // Create Servo Object ESC Vertical Right
Servo ESCHL;  // Create Servo Object ESC Horizontal Left
Servo ESCHR;  // Create Servo Object ESC Horizontal Right
Servo CamAng; // Create Servo Object for the Camera Pitch Servo.

GYRO gyroscope;
ACCEL accelerometer;
float accPitch = 0;
float accRoll = 0;

const int hmc5883Address = 0x1E; //0011110b, I2C 7bit address for compass
const byte hmc5883ModeRegister = 0x02;
const byte hmcContinuousMode = 0x00;
const byte hmcDataOutputXMSBAddress = 0x03;

int volts;    // variable to read the voltage from the analog pin
int x, y, z; //triple axis data for the digital compass.
int angle; //calculated horizontal heading angle.

// station keeping variables for PID
double oldAngle = 0, angSum = 0, PID = 0;      // PID variables
double cP = 1, cI = 1;
double cD = 1;                 // PID constants
double PIDScale = 1, PIDShift = 0;             // output scaling

struct SEND_DATA_STRUCTURE {
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
} txdata;

void setup() {
  Serial.begin(9600);
  //initialise_IMU();
}

void loop() {
  // read acceleration and gyroscope values
 // read_IMU();

  if (Serial.available()) {
    Serial.println("HI");
    char ch = Serial.read();
    if (ch == 'x') {
      changeParams();
    }
  }
  // trigger station keeping code if throttles on PS2 controller are not being moved
  // assumes 0 is not moving value.
 // stationKeepRoll();
}

void changeParams(){
  Serial.println("");
  Serial.print("CD: ");
  Serial.println(cD);
  while (!Serial.available()) {
  }
  float change = Serial.parseFloat();
  cD = change;
  Serial.print("CI: ");
  Serial.println(cI);
  while (!Serial.available()) {
  }
  float change = Serial.parseFloat();
  cI = change;
  Serial.print("CP: ");
  Serial.print(cP);
  while (!Serial.available()) {
  }
  float change = Serial.parseFloat();
  cP = change;
  Serial.print("PID ");
  Serial.print(PID);
  while (!Serial.available()) {
  }
  float change = Serial.parseFloat();
  PID = change;
  Serial.print("PIDScale ");
  Serial.print(PIDScale);
  while (!Serial.available()) {
  }
  float change = Serial.parseFloat();
  PIDScale = change;
  Serial.print("PIDShift ");
  Serial.print(PIDShift);
  while (!Serial.available()) {
  }
  float change = Serial.parseFloat();
  PIDShift = change;
}

void initialise_IMU() {
  // Initialize ADXL345
  while(!accelerometer.begin()){
    delay(500);
  }
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
  Serial.println("Hi");
  double roll = txdata.AccRoll;
  
  //  After you've read the angle from 0 (roll)
  angSum += roll;
  PID = cP*roll + cI*angSum + cD*(roll - oldAngle);
  oldAngle = roll;

  //  Output adjustment
  //  <output> -> PIDScale*PID + PIDShift;

  int leftVal = PIDScale*PID + PIDShift;   // TODO: NEED TO COME UP WITH APPROPRIATE TRANSFROMATION HERE

  // transform right thruster value to equivalent opposite value of left thruster
  int diff = 90 - leftVal;
  int rightVal = 90 + diff;

  // Send values to thruster
   ESCVL.write(leftVal);
   ESCVR.write(rightVal);
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
  txdata.AccRoll = accRoll;
  txdata.AccPitch = accPitch;

  // if the ptich is  negative value, the roll is on the left side
  if(accPitch < 0){
    // roll now assigned to any value between -90 to 0 to 90.
    txdata.AccRoll = -accRoll;
  }

  Serial.println("");
  Serial.print("Accel Pitch: ");
  Serial.println(accPitch);
  Serial.print("Accel Roll: ");
  Serial.println(accRoll);
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
