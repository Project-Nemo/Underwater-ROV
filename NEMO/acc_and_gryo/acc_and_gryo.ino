#include <Wire.h>

// for gyro
#define gyro_ctrl_reg1 0x20
#define gyro_ctrl_reg2 0x21
#define gyro_ctrl_reg3 0x22
#define gyro_address 0x69

// for acc
#define acc_ctrl_reg 0x2d
#define acc_address (0x53)  // ADXL345 device address 

int ax, gx;
int ay, gy;
int az, gz;
boolean notDriving = true;

void setup(){
  Wire.begin();
  Serial.begin(9600);
  powerOn(); // setup the accelerometer and gyroscope
  delay(1500); //wait for the sensor to be ready 
}

void loop(){
  getAccValues(); // read the accelerometer values and store them in variables x,y,z
  getGyroValues();  // read the gyroscope values and store them in variables x,y,z
  if(notDriving) {
    reactToReading();
  }
}

void reactToReading(){
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

  // PSEUDO CODE!
  // If gz is positive, LT go down, RT go up
  // If gz is negative, LT go up, RT go down
}

void printReadings(){
  Serial.print("AX:");
  Serial.print(ax);

  Serial.print(" AY:");
  Serial.print(ay);

  Serial.print(" AZ:");
  Serial.print(az);
  
  Serial.print("  GX:");
  Serial.print(gx);

  Serial.print(" GY:");
  Serial.print(gy);

  Serial.print(" GZ:");
  Serial.println(gz);
}

void powerOn() {
  //Turning on the accelerometer
  writeRegister(acc_address, acc_ctrl_reg, 0);   
  writeRegister(acc_address, acc_ctrl_reg, 16);
  writeRegister(acc_address, acc_ctrl_reg, 8); 
    
  //Turning on the gyroscope
  writeRegister(gyro_address, gyro_ctrl_reg1, 15);
  writeRegister(gyro_address, gyro_ctrl_reg2, 0);
  writeRegister(gyro_address, gyro_ctrl_reg3, 8);
}

void getAccValues(){

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

void getGyroValues(){

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

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}
