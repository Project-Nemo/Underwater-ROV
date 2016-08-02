#include <Wire.h>

// for gyro
#define GYRO_CTRL_REG1 0x20
#define GYRO_CTRL_REG2 0x21
#define GYRO_CTRL_REG3 0x22
#define GYRO_ADDRESS 0x69

// for acc
#define ACC_CTRL_REG 0x2d
#define ACC_ADDRESS (0x53)  // ADXL345 device address 

int ax, gx;
int ay, gy;
int az, gz;

void setup(){
  Wire.begin();
  Serial.begin(9600);
  powerOn(); // setup the accelerometer and gyroscope
  delay(1500); //wait for the sensor to be ready 
}

void loop(){
  getAccValues(); // read the accelerometer values and store them in variables x,y,z
  Serial.print("AX:");
  Serial.print(ax);

  Serial.print(" AY:");
  Serial.print(ay);

  Serial.print(" AZ:");
  Serial.print(az);

  
  getGyroValues();  // read the gyroscope values
  Serial.print("  GX:");
  Serial.print(gx);

  Serial.print(" GY:");
  Serial.print(gy);

  Serial.print(" GZ:");
  Serial.println(gz);
}

void powerOn() {
  //Turning on the accelerometer
  writeRegister(ACC_ADDRESS, ACC_CTRL_REG, 0);   
  writeRegister(ACC_ADDRESS, ACC_CTRL_REG, 16);
  writeRegister(ACC_ADDRESS, ACC_CTRL_REG, 8); 
    
  //Turning on the gyroscope
  writeRegister(GYRO_ADDRESS, GYRO_CTRL_REG1, 15);
  writeRegister(GYRO_ADDRESS, GYRO_CTRL_REG2, 0);
  writeRegister(GYRO_ADDRESS, GYRO_CTRL_REG3, 8);
}

void getAccValues(){

  byte xMSB = readRegister(ACC_ADDRESS, 0x33); //51
  byte xLSB = readRegister(ACC_ADDRESS, 0x32); //50
  ax = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(ACC_ADDRESS, 0x35); //53
  byte yLSB = readRegister(ACC_ADDRESS, 0x34); //52
  ay = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(ACC_ADDRESS, 0x37); //55
  byte zLSB = readRegister(ACC_ADDRESS, 0x36); //54
  az = ((zMSB << 8) | zLSB);
}

void getGyroValues(){

  byte xMSB = readRegister(GYRO_ADDRESS, 0x29); //41
  byte xLSB = readRegister(GYRO_ADDRESS, 0x28); //40
  gx = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(GYRO_ADDRESS, 0x2B); //43
  byte yLSB = readRegister(GYRO_ADDRESS, 0x2A); //42
  gy = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(GYRO_ADDRESS, 0x2D); //45
  byte zLSB = readRegister(GYRO_ADDRESS, 0x2C); //44
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
