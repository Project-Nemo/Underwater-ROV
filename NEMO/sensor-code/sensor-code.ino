/*
 * Code for Underwater ROV
 * Team 8 - Project Nemo
 * 
 * Authors: Bonnie Liao, Divya Patel
 */
#include <Wire.h>

const int MPU_addr = 0x68; // I2C address
int16_t ax, ay, az, tmp, gx, gy, gz;

/*
 * Class for the naive station keeping
 */
class StationKeeping
{
  int ledPin; // pin to read off of IMU sensor
  
};

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the IMU)
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  ax = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)  
  ay = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Serial.print(F("ax = ")); Serial.print(ax);
  Serial.print(F(" | ay = ")); Serial.print(ay);
  Serial.print(F(" | az = ")); Serial.print(az);
  Serial.print(F(" | Tmp = ")); Serial.print(tmp / 340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(F(" | gx = ")); Serial.print(gx);
  Serial.print(F(" | gy = ")); Serial.print(gy);
  Serial.print(F(" | gz = ")); Serial.println(gz);
  delay(333);
  
}
