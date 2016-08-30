/*
    Kalman Filter Example for ADXL345 & L3G4200D. Output for processing.
    Read more: http://www.jarzebski.pl/arduino/rozwiazania-i-algorytmy/odczyty-pitch-roll-oraz-filtr-kalmana.html
    GIT: https://github.com/jarzebski/Arduino-KalmanFilter
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <ACCEL_GYRO_KALMAN.h>

GYRO gyroscope;
ACCEL accelerometer;

float accPitch = 0;
float accRoll = 0;

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
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

void loop()
{
  Vector acc = accelerometer.read_normalised();
  Vector gyr = gyroscope.read_normalised();

  // Calculate Pitch & Roll from accelerometer (deg)
  accRoll = (atan2(acc.x_axis, sqrt(acc.y_axis*acc.y_axis + acc.z_axis*acc.z_axis))*180.0)/M_PI;
  accPitch  = -(atan2(acc.y_axis, acc.z_axis)*180.0)/M_PI;

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


