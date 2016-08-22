
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <BH1750.h>


//Hardware pin definitions
const int UV_OUT = A0; //Output from the sensor (ML8511)
const int REF_3V3 = A1; //3.3V power on the Arduino board (ML8511)
const int REF_5V = A2;
const int TURBIDITY_OUT = A7; 
const int chipSelect = 10; //SPI Chip select pin
BH1750 lightMeter; //Create Light meter object

byte buff[2];


void setup() {
  
   // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Wire.begin();
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  //###################################################
  //             SD Card Initializing Code
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
//#####################################################
//                UV Sensor (ML8511)
  Serial.print("Initializing ML8511 \n");
  pinMode(UV_OUT, INPUT);
  pinMode(REF_3V3, INPUT);
  Serial.print("ML8511 Initialized \n");
  
//##################################################### 
//         Setup Comms with light sensor (GY-30) 

  Serial.print("Initializing GY30 \n");
  lightMeter.begin();
  Serial.print("GY30 Initialized \n");
}

void loop() {
  // put your main code here, to run repeatedly:
  String dataString = "";
  dataString += String("UV: ");
  dataString += String(read_UV_Sensor());
  dataString += String(", Light: ");
  dataString += String(read_Light_Sensor());
  dataString += String(", Turbidity: ");
  dataString += String(read_Turbidity_Sensor());
  dataString += String("\n ");
  Serial.print(dataString);


  File dataFile = SD.open("lentonTest.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println("write to SD");
  }
}


//##################  Custom functions #################
void GY30_Init(){

}

float read_UV_Sensor()
{
  int uvLevel = averageAnalogRead(UV_OUT);
  int refLevel = averageAnalogRead(REF_3V3);
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level
  //Serial.print(String(uvIntensity));
  delay(200);
  return(uvIntensity);  
}


uint16_t read_Light_Sensor()
{
  float lux = float(lightMeter.readLightLevel());
  delay(200);
  return (lux);
}

float read_Turbidity_Sensor()
{
  int turbLevel = averageAnalogRead(TURBIDITY_OUT);
  int refLevel = averageAnalogRead(REF_5V);
  //Use the 5V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 5.0 / refLevel * turbLevel;
  float turbidity = mapfloat(outputVoltage, 0.04, 3.58, 0.0, 2000); //need to configure just lose linear fit from spreadsheet
  //Serial.println(String(outputVoltage));
  delay(200);
  return(turbidity);  
}



//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

