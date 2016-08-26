
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS_1 4

//Hardware pin definitions
#define UV_OUT A0 //Output from the sensor (ML8511)
#define REF_3V3 A1 //3.3V power on the Arduino board (ML8511)
#define REF_5V A2
#define TURBIDITY_OUT A7 
#define chipSelect 10 //SPI Chip select pin

//#################AS Conductivity########################
#include <SoftwareSerial.h>                           //we have to include the SoftwareSerial library, or else we can't use it
#define rx 8                                          //define what pin rx is going to be
#define tx 9                                          //define what pin tx is going to be
SoftwareSerial myserial(rx, tx);                      //define how the soft serial port is going to work
String inputstring = "";                              //a string to hold incoming data from the PC
String sensorstring = "";                             //a string to hold the data from the Atlas Scientific product
boolean input_string_complete = false;                //have we received all the data from the PC
boolean sensor_string_complete = false;               //have we received all the data from the Atlas Scientific product
//#################AS Conductivity########################



BH1750 lightMeter; //Create Light meter object

bool sdCardAvailable = false;
String fileName;
OneWire oneWire_in(ONE_WIRE_BUS_1);
DallasTemperature tempSensor(&oneWire_in);

//#################################################################
//#################################################################
//#################################################################
//#################################################################

void setup() {
  
   // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Wire.begin();               //begin I2C
  myserial.begin(9600);       //set baud rate for the software serial port to 9600 for AS CS
  inputstring.reserve(10);    //set aside some bytes for receiving data from the PC
  sensorstring.reserve(30);   //set aside some bytes for receiving data from Atlas Scientific product
  tempSensor.begin();
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  //###################################################
  //             SD Card Initializing Code
  Serial.println("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    sdCardAvailable = false;
  } else {
    Serial.println("card initialized.");
    sdCardAvailable = true;
    int dataNumber = 0;
    fileName = "data" + String(dataNumber) + ".csv";
    while(SD.exists(fileName)){
      dataNumber ++;
      fileName = "data" + String(dataNumber) + ".csv";
      if(dataNumber >99){
        dataNumber = 0;
        fileName = "data" + String(dataNumber) + ".csv";
        break;
      }
    }
    Serial.print("file name to save data is :");
    Serial.println(fileName);
    File SD_file = SD.open(fileName, FILE_WRITE);
        // if the file is available, write to it:
        if (SD_file) {
          SD_file.println("EC,TSD,SAL,GRAV,UV,LIGHT,Turbidity,Temperature");
          Serial.println("EC,TSD,SAL,GRAV,UV,LIGHT,Turbidity,Temperature");
          SD_file.close();
          Serial.println("write to SD successful");
        } else
          Serial.println("Error writing to SD");
  }
    
  
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


//#################################################################
//#################################################################
//#################################################################
//#################################################################

void loop() {
  // put your main code here, to run repeatedly:


//#################AS Conductivity########################
if (input_string_complete) {                        //if a string from the PC has been received in its entirety
    myserial.print(inputstring);                      //send that string to the Atlas Scientific product
    myserial.print('\r');                             //add a <CR> to the end of the string
    inputstring = "";                                 //clear the string
    input_string_complete = false;                    //reset the flag used to tell if we have received a completed string from the PC
  }

  if (myserial.available() > 0) {                     //if we see that the Atlas Scientific product has sent a character
    char inchar = (char)myserial.read();              //get the char we just received
    sensorstring += inchar;                           //add the char to the var called sensorstring
    if (inchar == '\r') {                             //if the incoming character is a <CR>
      sensor_string_complete = true;                  //set the flag
    }
  }


  if (sensor_string_complete == true) {               //if a string from the Atlas Scientific product has been received in its entirety
    if (isdigit(sensorstring[0]) == false) {          //if the first character in the string is a digit
      Serial.println(sensorstring);                   //send that string to the PC's serial monitor
    }
    else                                              //if the first character in the string is NOT a digit
    {
      String dataString = "";
      //dataString += String("UV:, ");
      dataString += ",";
      dataString += String(read_UV_Sensor());
      
      //dataString += String(", Light:, ");
      dataString += ",";
      dataString += String(read_Light_Sensor());
      
      //dataString += String(", Turbidity:, ");
      dataString += ",";
      dataString += String(read_Turbidity_Sensor());
      
      tempSensor.requestTemperatures();
      //dataString += String(", Temperature:, ");
      dataString += ",";
      dataString += String(tempSensor.getTempCByIndex(0));

      //dataString += String(", Conductivity_TDS_Salinity_GRAV:,");
      //dataString += sensorstring;
      dataString += String("\n");
      Serial.print(sensorstring);
      Serial.print(dataString);
      if(sdCardAvailable){
        File SD_file = SD.open(fileName, FILE_WRITE);
        // if the file is available, write to it:
        if (SD_file) {
          SD_file.println(dataString);
          SD_file.close();
          Serial.println("write to SD successful");
        } else
          Serial.println("Error writing to SD");
      } else
          Serial.println("SD card not available");
    }
    sensorstring = "";                                //clear the string
    sensor_string_complete = false;                   //reset the flag used to tell if we have received a completed string from the Atlas Scientific product
  }
//#################AS Conductivity########################
  
  
}
//#################################################################
//#################################################################
//#################################################################
//#################################################################

//##################  Custom functions #################

float read_UV_Sensor()
{
  int uvLevel = averageAnalogRead(UV_OUT);
  int refLevel = averageAnalogRead(REF_3V3);
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level
  //Serial.print(String(uvIntensity));
  return(uvIntensity);  
}


uint16_t read_Light_Sensor()
{
  float lux;
  lux += float(lightMeter.readLightLevel());
  return (lux);
}

void serialEvent() {                                  //if the hardware serial port_0 receives a char
  inputstring = Serial.readStringUntil(13);           //read the string until we see a <CR>
  input_string_complete = true;                       //set the flag used to tell if we have received a completed string from the PC
}

float read_Turbidity_Sensor()
{
  int turbLevel = averageAnalogRead(TURBIDITY_OUT);
  int refLevel = averageAnalogRead(REF_5V);
  //Use the 5V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 5.0 / refLevel * turbLevel;
  float turbidity = mapfloat(outputVoltage, 0.04, 3.58, 0.0, 2000); //need to configure just lose linear fit from spreadsheet
  //Serial.println(String(outputVoltage));
  return(turbidity);  
}


//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  unsigned int numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++){
    runningValue += analogRead(pinToRead);
    delay(5);
  }
  runningValue /= numberOfReadings;

  return(runningValue);  
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
