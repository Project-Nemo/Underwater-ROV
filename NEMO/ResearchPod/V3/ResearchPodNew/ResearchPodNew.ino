#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <SoftEasyTransfer.h>
#include <EasyTransfer.h>
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>    //we have to include the SoftwareSerial library, or else we can't use it

#define ONE_WIRE_BUS_1 4

//Hardware pin definitions
#define UVOUT A0 //Output from the sensor (ML8511)
#define REF_3V3 A1 //3.3V power on the Arduino board (ML8511)
#define REF_5V A2
#define TURBIDITY_OUT A7 
#define chipSelect 10 //SPI Chip select pin

//#################AS Conductivity##############################
                       
#define rx 2                                          //define what pin rx is going to be for conductivity
#define tx 3                                          //define what pin tx is going to be  for conductivity
SoftwareSerial ASSerial(rx, tx);            //define how the soft serial port is going to communicate with condiuctivity sensor Micro
String inputstring = "";                              //a string to hold incoming data from the PC
String sensorstring = "";                             //a string to hold the data from the Atlas Scientific product
boolean input_string_complete = false;                //have we received all the data from the PC
boolean sensor_string_complete = false;               //have we received all the data from the Atlas Scientific product

//#################AS Conductivity################################

BH1750 lightMeter; //Create Light meter object

bool sdCardAvailable = false;
String fileName;
OneWire oneWire_in(ONE_WIRE_BUS_1);
DallasTemperature tempSensor(&oneWire_in);

//##################Communcation############################
//SoftwareSerial podSerial(7,8);
//SoftEasyTransfer ETin, ETout;
EasyTransfer ETin, ETout;


struct RESEARCH_POD_RECEIVE_DATA {
  int ROVPressure;  // ROV depth reading
} podDataIn;

struct RESEARCH_POD_SEND_DATA {
  String SensorData; 
  int PodPower;
  int PodState;
} podDataOut;

//##########################################################

void setup() {                                        //set up the hardware
  /*Atlas Scientific Conductivity*/
  Serial.begin(9600);                                 //set baud rate for the hardware serial port_0 to 9600
  ASSerial.begin(9600);                               //set baud rate for the software serial port to 9600
  inputstring.reserve(10);                            //set aside some bytes for receiving data from the PC
  sensorstring.reserve(30);                           //set aside some bytes for receiving data from Atlas Scientific product

  /*ROV Communication*/
  //podSerial.begin(2400);
  //ETin.begin(details(podDataIn), &podSerial);  
  //ETout.begin(details(podDataOut), &podSerial);
  ETin.begin(details(podDataIn), &Serial);  
  ETout.begin(details(podDataOut), &Serial);

  /*SD Card Initializing Code*/
  if (!SD.begin(chipSelect)) {
    //Serial.println("SD Card failed, or not present");
    sdCardAvailable = false;
  } else {
    //Serial.println("SD card initialized.");
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
    //Serial.print("file name to save data is :");
    //Serial.println(fileName);
    File SD_file = SD.open(fileName, FILE_WRITE);
        // if the file is available, write to it:
        if (SD_file) {
          SD_file.println("Time,Temp,Light,Turbidity,UV,EC,TSD,SAL,GRAV");
          //Serial.println("Time,Temp,Light,Turbidity,UV,EC,TSD,SAL,GRAV");
          SD_file.close();;
        } //else
          //Serial.println("Error writing to SD");
  }
  
  /*DS18b20 Dallas one wire*/
  tempSensor.begin();

  /*BH1750 Light Meter*/
  lightMeter.begin();

  /*UV ML8511*/
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
  

  
}

void loop() {
  String datastring = "";

  /*ROV Communications*/
  get_power();
  ETout.sendData();
  
  /* Pressure Data
  if(ETin.receiveData()){
    //dataString += String("Pressure: ");
    dataString += String(podDataIn.ROVPressure);
    dataString += ",";
  }
  */
  
  if(Serial.available() >1){    
    String command;
    command = Serial.readString();
    ASSerial.print(command); 
    //Serial.println("command:");
    //Serial.print(command);
  }

  if (input_string_complete) {                        //if a string from the PC has been received in its entirety
    ASSerial.print(inputstring);                      //send that string to the Atlas Scientific product
    ASSerial.print('\r');                             //add a <CR> to the end of the string
    inputstring = "";                                 //clear the string
    input_string_complete = false;                    //reset the flag used to tell if we have received a completed string from the PC
  }

  if (ASSerial.available() > 0) {                     //if we see that the Atlas Scientific product has sent a character
    char inchar = (char)ASSerial.read();              //get the char we just received
    sensorstring += inchar;                           //add the char to the var called sensorstring
    if (inchar == '\r') {                             //if the incoming character is a <CR>
      sensor_string_complete = true;                  //set the flag
    }
  }


  if (sensor_string_complete == true) {               //if a string from the Atlas Scientific product has been received in its entirety
    if (isdigit(sensorstring[0]) == false) {          //if the first character in the string is a digit
      //Serial.println(sensorstring);                   //send that string to the PC's serial monitor
    }
    else                                              //if the first character in the string is NOT a digit
    {
      datastring += millis()/1000.00;                       //Add timestamp to string
      datastring += ',';
      datastring += get_temp();                       //Add temp to string
      datastring += ',';
      datastring += lightMeter.readLightLevel();      //Add light to string
      datastring += ',';
      datastring += get_turbidity();                  //Add turbidity to string
      datastring += ',';
      datastring += get_UV();                         //Add UV to string
      datastring += ',';
      datastring += sensorstring;                     //Add AS conductivity to string
      //Serial.println(datastring);
      if(sdCardAvailable){
        File SD_file = SD.open(fileName, FILE_WRITE);
        // if the file is available, write to it:
        if (SD_file) {
          podDataOut.SensorData = datastring;
          SD_file.println(datastring);
          SD_file.close();
          //Serial.println("write to SD successful");
        } //else
          //Serial.println("Error writing to SD");
      }
    }
    sensorstring = "";                                //clear the string
    sensor_string_complete = false;                   //reset the flag used to tell if we have received a completed string from the Atlas Scientific product
  }
}

//returns temperature value
float get_temp(){
  tempSensor.requestTemperatures();
  return tempSensor.getTempCByIndex(0);
}

//returns turbidity value
float get_turbidity(){
  float turbidity;
  float ntu;
  turbidity = averageAnalogRead(TURBIDITY_OUT)-65;
  turbidity *= 0.00488; // 5/1024 = 0.00488 --> converts to voltage
  /*Linear Approximation of Turbidity*/
  if (turbidity > 2.8)
    ntu = (turbidity-4.3856)/-0.0011;
  else if(turbidity >1.8)
    ntu = (turbidity - 3.8183)/-0.0007;
  else
    ntu = (turbidity - 3.1929)/-0.0005;
  return ntu;
}

//returns UV value
float get_UV(){
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level

  return uvIntensity;
}

//Takes an average of readings on a given pin
//Returns the average ---from ML8511 example
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

//The Arduino Map function but for floats ---from ML8511 example
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int get_power(){
  float limit = float(1023); //Take from reference pin?
  float currentRead = float(analogRead(A6));
  float voltageRead = float(analogRead(A6));
  float current = ((currentRead/limit)*5)/33.0;
  float voltage = (voltageRead/limit)*10;
  podDataOut.PodPower = voltage*current;       // watts 
  podDataOut.PodState = 1;
  
}

