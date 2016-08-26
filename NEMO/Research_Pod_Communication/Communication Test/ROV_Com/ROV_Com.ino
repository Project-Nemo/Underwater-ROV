// ROV
// RECEVIE FROM MASTER
// SEND TO MASTER
// RECEIVE FROM POD
// SEND TO POD

#include <SoftEasyTransfer.h>
#include <EasyTransfer.h>

/*   For Arduino 1.0 and newer, do this:   */
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3);

//create object
SoftEasyTransfer SETin, SETout; 
EasyTransfer ETin, ETout;

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int blinks;
  int pause;
} dataout;

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int power;
} mydata;

struct ETSEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int power;
} etdataout;

struct ETRECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int num;
} etmydata;

void setup(){
  Serial.begin(9600);
  mySerial.begin(9600);
  //start the library, pass in the data details and the name of the serial port.
  SETout.begin(details(dataout), &mySerial);  
  SETin.begin(details(mydata), &mySerial);  
  ETout.begin(details(etdataout), &Serial);
  ETin.begin(details(etmydata), &Serial);
}

void loop(){
  //this is how you access the variables. [name of the group].[variable name]
  dataout.blinks = random(5);
  dataout.pause = 3;
  //send the data
  SETout.sendData();
  if(SETin.receiveData()){
      Serial.println(mydata.power);
  }

  etdataout.power = 12;
  //send the data
  ETout.sendData();
  if(ETin.receiveData()){
      Serial.println(etmydata.num);
  }
  delay(2000);
}
