// RESEARCH POD
// RECIEVE FROM ROV
// SEND TO ROV

#include <SoftEasyTransfer.h>

/*   For Arduino 1.0 and newer, do this:   */
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3);

//create object
SoftEasyTransfer ETin, ETout;

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int blinks;
  int pause;
} mydata;

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int power;
} dataout;

void setup(){
  Serial.begin(9600);
  mySerial.begin(9600);
  //start the library, pass in the data details and the name of the serial port.
  ETin.begin(details(mydata), &mySerial); 
  ETout.begin(details(dataout), &mySerial); 
}

void loop(){
  //check and see if a data packet has come in. 
  if(ETin.receiveData()){
    //this is how you access the variables. [name of the group].[variable name]
    //since we have data, we will blink it out. 
    for(int i = mydata.blinks; i>0; i--){
      Serial.println(mydata.pause);
    }
  }
  dataout.power = 10;
  //you should make this delay shorter then your transmit delay or else messages could be lost
  delay(250);
  ETout.sendData();
}

