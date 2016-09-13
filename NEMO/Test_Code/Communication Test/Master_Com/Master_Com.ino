// ROV
// RECEVIE FROM MASTER
// SEND TO MASTER
// RECEIVE FROM POD
// SEND TO POD
#include <EasyTransfer.h>

EasyTransfer ETin, ETout;


struct ETSEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int num;
} etdataout;

struct ETRECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int power;
} etmydata;

void setup(){
  Serial.begin(9600);
  ETout.begin(details(etdataout), &Serial);
  ETin.begin(details(etmydata), &Serial);
}

void loop(){
  etdataout.num = 23;
  //send the data
  ETout.sendData();
  if(ETin.receiveData()){
      Serial.println(etmydata.power);
  }
  delay(2000);
}
