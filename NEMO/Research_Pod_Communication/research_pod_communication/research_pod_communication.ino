#include <EasyTransfer.h> // Bill Porter's Easy Transfer Library
#include <SoftwareSerial.h> 
#include <Wire.h>

//EasyTransfer ETpodIn, ETpodOut;
SoftwareSerial podSerial = SoftwareSerial(7,8);
EasyTransfer ETin, ETout;

struct RESEARCH_POD_RECEIVE_DATA {
  int ROVDepth;  // ROV depth reading
};

struct RESEARCH_POD_SEND_DATA {
  int PodPower;    // watts 
  int PodState;    // 1 if pod functioning correctly, 0 otherwise
};

RESEARCH_POD_RECEIVE_DATA indata;  // podOut
RESEARCH_POD_SEND_DATA outdata;     // podIn

void setup() {
  Wire.begin(); 
  Serial.begin(9600);
  // put your setup code here, to run once:
  podSerial.begin(9600);
  // Begin Serial communication with research pod
  ETin.begin(details(indata), &podSerial);
  ETout.begin(details(outdata), &podSerial);
}

void loop() {
  // put your main code here, to run repeatedly:
  ETout.sendData();
  ETin.receiveData();
  Serial.println(indata.ROVDepth);
}
