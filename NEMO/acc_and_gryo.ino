//Arduino 1.0+ Only!
//Arduino 1.0+ Only!

#include <Wire.h>
// for gyro
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

// for acc
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_DATAX0 0x32
#define ADXL345_DEVICE (0x53)  // ADXL345 device address
#define ADXL345_TO_READ (6)   // num of bytes we are going to read each time (two bytes for each axis)
#define ADXL345_ERROR 0 // indicates error is predent
#define ADXL345_READ_ERROR 1 // problem reading accel

int L3G4200D_Address = 105; //I2C address of the L3G4200D

int ax, gx;
int ay, gy;
int az, gz;

byte error_code;    // Initial state
byte _buff[6] ;  //6 bytes buffer for saving data read from the device
bool status;      // set when error occurs 

void setup(){
  Wire.begin();
  Serial.begin(9600);
  powerOn();
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  delay(1500); //wait for the sensor to be ready 
}

void loop(){
  readAccel(&ax, &ay, &az); //read the accelerometer values and store them in variables x,y,z
  // Output x,y,z values - Commented out
  Serial.print("AX:");
  Serial.print(ax);

  Serial.print(" AY:");
  Serial.print(ay);

  Serial.print(" AZ:");
  Serial.print(az);

  
  getGyroValues();  // This will update x, y, and z with new values
  Serial.print("  GX:");
  Serial.print(gx);

  Serial.print(" GY:");
  Serial.print(gy);

  Serial.print(" GZ:");
  Serial.println(gz);
}

//================= ACCEL FUNCTIONS ==========================//
void powerOn() {
    //Turning on the ADXL345
    writeTo(ADXL345_POWER_CTL, 0);   
    writeTo(ADXL345_POWER_CTL, 16);
    writeTo(ADXL345_POWER_CTL, 8); 
}

void writeTo(byte address, byte val) {
    Wire.beginTransmission(ADXL345_DEVICE); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

void readAccel(int *x, int *y, int *z) {
    readFrom(ADXL345_DATAX0, ADXL345_TO_READ, _buff); //read the acceleration data from the ADXL345
    
    // each axis reading comes in 10 bit resolution, ie 2 bytes. Least Significat Byte first!!
    // thus we are converting both bytes in to one int
    *x = (((int)_buff[1]) << 8) | _buff[0];  
    *y = (((int)_buff[3]) << 8) | _buff[2];
    *z = (((int)_buff[5]) << 8) | _buff[4];
}

void readFrom(byte address, int num, byte _buff[]) {
    Wire.beginTransmission(ADXL345_DEVICE); // start transmission to device 
    Wire.write(address);       // sends address to read from
    Wire.endTransmission();     // end transmission
    
    Wire.beginTransmission(ADXL345_DEVICE); // start transmission to device
    Wire.requestFrom(ADXL345_DEVICE, num);  // request 6 bytes from device
    
    int i = 0;
    while(Wire.available())     // device may send less than requested (abnormal)
    { 
        _buff[i] = Wire.read();  // receive a byte
        i++;
    }
    if(i != num){
        status = ADXL345_ERROR;
        error_code = ADXL345_READ_ERROR;
    }
    Wire.endTransmission();     // end transmission
}
//============== GYRO FUNCTIONS ==================//
void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  gx = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  gy = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  gz = ((zMSB << 8) | zLSB);
}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
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
