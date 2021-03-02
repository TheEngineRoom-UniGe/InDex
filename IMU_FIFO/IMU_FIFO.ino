#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <WiFi.h>
#include <std_msgs/String.h>
#include <MPU9250_FIFO.h>
const char* ssid = "EmaroLab-WiFi";//"MiHotspot" ; // 
const char* password = "walkingicub";//"Pass123455";//
IPAddress server (130, 251, 13, 113); //(192,168,43,94);////// ip of your ROS server
IPAddress ip;  
int status = WL_IDLE_STATUS;
char a = 51;
int8_t P[13] = {0,1,2,3,4,5,15,16,17,18,19,20,21};
WiFiClient client;
unsigned int localPort = 2390;

void setupWiFi()
{ Serial.println("starting WIFI");
  bool onoff=true;
  WiFi.begin(ssid, password);
  //Print to serial to find out IP address and debugging
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) 
  {
    delay(500);
    onoff=!onoff;
    digitalWrite (LED_BUILTIN, onoff);
   //pinMode(D9,OUTPUT); 
//    digitalWrite (D9, onoff);
    Serial.print(".....");

  }
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  ip = WiFi.localIP();
  Serial.print(ip);
  Serial.println(" to access client");
  

}

typedef union {
 float floatingPoint[4];
 byte binary[16];
} binary4Float;

typedef union{
  int8_t integerPoint;
  byte binary;
} binaryInt;

typedef union{
  int16_t integerPoint[3];
  byte binary[6];
} binary3Int;

void sendData(  geometry_msgs::Vector3 accel, geometry_msgs::Vector3 gyro,  geometry_msgs::Quaternion q, int8_t id){
  binaryInt ID;
  ID.integerPoint = id;
  WiFiUDP Udp;
  binary4Float quaternion;
  quaternion.floatingPoint[0] = q.x;
  quaternion.floatingPoint[1] = q.y;
  quaternion.floatingPoint[2] = q.z;
  quaternion.floatingPoint[3] = q.w;

  binary3Int accelerometer;
  accelerometer.integerPoint[0] =accel.x;
  accelerometer.integerPoint[1] =accel.y;
  accelerometer.integerPoint[2] = accel.z;

  binary3Int gyroscope;
  gyroscope.integerPoint[0] = gyro.x;
  gyroscope.integerPoint[1] = gyro.y;
  gyroscope.integerPoint[2] = gyro.z;

  byte packetBuffer[29];
  memset(packetBuffer, 0, 29);

  packetBuffer[28] = ID.binary;
 // packetBuffer[16] = ID.binary;
  
  for(int i=0; i<16; i++){
    packetBuffer[i] = quaternion.binary[i];
  }
//  
  for(int i=16; i<22; i++){
    packetBuffer[i] = accelerometer.binary[i-16];
  }

  for(int i=22; i<28; i++){
    packetBuffer[i] = gyroscope.binary[i-22];
  }

  Udp.beginPacket(server, localPort);
  Udp.write(packetBuffer, 29);
  Udp.endPacket();  
}



class WiFiHardware {

  public:
  WiFiHardware() {};

  void init() {
    // do your initialization here. this probably includes TCP server/client setup
    client.connect(server, 11411);
  }

  // read a byte from the serial port. -1 = failure
  int read() {
    // implement this method so that it reads a byte from the TsendCP connection and returns it
    //  you may return -1 is there is an error; for example if the TCP connection is not open
    return client.read();         //will return -1 when it will works
  }

  // write data to the connection to ROS
  void write(uint8_t* data, int length) {
    // implement this so that it takes the arguments and writes or prints them to the TCP connection
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }

  // returns milliseconds since start of program
  unsigned long time() {
     return millis(); // easy; did this one for you
  }
};
//#include <MPU9250.h>
//#include "eeprom_utils.h"
//#include "EEPROM.h"
//
//  float getQuaternion(uint8_t i) const { return (i < 4) ? q[i] : 0.f; }
//
//    float getAcc(uint8_t i) const { return (i < 3) ? a[i] : 0.f; }
//    float getGyro(uint8_t i) const { return (i < 3) ? g[i] : 0.f; }
//    float getMag(uint8_t i) const { return (i < 3) ? m[i] : 0.f; }
// to get smaller array these are the raw functions
//
//
// ADC on 4D address 
MPU9250_FIFO mpu(0),mpu2(1), mpu3(2),mpu4(3),mpu5(4),mpu6(5),mpu7(6),mpu8(7),mpu9(8),mpu10(9),mpu11(10);
//MPU9250 mpu2; 
//MPU9250 mpu, mpu2, mpu3,mpu4; // mpu2 daisychained with mpu  , mpu4 daisychained with mpu3
//MPU9250 mpu5;
//MPU9250 mpu6;
//MPU9250 mpu7;
//MPU9250 mpu8;
//MPU9250 mpu9;
//MPU9250 mpu0;
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void tcaselect_w1(uint8_t i) {
  if (i > 7) return;
  Wire1.beginTransmission(0x70);
  Wire1.write(1 << i);
  int error=Wire1.endTransmission();  
  if (error) Serial.println("error connecting to multiplexer");
}

void i2cTest() {
  byte error, address;
  int nDevices;
  delay(500);
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("found ");Serial.print(nDevices);
  }
  delay(1000);          
}

void scanW1(){
  int error;
    for ( uint8_t i = 0; i <8;i++)
    {
    tcaselect_w1(i);
    Wire1.beginTransmission(0x68);
    error = Wire1.endTransmission();
    if (error == 0) 
      Serial.println("I2C device found at 0x68, second I2c");
    else {
      Serial.print("second I2c lane ,0x68 not found at MX "); Serial.println(i); }
    
    Wire1.beginTransmission(0x69);
    error = Wire1.endTransmission();
    if (error == 0) 
      Serial.println("I2C device found at 0x69, second I2c");
    else 
    {  Serial.print("second I2c lane ,0x69 not found at MX "); Serial.println(i); }
    }
}
  
void setup()
{   EEPROM.begin(512); // otherwise it wont write
    Serial.begin(115200);  
    setupWiFi();
    Serial.println("Here I Am");
    for (int i =0;i <5;i ++)
    {
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(500);  
    digitalWrite(LED_BUILTIN, LOW); 
    delay(200);  
    // does not work when on usb
    }
    
    Serial.println("I am Ready for I2C");
    //DEFAULT ADDRESS 0X68
    Wire.begin(0x68);
    Wire.setClock(400000);

    Wire1.begin (D7,D8,400000);  // Data , clock ,frequency 

    scanW1();
    delay(1000);
    
    tcaselect(6);Serial.println("scanning channel 6"); // 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
    i2cTest();
   tcaselect(5);Serial.println("scanning channel 5"); // 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
    i2cTest();
     tcaselect(4);Serial.println("scanning channel 4"); // 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
    i2cTest();
     tcaselect(3); Serial.println("scanning channel 3");// 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
    i2cTest();
    //tcaselect(2);Serial.println("scanning channel 2"); // 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
    //i2cTest();
    
    delay(1000);
    tcaselect(6);
    mpu.setup();
    mpu2.setI2CAddress(0x69);
    mpu2.setup();
   
    tcaselect(5);
    mpu3.setup();
  
    Serial.println(" Send C to calibrate magnetometer1 and D for magnetometer 2, E for 3, L for lazy calibration "); 
    delay (3000); 
    

    mpu4.setI2CAddress(0x69); 
    mpu4.setup();
//  
    tcaselect(4);
      
    mpu5.setup(); 
    mpu6.setI2CAddress(0x69); 
    mpu6.setup();
    
    tcaselect(3);
    mpu7.setup(); 
    mpu8.setI2CAddress(0x69); 
    mpu8.setup();
    
//     tcaselect(2); // when we read from channel 2 , an error occured and all other readings fail
////  mpu9.setI2CAddress(0x69); 
////
//    mpu9.setup(); 
//    
    Serial.println("setup MPU on second multiplexer"); 
    tcaselect_w1(6);
    mpu10.setup(Wire1);

    Serial.println("setup MPU on second multiplexer"); 
    tcaselect_w1(7);
    mpu11.setup(Wire1);
    while (Serial.available() >0)
    {
   
    int a = Serial.read(); 
    switch (a){
    case 'C':
    {
      tcaselect(6);
      // calibrate when you want to
      mpu.clearCalibration();
      mpu.calibrateAccelGyro();
      mpu.calibrateMag();
      //   mpu.calibrateMag();
      
      // save to eeprom
      mpu.saveCalibration(true);
      //
      break;
    }
    case 'D':
    {  
      tcaselect(6);
      mpu2.clearCalibration();
      mpu2.calibrateAccelGyro();
      mpu2.calibrateMag();
      //   mpu.calibrateMag();
      
      // save to eeprom
      mpu2.saveCalibration(true);
      break;
    }
    case 'E':
    { 
      tcaselect(5);
      mpu3.clearCalibration();
      mpu3.calibrateAccelGyro();
      mpu3.calibrateMag();
      //   mpu.calibrateMag();
      
      // save to eeprom
      mpu3.saveCalibration(true);
      break;
    }
    case 'L':
    {
    Serial.println("lazy calibration");
    mpu.lazyCalibration(3); // number of sensors in total
    }
    }
    }
 //  mpu.calibrateAccelGyro();
    if (mpu.isCalibrated())
    mpu.loadCalibration(); // currently it loads for instance called mpu.
    mpu.printCalibrationfromEEPROM();
    mpu.printCalibration();

      if (mpu2.isCalibrated())
    mpu2.loadCalibration(); // currently it loads for instance called mpu.
    mpu2.printCalibrationfromEEPROM();
    mpu2.printCalibration();
//   // mpu = mpu; 
//    //mpu.printCalibration
    
   if (mpu3.isCalibrated())
    mpu3.loadCalibration(); // currently it loads for instance called mpu.
    mpu3.printCalibrationfromEEPROM();
    mpu3.printCalibration();
 
    delay(500);

}
geometry_msgs::Vector3 acc;
geometry_msgs::Vector3 gyr;
geometry_msgs::Quaternion q;

void loop()
{
 
//    {  tcaselect(6);
//      delay(200);
//       mpu.update();
//      // mpu.print();
//       // mpu.printRawData(); 
//        //mpu.printRollPitchYaw();
//        Serial.print("mpu"); 
//        Serial.print("roll  (x-forward (north)) : ");
//        Serial.print(mpu.getRoll());
//        Serial.print(" pitch (y-right (east))    : ");
//        Serial.print(mpu.getPitch());
//        Serial.print(" yaw   (z-down (down))     : ");
//        Serial.println(mpu.getYaw());
//       delay(200);
//        mpu2.update();
//        Serial.print ("MPU2"); 
//        Serial.print(" roll  (x-forward (north)) : ");
//        Serial.print(mpu2.getRoll());
//        Serial.print(" pitch (y-right (east))    : ");
//        Serial.print(mpu2.getPitch());
//        Serial.print(" yaw   (z-down (down))     : ");
//        Serial.println(mpu2.getYaw());

      {
    //static uint32_t prev_ms = millis();
  //  if ((millis() - prev_ms) > 1)
      {
        tcaselect(6);
       // delay(5); // with 10, 20 ms , works fine , always updating ,visualised using teapot // 10 ms seems more stable (26fps with teapot)
       
      //   prev_ms = millis();
         mpu.update();
         
        // delay(5);
   //     mpu.printRawData();
      //Serial.print(mpu.getTemperature());
      // the below code works with the PYteapot .
      // to read the temperature , we have set the AHRS to false
     // Serial.print("t1");Serial.print(mpu.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu.getQuaternion(3));Serial.println("c");
      acc.x=0; acc.y=0; acc.z=0;
      gyr.x=0; gyr.y=0; gyr.z=0;
      q.x = mpu.getQuaternion(0);  q.y = mpu.getQuaternion(1);  q.z = mpu.getQuaternion(2);  q.w = mpu.getQuaternion(3);
//      acc.x = mpu.getAcc(0); acc.y = mpu.getAcc(1); acc.x = mpu.getAcc(2);
//      gyr.x = mpu.getGyro(0); gyr.y = mpu.getGyro(1); gyr.z = mpu.getGyro(2);

     sendData( acc,  gyr,   q, P[2]);
//
      mpu2.update();
    //  Serial.print("2");
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu2.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu2.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu2.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu2.getQuaternion(3));Serial.println("c");
//    
   q.x = mpu2.getQuaternion(0);  q.y = mpu2.getQuaternion(1);  q.z = mpu2.getQuaternion(2);  q.w = mpu2.getQuaternion(3);
//   acc.x=mpu2.getAcc(0); acc.y=mpu2.getAcc(1); acc.x=mpu2.getAcc(2);
//   gyr.x=mpu2.getGyro(0); gyr.y=mpu2.getGyro(1); gyr.z=mpu2.getGyro(2);
     sendData( acc,  gyr,   q, P[3]);
     //
      tcaselect(5);
      mpu3.update();
     // delay(5); adding this delay helped to get more updates on the 0x69 address
      q.x = mpu3.getQuaternion(0);  q.y = mpu3.getQuaternion(1);  q.z = mpu3.getQuaternion(2);  q.w = mpu3.getQuaternion(3);
//      acc.x=mpu3.getAcc(0); acc.y=mpu3.getAcc(1); acc.x=mpu3.getAcc(2);
//      gyr.x=mpu3.getGyro(0); gyr.y=mpu3.getGyro(1); gyr.z=mpu3.getGyro(2);
      sendData( acc,  gyr,   q, P[4]);
    //  Serial.print("3");
//
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu3.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu3.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu3.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu3.getQuaternion(3));Serial.println("c");
////  
        
      mpu4.update();
      q.x = mpu4.getQuaternion(0);  q.y = mpu4.getQuaternion(1);  q.z = mpu4.getQuaternion(2);  q.w = mpu4.getQuaternion(3);
//      acc.x=mpu4.getAcc(0); acc.y=mpu4.getAcc(1); acc.x=mpu4.getAcc(2);
//      gyr.x=mpu4.getGyro(0); gyr.y=mpu4.getGyro(1); gyr.z=mpu4.getGyro(2);
      sendData( acc,  gyr,   q, P[5]);
   //   Serial.print("4");
//
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu4.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu4.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu4.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu4.getQuaternion(3));Serial.println("c");
////
     tcaselect(4);
//
     mpu5.update();
      q.x = mpu5.getQuaternion(0);  q.y = mpu5.getQuaternion(1);  q.z = mpu5.getQuaternion(2);  q.w = mpu5.getQuaternion(3);
//      acc.x=mpu5.getAcc(0); acc.y=mpu5.getAcc(1); acc.x=mpu5.getAcc(2);
//      gyr.x=mpu5.getGyro(0); gyr.y=mpu5.getGyro(1); gyr.z=mpu5.getGyro(2);
      sendData( acc,  gyr,   q, P[6]);
   //  Serial.print("5");
//
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu5.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu5.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu5.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu5.getQuaternion(3));Serial.println("c");
//
      mpu6.update();
       q.x = mpu6.getQuaternion(0);  q.y = mpu6.getQuaternion(1);  q.z = mpu6.getQuaternion(2);  q.w = mpu6.getQuaternion(3);
//       acc.x=mpu6.getAcc(0); acc.y=mpu6.getAcc(1); acc.x=mpu6.getAcc(2);
//      gyr.x=mpu6.getGyro(0); gyr.y=mpu6.getGyro(1); gyr.z=mpu6.getGyro(2);
   sendData( acc,  gyr,   q, P[7]);
   //   Serial.print("6");
//
//      Serial.print("w");Serial.print(mpu6.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu6.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu6.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu6.getQuaternion(3));Serial.println("c");
//
      tcaselect(3);
       mpu7.update();
       q.x = mpu7.getQuaternion(0);  q.y = mpu7.getQuaternion(1);  q.z = mpu7.getQuaternion(2);  q.w = mpu7.getQuaternion(3);
       acc.x=mpu7.getAcc(0); acc.y=mpu7.getAcc(1); acc.x=mpu7.getAcc(2);
       gyr.x=mpu7.getGyro(0); gyr.y=mpu7.getGyro(1); gyr.z=mpu7.getGyro(2);
       sendData( acc,  gyr,   q, P[8]);
//    //  Serial.print("7");
////      
////      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
////      Serial.print("w");Serial.print(mpu7.getQuaternion(0));Serial.print("w");
////      Serial.print("a");Serial.print(mpu7.getQuaternion(1));Serial.print("a");
////      Serial.print("b");Serial.print(mpu7.getQuaternion(2));Serial.print("b");
////      Serial.print("c");Serial.print(mpu7.getQuaternion(3));Serial.println("c");
////
      mpu8.update();
       q.x = mpu8.getQuaternion(0);  q.y = mpu8.getQuaternion(1);  q.z = mpu8.getQuaternion(2);  q.w = mpu8.getQuaternion(3);
       
       acc.x=mpu8.getAcc(0); acc.y=mpu8.getAcc(1); acc.x=mpu8.getAcc(2);
       gyr.x=mpu8.getGyro(0); gyr.y=mpu8.getGyro(1); gyr.z=mpu8.getGyro(2);
       sendData( acc,  gyr,   q, P[9]);
      // Serial.print("8");
      //tcaselect(2);
  //    mpu9.update();

//      Serial.print("9");
//
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu9.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu9.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu9.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu9.getQuaternion(3));Serial.println("c");
//    Wire.requestFrom(0x4D, 1);    // request potentiometer position from slave 0x08
//    while(Wire.available()) {        // read response from slave 0x08
//    Serial.print( Wire.read());
//      }
//         
//
    //Serial.print("time elapsed");Serial.println(millis() - prev_ms);


    //Serial.println("setup MPU on second multiplexer"); 
    tcaselect_w1(6);
    mpu10.update();
    q.x = mpu10.getQuaternion(0);  q.y = mpu10.getQuaternion(1);  q.z = mpu10.getQuaternion(2);  q.w = mpu10.getQuaternion(3);
//       
       acc.x=mpu10.getAcc(0); acc.y=mpu10.getAcc(1); acc.x=mpu10.getAcc(2);
       gyr.x=mpu10.getGyro(0); gyr.y=mpu10.getGyro(1); gyr.z=mpu10.getGyro(2);
   sendData( acc,  gyr,   q, P[11]);
//
    tcaselect_w1(7);
    mpu11.update();
//
       q.x = mpu11.getQuaternion(0);  q.y = mpu11.getQuaternion(1);  q.z = mpu11.getQuaternion(2);  q.w = mpu11.getQuaternion(3);
//       
//       acc.x=mpu11.getAcc(0); acc.y=mpu11.getAcc(1); acc.x=mpu11.getAcc(2);
//       gyr.x=mpu11.getGyro(0); gyr.y=mpu11.getGyro(1); gyr.z=mpu11.getGyro(2);
   sendData( acc,  gyr,   q, P[12]);
//    
    
    }
}
}
