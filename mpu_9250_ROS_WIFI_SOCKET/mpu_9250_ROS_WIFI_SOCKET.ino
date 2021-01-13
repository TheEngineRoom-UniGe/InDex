

#include <ros.h>
#include <MPU9250.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <WiFi.h>
#include <std_msgs/String.h>
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
MPU9250 mpu1, mpu2, mpu3,mpu4; // mpu2 daisychained with mpu  , mpu4 daisychained with mpu3
MPU9250 mpu5;
MPU9250 mpu6;
MPU9250 mpu7;
MPU9250 mpu8;
MPU9250 mpu9;
MPU9250 mpu10;
const char* ssid = "EmaroLab-WiFi";
const char* password = "walkingicub";
IPAddress server (130, 251, 13, 113); //(192,168,43,94);//// ip of your ROS server
IPAddress ip;  
int status = WL_IDLE_STATUS;
char a = 51;
int8_t P[12] = {0,1,2,3,4,5,15,16,17,18,19,20};
WiFiClient client;
unsigned int localPort = 2390;


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
  accelerometer.integerPoint[0] = accel.x*1000;
  accelerometer.integerPoint[1] = accel.y*1000;
  accelerometer.integerPoint[2] = accel.z*1000;

  binary3Int gyroscope;
  gyroscope.integerPoint[0] = gyro.x*1000;
  gyroscope.integerPoint[1] = gyro.y*1000;
  gyroscope.integerPoint[2] = gyro.z*1000;

  byte packetBuffer[29];//[17];
  memset(packetBuffer, 0,29);// 17);

  packetBuffer[28] = ID.binary;
//  packetBuffer[16] = ID.binary;
  
  for(int i=0; i<16; i++){
    packetBuffer[i] = quaternion.binary[i];
  }
  
  for(int i=16; i<22; i++){
    packetBuffer[i] = accelerometer.binary[i-16];
  }

  for(int i=22; i<28; i++){
    packetBuffer[i] = gyroscope.binary[i-22];
  }

  Udp.beginPacket(server, localPort);
  Udp.write(packetBuffer,29);//17);
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

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();  
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
void setupWiFi()
{ bool onoff=true;
  WiFi.begin(ssid, password);
  //Print to serial to find out IP address and debugging
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) 
  {
    delay(500);
    onoff=!onoff;
    digitalWrite (LED_BUILTIN, onoff);
    
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
geometry_msgs::Vector3 acc;
geometry_msgs::Vector3 gyr;
geometry_msgs::Quaternion q;
void setup()
{   Serial.begin(115200);  
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
    acc.x = 0;  acc.y = 0;  acc.z = 0; gyr.x = 0;  gyr.y = 0;  gyr.z = 0;

    Serial.println("I am Ready for I2C");
    //DEFAULT ADDRESS 0X68
    Wire.begin();
    Wire.setClock(400000);
    delay(1000);
    tcaselect(6);Serial.println("scanning channel 6"); // 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
    i2cTest();
    tcaselect(5);Serial.println("scanning channel 5"); // 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
    i2cTest();
     tcaselect(4);Serial.println("scanning channel 4"); // 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
    i2cTest();
     tcaselect(3); Serial.println("scanning channel 3");// 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
    i2cTest();
    tcaselect(2);Serial.println("scanning channel 2"); // 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
    i2cTest();
    
    delay(1000);
    tcaselect(6);
    mpu1.setup();
    mpu2.setI2CAddress(0x69); 
    mpu2.setup();

    tcaselect(5);
   
//  //  i2cTest();
//
    mpu3.setup();
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
    //
//     tcaselect(2); // when we read from channel 2 , an error occured and all other readings fail
//    mpu9.setup(); 
//    
    delay(500);

}

void loop()
{
    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 1)
      {
        tcaselect(6);
       // delay(5); // with 10, 20 ms , works fine , always updating ,visualised using teapot // 10 ms seems more stable (26fps with teapot)
       
         
         mpu1.update();
        // delay(5);
   //     mpu1.printRawData();
      //Serial.print(mpu1.getTemperature());
      // the below code works with the PYteapot .
      // to read the temperature , we have set the AHRS to false
     // Serial.print("t1");Serial.print(mpu1.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu1.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu1.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu1.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu1.getQuaternion(3));Serial.println("c");
      
      q.x = mpu1.getQuaternion(0);  q.y = mpu1.getQuaternion(1);  q.z = mpu1.getQuaternion(2);  q.w = mpu1.getQuaternion(3);
      acc.x = mpu1.getAcc(0); acc.y = mpu1.getAcc(1); acc.x = mpu1.getAcc(2);
      gyr.x = mpu1.getGyro(0); gyr.y = mpu1.getGyro(1); gyr.z = mpu1.getGyro(2);

   sendData( acc,  gyr,   q, P[2]);
//
      mpu2.update();
//      Serial.print("2");
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu2.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu2.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu2.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu2.getQuaternion(3));Serial.println("c");
//    
   q.x = mpu2.getQuaternion(0);  q.y = mpu2.getQuaternion(1);  q.z = mpu2.getQuaternion(2);  q.w = mpu2.getQuaternion(3);
   acc.x=mpu2.getAcc(0); acc.y=mpu2.getAcc(1); acc.x=mpu2.getAcc(2);
   gyr.x=mpu2.getGyro(0); gyr.y=mpu2.getGyro(1); gyr.z=mpu2.getGyro(2);
     sendData( acc,  gyr,   q, P[3]);
     //
      tcaselect(5);
      mpu3.update();
     // delay(5); adding this delay helped to get more updates on the 0x69 address
      q.x = mpu3.getQuaternion(0);  q.y = mpu3.getQuaternion(1);  q.z = mpu3.getQuaternion(2);  q.w = mpu3.getQuaternion(3);
      acc.x=mpu3.getAcc(0); acc.y=mpu3.getAcc(1); acc.x=mpu3.getAcc(2);
      gyr.x=mpu3.getGyro(0); gyr.y=mpu3.getGyro(1); gyr.z=mpu3.getGyro(2);
      sendData( acc,  gyr,   q, P[4]);
//      Serial.print("3");
//
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu3.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu3.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu3.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu3.getQuaternion(3));Serial.println("c");
////  
        
      mpu4.update();
      q.x = mpu4.getQuaternion(0);  q.y = mpu4.getQuaternion(1);  q.z = mpu4.getQuaternion(2);  q.w = mpu4.getQuaternion(3);
      acc.x=mpu4.getAcc(0); acc.y=mpu4.getAcc(1); acc.x=mpu4.getAcc(2);
      gyr.x=mpu4.getGyro(0); gyr.y=mpu4.getGyro(1); gyr.z=mpu4.getGyro(2);
      sendData( acc,  gyr,   q, P[5]);
//      Serial.print("4");
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
      acc.x=mpu5.getAcc(0); acc.y=mpu5.getAcc(1); acc.x=mpu5.getAcc(2);
      gyr.x=mpu5.getGyro(0); gyr.y=mpu5.getGyro(1); gyr.z=mpu5.getGyro(2);
      sendData( acc,  gyr,   q, P[6]);
//     Serial.print("5");
//
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu5.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu5.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu5.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu5.getQuaternion(3));Serial.println("c");
//
      mpu6.update();
       q.x = mpu6.getQuaternion(0);  q.y = mpu6.getQuaternion(1);  q.z = mpu6.getQuaternion(2);  q.w = mpu6.getQuaternion(3);
       acc.x=mpu6.getAcc(0); acc.y=mpu6.getAcc(1); acc.x=mpu6.getAcc(2);
      gyr.x=mpu6.getGyro(0); gyr.y=mpu6.getGyro(1); gyr.z=mpu6.getGyro(2);
   sendData( acc,  gyr,   q, P[7]);
//      Serial.print("6");
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
//      Serial.print("7");
//      
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu7.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu7.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu7.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu7.getQuaternion(3));Serial.println("c");
//
      mpu8.update();
       q.x = mpu8.getQuaternion(0);  q.y = mpu8.getQuaternion(1);  q.z = mpu8.getQuaternion(2);  q.w = mpu8.getQuaternion(3);
       
       acc.x=mpu8.getAcc(0); acc.y=mpu8.getAcc(1); acc.x=mpu8.getAcc(2);
       gyr.x=mpu8.getGyro(0); gyr.y=mpu8.getGyro(1); gyr.z=mpu8.getGyro(2);
   sendData( acc,  gyr,   q, P[9]);
//
//      Serial.print("8");
//    
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu8.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu8.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu8.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu8.getQuaternion(3));Serial.println("c");


//      tcaselect(2);
//      mpu9.update();
//
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





    }
}
