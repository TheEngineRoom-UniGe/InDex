/// IMU publish with MPU9250 and ESP32


#include <ros.h>
#include <MPU9250.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <WiFi.h>
#include <std_msgs/String.h>


//////////////////////
// WiFi Definitions //
//////////////////////
//#include "arduino_secrets.h"


char hello[13] = "hello world!";
const char* ssid = "MiHotspot"; //"EmaroLab-WiFi";
const char* password = "Pass123455";// "walkingicub";
MPU9250 mpu; 
// tested successfully on VM
IPAddress server(192,168,43,94);//(130, 251, 13, 113); // ip of your ROS server
IPAddress ip;  //Storage local IP address
int status = WL_IDLE_STATUS;
char a = 51;
int8_t P[12] = {0,1,2,3,4,5,15,16,17,18,19,20};
WiFiClient client;
unsigned int localPort = 2390;
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();  
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

void sendData(  geometry_msgs::Vector3 accel, geometry_msgs::Vector3 gyro,  geometry_msgs::Quaternion q, int8_t *id){
  binaryInt ID;
  ID.integerPoint = *id;
  WiFiUDP Udp;
  binary4Float quaternion;
  quaternion.floatingPoint[0] = q.x;
  quaternion.floatingPoint[1] = q.y;
  quaternion.floatingPoint[2] = q.z;
  quaternion.floatingPoint[3] = q.w;

  binary3Int accelerometer;
  accelerometer.integerPoint[0] = accel.x;
  accelerometer.integerPoint[1] = accel.y;
  accelerometer.integerPoint[2] = accel.z;

  binary3Int gyroscope;
  gyroscope.integerPoint[0] = gyro.x;
  gyroscope.integerPoint[1] = gyro.y;
  gyroscope.integerPoint[2] = gyro.z;

  byte packetBuffer[29];
  memset(packetBuffer, 0, 29);

  packetBuffer[28] = ID.binary;
  
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


//ROS applicable

ros::NodeHandle_<WiFiHardware> nh;
std_msgs::String msg;
ros::Publisher string("outString", &msg);

sensor_msgs::Imu imu_msg; 
ros::Publisher imu_pub("imu", &imu_msg);


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
void i2cTest() {
  byte error, address;
  int nDevices;
  delay(2000);
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
    Serial.println("done\n");
  }
  delay(5000);          
}

void setup() {
  //Configure pins for adafruit ATWINC1500 breakout
//  WiFi.setPins(8,7,4);


  Serial.begin(115200);
  setupWiFi();
  delay(1000);
  //  Serial.println("pin number is ");
  //  Serial.println (D9);
//  pinMode (D9, OUTPUT);
   pinMode (LED_BUILTIN, OUTPUT);
 
  //digitalWrite (D9, HIGH);  // turn on the LED
    digitalWrite (LED_BUILTIN, HIGH);  // turn on the LED
  Wire.begin();
  tcaselect(6); // 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
  i2cTest();
  
  delay(500);
  mpu.setI2CAddress(0x68);  mpu.setup();
  
  Serial.println("initializing IMU");
  delay(2500);

  nh.initNode();
  // nh.advertise(string);
  nh.advertise(imu_pub);
}

void loop() {

//  Serial.println("pin number is ");
//  Serial.println (D9);
//  hello[12]=a;
//  //a++;
//  msg.data = hello;
//  string.publish(&msg);
//  nh.spinOnce();
//  delay(1000);
  mpu.update();
  //Serial.println("updating IMU");
  geometry_msgs::Vector3 acc;
 // acc.x = 1.0; acc.y = 2.0;acc.z=3.0;
  acc.x = mpu.getAcc(0);
  acc.y = mpu.getAcc(1);
  acc.z = mpu.getAcc(2);
  //Serial.println ("vector3");
  geometry_msgs::Vector3 gyr;
 // gyr.x = 4.0; gyr.y = 5.0;gyr.z=6.0;
  gyr.x = mpu.getGyro(0);
  gyr.y = mpu.getGyro(1);
  gyr.z = mpu.getGyro(2);
  
//  geometry_msgs::Vector3 mag; 
//  mag.x=6.0;mag.y=6.0;mag.z=8.0;
//  
  geometry_msgs::Quaternion q;
  //q.x=9.0;q.y=10.0;q.z=11.0;q.w=12.0;
  q.x = mpu.getQuaternion(0);
  q.y = mpu.getQuaternion(1);
  q.z = mpu.getQuaternion(2);
  q.w = mpu.getQuaternion(3);
  sendData( acc,  gyr,   q, &P[0]);

  Serial.println (q.x);Serial.print(q.y);Serial.print(q.z);Serial.print(q.w);
  //
//  imu_msg.linear_acceleration = acc; 
//  imu_msg.orientation=q;
//  imu_msg.angular_velocity = gyr; 
//  imu_pub.publish( &imu_msg );
//    //imu_msg.mag =mag;
  //mpu.printRawData();
  //nh.spinOnce();
  delay(10);
}
