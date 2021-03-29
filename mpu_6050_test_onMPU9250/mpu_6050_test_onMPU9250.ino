
//#include <MPU6050.h>
#include <I2Cdev.h>
#include "MPU6050_9Axis_MotionApps41.h"
#include <WiFi.h>
#include <ros.h>
//#include <MPU9250.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>


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

int packetsize = 0 ;
MPU6050 mpu6050(0x68);
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
MPU6050 imu[12]; 
//const int MPU=0x68;
const uint8_t MPU_ADD0 = 0x68; 
const uint8_t MPU_ADD1 = 0x69; 
const uint8_t ADDRESSES[2] = {MPU_ADD0 , MPU_ADD1};

const int num_imus = 8; // 1 for the cube 
const float RADIANS_TO_DEGREES = 57.2958;
const bool printFlag = true;

uint8_t activeIMU = 0;
uint8_t devStatus;
uint16_t packetSize;
bool dmpReady[12] = {false};
//bool dmpReady = false;
uint16_t fifoCount; 
uint8_t fifoBuffer[64];

const char* ssid = "EmaroLab-WiFi";
const char* password = "walkingicub";
IPAddress server (130, 251, 13,185);//113)//195;//(192,168,43,94);//// ip of your ROS server
IPAddress ip;  
int status = WL_IDLE_STATUS;
const bool cubeFlag =false;
char a = 51;
int8_t P[14] = {0,1,2,3,4,5,15,16,17,18,19,20,21,22};
//names = ["thumb_1","thumb_2","index_1","index_2","middle_1","middle_2","6","7","8","9","10","11","12","13","14","ring_finger_1","ring_finger_2","pinkie_1","pinkie_2","back","wrist","hand2","cube"]
int8_t channels[14] = {6,6,5,5,4,4,3,3,2,2,1,1,0,0};
WiFiClient client;
unsigned int localPort = 2390;
WiFiClient clientDebug;
unsigned int debugPort = 2490;
WiFiUDP udpDebug; 
bool onoff=true;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
int16_t accel[3];
int16_t gyro[3];
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
 void sendData(  geometry_msgs::Vector3 accel, geometry_msgs::Vector3 gyro,  Quaternion q, int8_t id){
  binaryInt ID;
  ID.integerPoint = id;
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
 // packetBuffer[16] = ID.binary;
  
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



void setupWiFi()
{ 
  
  WiFi.begin(ssid, password);
  //Print to serial to find out IP address and debugging
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) 
  {
    delay(500);
    onoff=!onoff;
 //   digitalWrite (LED_BUILTIN, onoff);
        digitalWrite (D9, onoff);
    Serial.print(".....");

  }
   digitalWrite (D9, HIGH);
   delay(500);
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
   // Serial.println("found ");Serial.print(nDevices);
  }
  delay(1000);          
}



geometry_msgs::Vector3 acc;
geometry_msgs::Vector3 gyr;


void setup() {
  Serial.begin(115200);
  pinMode(D9,OUTPUT);
  digitalWrite(D9,HIGH);
  //pinMode(LED_BUILTIN,OUTPUT); 
  //digitalWrite(LED_BUILTIN,HIGH);
  setupWiFi();
  
  int devStatus; 
  
  Wire.begin();
  Wire.setClock(1000000);
  // FIFO rate was set to 200 , by setting the divider to 0, means (200 / (0+1)); 
for (int i = 6; i>0 ; i --)
    {
    tcaselect(i);
    Serial.print ("channel "); Serial.println (i);
    onoff=!onoff;
    digitalWrite(D9,onoff);
    i2cTest();
    }
    
for (int i = 0 ; i <num_imus ; i++) 
{
  tcaselect(channels[i]);
  imu[i] = MPU6050(ADDRESSES[i%2]);
  
  imu[i].initialize();
  bool cnctd  = imu[i].testConnection();
  Serial.println(cnctd ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
   if (cnctd){
   imu[0].resetFIFO(); 
   }
   
   devStatus= imu[i].dmpInitialize(); // stuck here when address is 0x69
Serial.println ("MPU initialized");
Serial.println(devStatus);
   if (devStatus == 0 ) 
    {
//  imu[i].calcGyroOffsets(true);
   // imu[i].setXGyroOffset(220);
   // imu[i].setYGyroOffset(76);
    //imu[i].setZGyroOffset(-85);
   // imu[i].setXAccelOffset(-76); //0
   // imu[i].setYAccelOffset(-2359); //0
   // imu[i].setZAccelOffset(1688); // 1688 factory default for my test chip
// 
  imu[i].CalibrateAccel(32);
  imu[i].CalibrateGyro(32);
    imu[i].PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    imu[i].setDMPEnabled(true);

        
        // enable Arduino interrupt detection
//        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//        Serial.println(F(")..."));
//        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = imu[i].getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready!"));
        dmpReady[i] = true;
        packetSize = imu[i].dmpGetFIFOPacketSize();
        fifoCount = imu[i].getFIFOCount(); 
        Serial.print ("fifocount"); Serial.println(fifoCount);
        Serial.print("PacketSize");Serial.println(packetSize); 
    }
   else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  
} 
for (int i = 0;i <num_imus ;i ++)
    {
    Serial.print ("DMP ready" ); 
    Serial.print (i);
     Serial.print(dmpReady[i]);
    }

}


void loop() {
    acc.x = 0;  acc.y = 0;  acc.z =0; gyr.x = 0;  gyr.y =0;  gyr.z = 0;
  
  
  
  // 100hz for a single sensor
 
      for (int i = 0;i <num_imus ;i++)
    {

    //Serial.print ("DMP ready" ); 
    //Serial.print (i);
    // Serial.print(dmpReady[i]);
    if (dmpReady[i]){
      
    tcaselect(channels[i]);    
    if (imu[i].GetCurrentFIFOPacket(fifoBuffer,packetSize)) 
    
    { // Get the Latest packet 

            // display quaternion values in easy matrix form: w x y z
            imu[i].dmpGetQuaternion(&q, fifoBuffer);
//            //Serial.print ("i ");  Serial.println (i);
//             Serial.print("w");Serial.print(q.w);Serial.print("w");
//            Serial.print("a");Serial.print(q.x);Serial.print("a");
//            Serial.print("b");Serial.print(q.y);Serial.print("b");
//            Serial.print("c");Serial.print(q.z);Serial.println("c");
      if (cubeFlag) 
            {
              sendData(acc,gyr,q,22); 
            }
            else {
            sendData( acc,  gyr,   q, P[i+2]);   
            }
                 onoff = !onoff;
            digitalWrite(D9,onoff);
            
    }
    }

 }
}
///// Logic done by Ale
//
//  fifoCount = mpu6050.getFIFOCount();
//  Serial.print ("fifocount"); Serial.println(fifoCount);
//      if (fifoCount == 1024)
//        {
//          mpu6050.resetFIFO();
//          if(printFlag) Serial.println(F("FIFO overflow!"));
//        }
//      else
//      {
//       if (fifoCount % packetSize != 0) 
//          {
//            mpu6050.resetFIFO();
//            if(printFlag) Serial.println(F("ERROR"));
//          }
//       else
//       {
//          while (fifoCount < packetSize) fifoCount = mpu6050.getFIFOCount();
//            Serial.print ("getting fifoCount") ; 
//            mpu6050.getFIFOBytes(fifoBuffer, packetSize);
//  
//            mpu6050.dmpGetAccel(accel, fifoBuffer);
//            mpu6050.dmpGetGyro(gyro, fifoBuffer);
//            mpu6050.dmpGetQuaternion(&q, fifoBuffer);
//            mpu6050.dmpGetEuler(euler, &q);
//            //mpu6050.dmpGetGravity(&gravity,&q);
//            mpu6050.dmpGetYawPitchRoll(ypr,&q,&gravity); 
//
//          //  sendData(accel, gyro, &q, &P[i]);
//            
//            if(printFlag){
//              Serial.println("accel");
//              Serial.print(accel[0]);
//              Serial.print(":");
//              Serial.print(accel[1]);
//              Serial.print(":");
//              Serial.println(accel[2]);
//    
//              Serial.println("gyro");
//              Serial.print(gyro[0]);
//              Serial.print(":");
//              Serial.print(gyro[1]);
//              Serial.print(":");
//              Serial.println(gyro[2]);
//    
//              Serial.println("ypr");
//              Serial.print(ypr[2]*RADIANS_TO_DEGREES, 2);
//              Serial.print(":");
//              Serial.print(-ypr[1]*RADIANS_TO_DEGREES, 2);
//              Serial.print(":");
//              Serial.println(ypr[0]*RADIANS_TO_DEGREES, 2);
//    
//              Serial.println("euler");
//              Serial.print(euler[0], 2);
//              Serial.print(":");
//              Serial.print(euler[1], 2);
//              Serial.print(":");
//              Serial.println(euler[2], 2);
//              
//              Serial.println("quaternion");
//              Serial.print(q.x, 2);
//              Serial.print(":");
//              Serial.print(q.y, 2);
//              Serial.print(":");
//              Serial.print(q.z, 2);
//              Serial.print(":");
//              Serial.println(q.w, 2);
//            }
//            }


//if (!dmpReady) return;
    // read a packet from FIFO
    //Serial.print("getcurrentPcket");Serial.println (mpu6050.GetCurrentFIFOPacket(fifoBuffer,packetSize));
  
    

 
 
      //
//
//            // display Euler angles in degrees
//            mpu6050.dmpGetQuaternion(&q, fifoBuffer);
//            mpu6050.dmpGetEuler(euler, &q);
//            Serial.print("euler\t");
//            Serial.print(euler[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(euler[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(euler[2] * 180/M_PI);
//
//
//
//            // display Euler angles in degrees
//            mpu6050.dmpGetQuaternion(&q, fifoBuffer);
//            mpu6050.dmpGetGravity(&gravity, &q);
//            mpu6050.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
//
//
//            // display real acceleration, adjusted to remove gravity
//            mpu6050.dmpGetQuaternion(&q, fifoBuffer);
//            mpu6050.dmpGetAccel(&aa, fifoBuffer);
//            mpu6050.dmpGetGravity(&gravity, &q);
//            mpu6050.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            Serial.print("areal\t");
//            Serial.print(aaReal.x);
//            Serial.print("\t");
//            Serial.print(aaReal.y);
//            Serial.print("\t");
//            Serial.println(aaReal.z);
//
//
//
//            // display initial world-frame acceleration, adjusted to remove gravity
//            // and rotated based on known orientation from quaternion
//            mpu6050.dmpGetQuaternion(&q, fifoBuffer);
//            mpu6050.dmpGetAccel(&aa, fifoBuffer);
//            mpu6050.dmpGetGravity(&gravity, &q);
//            mpu6050.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            mpu6050.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//            Serial.print("aworld\t");
//            Serial.print(aaWorld.x);
//            Serial.print("\t");
//            Serial.print(aaWorld.y);
//            Serial.print("\t");
//            Serial.println(aaWorld.z);
//
//    
//        #ifdef OUTPUT_TEAPOT
//            // display quaternion values in InvenSense Teapot demo format:
//            teapotPacket[2] = fifoBuffer[0];
//            teapotPacket[3] = fifoBuffer[1];
//            teapotPacket[4] = fifoBuffer[4];
//            teapotPacket[5] = fifoBuffer[5];
//            teapotPacket[6] = fifoBuffer[8];
//            teapotPacket[7] = fifoBuffer[9];
//            teapotPacket[8] = fifoBuffer[12];
//            teapotPacket[9] = fifoBuffer[13];
//            Serial.write(teapotPacket, 14);
//            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
//        #endif

        // blink LED to indicate activity
   //     blinkState = !blinkState;
    //    digitalWrite(LED_PIN, blinkState);
 //   }
 
  //mpu6050.update();
//  Serial.print("angleX : ");
//  Serial.print(mpu6050.getAngleX());
//  Serial.print("\tangleY : ");
//  Serial.print(mpu6050.getAngleY());
//  Serial.print("\tangleZ : ");
//  Serial.println(mpu6050.getAngleZ());
//}
