#include<Wire.h>
#include<WiFiNINA.h>
#include <WiFiUdp.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//Communication variables
WiFiUDP Udp;
//char ssid[] = "EmaroLab-WiFi";
//char pass[] = "walkingicub";
char ssid[] = "MiHotspot";
char pass[] = "Pass123455";
unsigned int localPort = 2390;
//IPAddress address(130,251,13,72);
IPAddress address(192,168,43,94);
int status = WL_IDLE_STATUS;


//IMUs variables
MPU6050 mpu[12];

const int MPU=0x68;
int8_t P[12] = {0,1,2,3,4,5,15,16,17,18,19,20};
const int num_imus = 1;
const float RADIANS_TO_DEGREES = 57.2958;
const bool printFlag = true;

uint8_t activeIMU = 0;
uint8_t devStatus;
uint16_t packetSize;
bool dmpReady[12] = {false};
uint16_t fifoCount; 
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];
float euler[3];
int16_t accel[3];
int16_t gyro[3];

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

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void wifiSetup(){
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(3000);
  }

  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  Udp.begin(localPort);
}

void sendData(int16_t *accel, int16_t *gyro, Quaternion *q, int8_t *id){
  binaryInt ID;
  ID.integerPoint = *id;
  
  binary4Float quaternion;
  quaternion.floatingPoint[0] = q->x;
  quaternion.floatingPoint[1] = q->y;
  quaternion.floatingPoint[2] = q->z;
  quaternion.floatingPoint[3] = q->w;

  binary3Int accelerometer;
  accelerometer.integerPoint[0] = accel[0];
  accelerometer.integerPoint[1] = accel[1];
  accelerometer.integerPoint[2] = accel[2];

  binary3Int gyroscope;
  gyroscope.integerPoint[0] = gyro[0];
  gyroscope.integerPoint[1] = gyro[1];
  gyroscope.integerPoint[2] = gyro[2];

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

  Udp.beginPacket(address, localPort);
  Udp.write(packetBuffer, 29);
  Udp.endPacket();  
}

void setup() {
  // put your setup code here, to run once:
  Wire.setClock(1000000);
  Wire.begin();

  Serial.begin(115200);
  while (!Serial);

  wifiSetup();

  Serial.println(F("Initializing I2C devices..."));
  for(int i=0; i<num_imus; i++){
    mpu[i].initialize();
 //   pinMode(P[i], OUTPUT);
  }

//  for(int i=0; i<num_imus; i++){
//    digitalWrite(P[i], HIGH);
//  }
}
//  
//  for(int i=0; i<num_imus; i++){
//    digitalWrite(P[activeIMU], HIGH);
//    digitalWrite(P[i], LOW);
//    activeIMU = i;
//    
//    Serial.print(i);
//    Serial.print(" ");
//    Serial.println(mpu[i].testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
//
//    Serial.println(F("Initializing DMP..."));
//    devStatus = mpu[i].dmpInitialize();
//
//    if (devStatus == 0) {
//      // turn on the DMP, now that it's ready
//      Serial.println(F("Enabling DMP..."));
//      mpu[i].setDMPEnabled(true);
//      Serial.println(F("DMP ready! Waiting for first interrupt..."));
//      dmpReady[i] = true;
//
//      // Set the full scale range of the gyro
//      //uint8_t FS_SEL = 0;
//      //mpu.setFullScaleGyroRange(FS_SEL);
//
//      // get default full scale value of gyro - may have changed from default
//      // function call returns values between 0 and 3
//      //uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
//      //Serial.print("FS_SEL = ");
//      //Serial.println(READ_FS_SEL);
//      //GYRO_FACTOR = 131.0/(FS_SEL + 1);
//        
//      // get default full scale value of accelerometer - may not be default value.  
//      // Accelerometer scale factor doesn't reall matter as it divides out
//      //uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
//      //Serial.print("AFS_SEL = ");
//      //Serial.println(READ_AFS_SEL);
//      //ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);
//        
//      // Set the full scale range of the accelerometer
//      //uint8_t AFS_SEL = 0;
//      //mpu.setFullScaleAccelRange(AFS_SEL);
//
//      // get expected DMP packet size for later comparison
//      packetSize = mpu[i].dmpGetFIFOPacketSize();
//      fifoCount = mpu[i].getFIFOCount();
//    }else{
//      // ERROR!
//      // 1 = initial memory load failed
//      // 2 = DMP configuration updates failed
//      // (if it's going to break, usually the code will be 1)
//      Serial.print(F("DMP Initialization failed (code "));
//      Serial.print(devStatus);
//      Serial.println(F(")"));
//    }
//    Serial.println("---------------------------------------");
//  }
//  //Serial.println(CTRLA, BIN);
//  //Serial.println(PORT->Group[SERCOM].CTRLA.reg, BIN);
//  //Serial.println(SECOM1->SPI.CTRLA.reg, BIN);
//}

void loop() {  
  for(int i=0; i<num_imus; i++){
    
//    digitalWrite(P[activeIMU], HIGH);
//    digitalWrite(P[i], LOW);
    activeIMU = i;
    
    if (dmpReady[i]){
      if(printFlag){
        Serial.print("IMU ");
        Serial.println(i);
      }
      fifoCount = mpu[i].getFIFOCount();
      if (fifoCount == 1024){
        mpu[i].resetFIFO();
        if(printFlag) Serial.println(F("FIFO overflow!"));
      }else{
        if (fifoCount % packetSize != 0) {
          mpu[i].resetFIFO();
          if(printFlag) Serial.println(F("ERROR"));
        }else{
          while (fifoCount < packetSize) fifoCount = mpu[i].getFIFOCount();
            mpu[i].getFIFOBytes(fifoBuffer, packetSize);
  
            mpu[i].dmpGetAccel(accel, fifoBuffer);
            mpu[i].dmpGetGyro(gyro, fifoBuffer);
            mpu[i].dmpGetQuaternion(&q, fifoBuffer);
            mpu[i].dmpGetEuler(euler, &q);
            //mpu[i].dmpGetGravity(&gravity,&q);
            mpu[i].dmpGetYawPitchRoll(ypr,&q,&gravity); 

            sendData(accel, gyro, &q, &P[i]);
            
            if(printFlag){
              Serial.println("accel");
              Serial.print(accel[0]);
              Serial.print(":");
              Serial.print(accel[1]);
              Serial.print(":");
              Serial.println(accel[2]);
    
              Serial.println("gyro");
              Serial.print(gyro[0]);
              Serial.print(":");
              Serial.print(gyro[1]);
              Serial.print(":");
              Serial.println(gyro[2]);
    
              Serial.println("ypr");
              Serial.print(ypr[2]*RADIANS_TO_DEGREES, 2);
              Serial.print(":");
              Serial.print(-ypr[1]*RADIANS_TO_DEGREES, 2);
              Serial.print(":");
              Serial.println(ypr[0]*RADIANS_TO_DEGREES, 2);
    
              Serial.println("euler");
              Serial.print(euler[0], 2);
              Serial.print(":");
              Serial.print(euler[1], 2);
              Serial.print(":");
              Serial.println(euler[2], 2);
              
              Serial.println("quaternion");
              Serial.print(q.x, 2);
              Serial.print(":");
              Serial.print(q.y, 2);
              Serial.print(":");
              Serial.print(q.z, 2);
              Serial.print(":");
              Serial.println(q.w, 2);
            }
            
        }
      }
      if(printFlag) Serial.println("---------------------------------------");
    }
  }
}
