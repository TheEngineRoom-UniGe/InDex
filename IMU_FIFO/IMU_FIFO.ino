#include <MPU9250_FIFO.h>
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
MPU9250_FIFO mpu(0),mpu2(1), mpu3(2);
//MPU9250 mpu2; 
//MPU9250 mpu1, mpu2, mpu3,mpu4; // mpu2 daisychained with mpu  , mpu4 daisychained with mpu3
//MPU9250 mpu5;
//MPU9250 mpu6;
//MPU9250 mpu7;
//MPU9250 mpu8;
//MPU9250 mpu9;
//MPU9250 mpu10;
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


void setup()
{   EEPROM.begin(512); // otherwise it wont write
    Serial.begin(115200);  
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
    Wire.begin();
    Wire.setClock(400000);

    delay(1000);
    tcaselect(6);Serial.println("scanning channel 6"); // 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
    i2cTest();
   tcaselect(5);Serial.println("scanning channel 5"); // 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
    i2cTest();
//     tcaselect(4);Serial.println("scanning channel 4"); // 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
//    i2cTest();
//     tcaselect(3); Serial.println("scanning channel 3");// 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
//    i2cTest();
//    tcaselect(2);Serial.println("scanning channel 2"); // 7 thumb //2 local / arm // 6 index ,5 middle, 4 ring, 3 pinky
//    i2cTest();
    
    delay(1000);
    tcaselect(6);
    mpu.setup();
    mpu2.setI2CAddress(0x69);
    mpu2.setup();
   
    tcaselect(5);
    mpu3.setup();
  
    Serial.println(" Send C to calibrate magnetometer1 and D for magnetometer 2, E for 3, L for lazy calibration "); 
    delay (3000); 
    

//    mpu4.setI2CAddress(0x69); 
//    mpu4.setup();
////  
//    tcaselect(4);
//      
//    mpu5.setup(); 
//    mpu6.setI2CAddress(0x69); 
//    mpu6.setup();
//    
//    tcaselect(3);
//    mpu7.setup(); 
//    mpu8.setI2CAddress(0x69); 
//    mpu8.setup();
//    
//     tcaselect(2); // when we read from channel 2 , an error occured and all other readings fail
//  mpu9.setI2CAddress(0x69); 
//
//    mpu9.setup(); 
//    
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
//   // mpu1 = mpu; 
//    //mpu.printCalibration
    
   if (mpu3.isCalibrated())
    mpu3.loadCalibration(); // currently it loads for instance called mpu.
    mpu3.printCalibrationfromEEPROM();
    mpu3.printCalibration();
 
    delay(500);

}

void loop()
{
    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 20)
//    {  tcaselect(6);
//      delay(200);
//       mpu1.update();
//      // mpu1.print();
//       // mpu1.printRawData(); 
//        //mpu1.printRollPitchYaw();
//        Serial.print("MPU1"); 
//        Serial.print("roll  (x-forward (north)) : ");
//        Serial.print(mpu1.getRoll());
//        Serial.print(" pitch (y-right (east))    : ");
//        Serial.print(mpu1.getPitch());
//        Serial.print(" yaw   (z-down (down))     : ");
//        Serial.println(mpu1.getYaw());
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
        tcaselect(6);
       // delay(5); // with 10, 20 ms , works fine , always updating ,visualised using teapot // 10 ms seems more stable (26fps with teapot)
        mpu.update();
//        Serial.print((int)1000 * mpu.getAcc(0));Serial.print("a\t");
//        Serial.print((int)1000 * mpu.getAcc(1));Serial.print("a\t");
//        Serial.print((int)1000 * mpu.getAcc(2));Serial.print("a\t");
//        // Print gyro values in degree/sec
//         Serial.print(mpu.getGyro(0));Serial.print("g\t");
//         Serial.print(mpu.getGyro(1));Serial.print("g\t");
//         Serial.print(mpu.getGyro(2));Serial.print("g\t");
////
//        Serial.print((int)mpu.getMag(0));Serial.print("m\t");
//        Serial.print((int)mpu.getMag(1));Serial.print("m\t");
//        Serial.print((int)mpu.getMag(2));Serial.print("m\n");
   //     mpu1.printRawData();
      //Serial.print(mpu1.getTemperature());
      // the below code works with the PYteapot .
      // to read the temperature , we have set the AHRS to false
     // Serial.print("t1");Serial.print(mpu1.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu.getQuaternion(3));Serial.println("c");
////      Serial.print("roll  (x-forward (north)) : ");
//      Serial.print(mpu1.getRoll());
//      Serial.print(" pitch (y-right (east))    : ");
//      Serial.print(mpu1.getPitch());
//      Serial.print(" yaw   (z-down (down))     : ");
//      Serial.println(mpu1.getYaw());


      mpu2.update();
//      Serial.print("2");Serial.print("m\t");
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
      Serial.print("w");Serial.print(mpu2.getQuaternion(0));Serial.print("w");
      Serial.print("a");Serial.print(mpu2.getQuaternion(1));Serial.print("a");
      Serial.print("b");Serial.print(mpu2.getQuaternion(2));Serial.print("b");
      Serial.print("c");Serial.print(mpu2.getQuaternion(3));Serial.println("c");
////     
//        Serial.print("m\t"); 
//        Serial.print((int)mpu2.getMag(0));Serial.print("m\t");
//        Serial.print((int)mpu2.getMag(1));Serial.print("m\t");
//        Serial.print((int)mpu2.getMag(2));Serial.print("m\n");
      
      tcaselect(5);

 //     mpu3.update();


// /           Serial.print("3");
//
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
   //   Serial.print("w");Serial.print(mpu3.getQuaternion(0));Serial.print("w");
    //  Serial.print("a");Serial.print(mpu3.getQuaternion(1));Serial.print("a");
     // Serial.print("b");Serial.print(mpu3.getQuaternion(2));Serial.print("b");
     // Serial.print("c");Serial.print(mpu3.getQuaternion(3));Serial.println("c");
//      /

      
   //   mpu4.update();
//            Serial.print("4");
//
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu4.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu4.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu4.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu4.getQuaternion(3));Serial.println("c");

     tcaselect(4);

 //    mpu5.update();
//     Serial.print("5");
//
//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu5.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu5.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu5.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu5.getQuaternion(3));Serial.println("c");

  //    mpu6.update();

//      Serial.print("6");
//




      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu6.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu6.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu6.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu6.getQuaternion(3));Serial.println("c");

       tcaselect(3);
  //    mpu7.update();
//  /    Serial.print("7");
      
      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");

//      Serial.print("w");Serial.print(mpu7.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu7.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu7.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu7.getQuaternion(3));Serial.println("c");
//
  //    mpu8.update();

//  /    Serial.print("8");

//      //Serial.print("t2");Serial.print(mpu2.getTemperature());Serial.print("t");
//      Serial.print("w");Serial.print(mpu8.getQuaternion(0));Serial.print("w");
//      Serial.print("a");Serial.print(mpu8.getQuaternion(1));Serial.print("a");
//      Serial.print("b");Serial.print(mpu8.getQuaternion(2));Serial.print("b");
//      Serial.print("c");Serial.print(mpu8.getQuaternion(3));Serial.println("c");
      tcaselect(2);
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
    }
}
