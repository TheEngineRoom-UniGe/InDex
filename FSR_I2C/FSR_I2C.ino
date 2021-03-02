
#include <Wire.h>

#define ADDRESS 0x4D // 7 bits address is 0x4D, 8 bits is 0x9B
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
void setup(){
 Wire.begin(); //conects I2C
 tcaselect(6);
 Serial.begin(115200);
}

void loop(){
 byte ad_high;
 byte ad_low;
 int Result;
 
  Wire.requestFrom(ADDRESS, 2);        //requests 2 bytes
  while(Wire.available() < 2);         //while two bytes to receive
   
  ad_high = Wire.read();          
  ad_low = Wire.read();
  Result = (ad_high * 256) + ad_low;
 
  Serial.println(Result);
  Serial.print (" high");Serial.print(ad_high) ;Serial.print(" low ");Serial.println( ad_low);
 
  delay(10);
}
