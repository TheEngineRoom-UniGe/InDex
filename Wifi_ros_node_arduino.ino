#include <WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
// In order to make this code works, a ROS python script is executed at the ROS server side
//////////////////////
// WiFi Definitions //
//////////////////////
const char* ssid = "yournetworkID";
const char* password = "yourNetworkPass";

IPAddress server(130, 251, 13, 113); // ip of your ROS server
IPAddress ip;  //Storage local IP address
int status = WL_IDLE_STATUS;
char a = 51;

WiFiClient client;

class WiFiHardware { // this is done in order to make the redirect the ROS node connection to the TCP socket

  public:
  WiFiHardware() {};

  void init() {
    // do your initialization here. this probably includes TCP server/client setup
    client.connect(server, 11411);
  }

  // read a byte from the serial port. -1 = failure
  int read() {
    // implement this method so that it reads a byte from the TCP connection and returns it
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

char hello[13] = "Hello World2";


void setupWiFi()
{
  WiFi.begin(ssid, password);
  //Print to serial to find out IP address and debugging
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  ip = WiFi.localIP();
  Serial.print(ip);
  Serial.println(" to access client");

}

void setup() {

  Serial.begin(9600);
  setupWiFi();
  delay(2000);
  nh.initNode();
  nh.advertise(string);
}

void loop() {

  
  hello[12]=a;
  a++; // this was added just to make sure that we are receiving different messages at each time on ROS side
  msg.data = hello;
  string.publish(&msg);
  nh.spinOnce();
  delay(1000);


}
