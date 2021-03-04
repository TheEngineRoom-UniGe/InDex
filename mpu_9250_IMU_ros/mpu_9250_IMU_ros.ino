
#include <ros.h>

/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

sensor_msgs::Imu imu_msg; 
ros::Publisher imu_pub("imu", &imu_msg);


char hello[13] = "hello world!";

void setup()
{
 // nh.getHardware()->setBaud(9600); did not work
  nh.initNode();
  nh.advertise(chatter);
    nh.advertise(imu_pub);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  delay(500);
  nh.spinOnce();
  delay(1000);

  geometry_msgs::Vector3 acc;
  acc.x = 1.0; acc.y = 2.0;acc.z=3.0;
  
  geometry_msgs::Vector3 gyr;
  gyr.x = 4.0; gyr.y = 5.0;gyr.z=6.0;

  geometry_msgs::Vector3 mag; 
  mag.x=6.0;mag.y=6.0;mag.z=8.0;
  geometry_msgs::Quaternion q;
  q.x=9.0;q.y=10.0;q.z=11.0;q.w=12.0;
  
  imu_msg.linear_acceleration = acc; 
  //imu_msg.mag =mag;
  imu_msg.orientation=q;
  imu_msg.angular_velocity = gyr; 
  chatter.publish( &imu_msg );
  delay(500);
  nh.spinOnce();
  

  
  
}
