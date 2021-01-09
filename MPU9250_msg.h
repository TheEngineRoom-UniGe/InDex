#ifndef _ROS_sensor_msgs_MPU9250_h
#define _ROS_sensor_msgs_MPU9250_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ros/msg.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace sensor_msgs
{

  class MPU9250_msg: public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Quaternion _orientation_type;
      _orientation_type orientation;
      typedef geometry_msgs::Vector3 _gyro_type;
      _gyro_type gyr;
     
      typedef geometry_msgs::Vector3 _magnetometer_type;
      _magnetometer_type mag;
      typedef geometry_msgs::Vector3 _accelerometer_type;
      _accelerometer_type acc;
      
     
  //    typedef geometry_msgs::Vector3 _angular_velocity_type;
    //  _angular_velocity_type angular_velocity;

     // typedef geometry_msgs::Vector3 _linear_acceleration_type;
      //_linear_acceleration_type linear_acceleration;
	
	//float linear_acceleration_covariance[9];
	//float orientation_covariance[9];
	//float angular_velocity_covariance[9];
    MPU9250_msg():
      header(),
      orientation(),
      gyr(),
      mag(),
      acc()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      offset += this->gyr.serialize(outbuffer + offset);
      offset += this->mag.serialize(outbuffer + offset);
      offset += this->acc.serialize(outbuffer + offset);
    
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
      offset += this->gyr.deserialize(inbuffer + offset);
      offset += this->mag.deserialize(inbuffer + offset);
      offset += this->acc.deserialize(inbuffer + offset);
      return offset;
    }

    virtual const char * getType() override { return "sensor_msgs/MPU9250_msg.h"; };
    virtual const char * getMD5() override { return "6a62c6daae103f4ff57a132d6f95cecc3"; };

  };

}
#endif