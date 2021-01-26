#ifndef _ROS_origarm_ros_Sensor_Act_h
#define _ROS_origarm_ros_Sensor_Act_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace origarm_ros
{

  class Sensor_Act : public ros::Msg
  {
    public:
      typedef int16_t _pressure_type;
      _pressure_type pressure;
      typedef uint16_t _distance_type;
      _distance_type distance;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef int16_t _imu_active_type;
      _imu_active_type imu_active;

    Sensor_Act():
      pressure(0),
      distance(0),
      pose(),
      imu_active(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_pressure;
      u_pressure.real = this->pressure;
      *(outbuffer + offset + 0) = (u_pressure.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pressure.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pressure);
      *(outbuffer + offset + 0) = (this->distance >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->distance >> (8 * 1)) & 0xFF;
      offset += sizeof(this->distance);
      offset += this->pose.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_imu_active;
      u_imu_active.real = this->imu_active;
      *(outbuffer + offset + 0) = (u_imu_active.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_imu_active.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->imu_active);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_pressure;
      u_pressure.base = 0;
      u_pressure.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pressure.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pressure = u_pressure.real;
      offset += sizeof(this->pressure);
      this->distance =  ((uint16_t) (*(inbuffer + offset)));
      this->distance |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->distance);
      offset += this->pose.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_imu_active;
      u_imu_active.base = 0;
      u_imu_active.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_imu_active.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->imu_active = u_imu_active.real;
      offset += sizeof(this->imu_active);
     return offset;
    }

    virtual const char * getType() override { return "origarm_ros/Sensor_Act"; };
    virtual const char * getMD5() override { return "da0bd911c8c3e1b4f2d8753b34a1a337"; };

  };

}
#endif
