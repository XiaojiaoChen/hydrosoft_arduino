#ifndef _ROS_origarm_ros_Sensor_h
#define _ROS_origarm_ros_Sensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "origarm_ros/Sensor_Seg.h"

namespace origarm_ros
{

  class Sensor : public ros::Msg
  {
    public:
      origarm_ros::Sensor_Seg sensor_segment[9];

    Sensor():
      sensor_segment()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 9; i++){
      offset += this->sensor_segment[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 9; i++){
      offset += this->sensor_segment[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    virtual const char * getType() override { return "origarm_ros/Sensor"; };
    virtual const char * getMD5() override { return "f371dced176dd204cd574eb13d8c8cd1"; };

  };

}
#endif
