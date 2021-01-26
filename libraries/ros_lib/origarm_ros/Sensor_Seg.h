#ifndef _ROS_origarm_ros_Sensor_Seg_h
#define _ROS_origarm_ros_Sensor_Seg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "origarm_ros/Sensor_Act.h"

namespace origarm_ros
{

  class Sensor_Seg : public ros::Msg
  {
    public:
      origarm_ros::Sensor_Act sensor_actuator[6];

    Sensor_Seg():
      sensor_actuator()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += this->sensor_actuator[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += this->sensor_actuator[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    virtual const char * getType() override { return "origarm_ros/Sensor_Seg"; };
    virtual const char * getMD5() override { return "9f5172fb0cc821570fc273a5a77cce34"; };

  };

}
#endif
