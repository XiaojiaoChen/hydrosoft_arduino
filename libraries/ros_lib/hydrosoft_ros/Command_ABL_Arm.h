#ifndef _ROS_hydrosoft_ros_Command_ABL_Arm_h
#define _ROS_hydrosoft_ros_Command_ABL_Arm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "hydrosoft_ros/Command_ABL_Seg.h"

namespace hydrosoft_ros
{

  class Command_ABL_Arm : public ros::Msg
  {
    public:
      hydrosoft_ros::Command_ABL_Seg segment[4];

    Command_ABL_Arm():
      segment()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      offset += this->segment[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      offset += this->segment[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    virtual const char * getType() override { return "hydrosoft_ros/Command_ABL_Arm"; };
    virtual const char * getMD5() override { return "78385bb25f040e30c47d0755743d0d44"; };

  };

}
#endif
