#ifndef _ROS_origarm_ros_Command_Pre_Open_h
#define _ROS_origarm_ros_Command_Pre_Open_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "origarm_ros/Seg_Pre.h"

namespace origarm_ros
{

  class Command_Pre_Open : public ros::Msg
  {
    public:
      origarm_ros::Seg_Pre segment[9];

    Command_Pre_Open():
      segment()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 9; i++){
      offset += this->segment[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 9; i++){
      offset += this->segment[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    virtual const char * getType() override { return "origarm_ros/Command_Pre_Open"; };
    virtual const char * getMD5() override { return "f7b1e00db6e6598489850523304ac062"; };

  };

}
#endif
