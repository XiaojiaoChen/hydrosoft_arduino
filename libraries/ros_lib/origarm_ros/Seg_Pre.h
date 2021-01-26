#ifndef _ROS_origarm_ros_Seg_Pre_h
#define _ROS_origarm_ros_Seg_Pre_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "origarm_ros/Valve.h"

namespace origarm_ros
{

  class Seg_Pre : public ros::Msg
  {
    public:
      origarm_ros::Valve command[6];

    Seg_Pre():
      command()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += this->command[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += this->command[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    virtual const char * getType() override { return "origarm_ros/Seg_Pre"; };
    virtual const char * getMD5() override { return "dcfa8f034f96384a0cbadd0dcc436399"; };

  };

}
#endif
