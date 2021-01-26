#ifndef _ROS_origarm_ros_SegOpening_h
#define _ROS_origarm_ros_SegOpening_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace origarm_ros
{

  class SegOpening : public ros::Msg
  {
    public:
      float Op[6];

    SegOpening():
      Op()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_Opi;
      u_Opi.real = this->Op[i];
      *(outbuffer + offset + 0) = (u_Opi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Opi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Opi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Opi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Op[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_Opi;
      u_Opi.base = 0;
      u_Opi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Opi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Opi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Opi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Op[i] = u_Opi.real;
      offset += sizeof(this->Op[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "origarm_ros/SegOpening"; };
    virtual const char * getMD5() override { return "eee85da1d9f3b19461987465c0a06089"; };

  };

}
#endif
