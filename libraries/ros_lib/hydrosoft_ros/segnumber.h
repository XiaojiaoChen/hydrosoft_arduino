#ifndef _ROS_hydrosoft_ros_segnumber_h
#define _ROS_hydrosoft_ros_segnumber_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hydrosoft_ros
{

  class segnumber : public ros::Msg
  {
    public:
      typedef int16_t _segmentNumber_type;
      _segmentNumber_type segmentNumber;

    segnumber():
      segmentNumber(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_segmentNumber;
      u_segmentNumber.real = this->segmentNumber;
      *(outbuffer + offset + 0) = (u_segmentNumber.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_segmentNumber.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->segmentNumber);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_segmentNumber;
      u_segmentNumber.base = 0;
      u_segmentNumber.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_segmentNumber.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->segmentNumber = u_segmentNumber.real;
      offset += sizeof(this->segmentNumber);
     return offset;
    }

    virtual const char * getType() override { return "hydrosoft_ros/segnumber"; };
    virtual const char * getMD5() override { return "8372133d19d136441c6a702d77321942"; };

  };

}
#endif
