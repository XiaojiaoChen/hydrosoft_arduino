#ifndef _ROS_hydrosoft_ros_keynumber_h
#define _ROS_hydrosoft_ros_keynumber_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hydrosoft_ros
{

  class keynumber : public ros::Msg
  {
    public:
      typedef uint16_t _keycodePressed_type;
      _keycodePressed_type keycodePressed;

    keynumber():
      keycodePressed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->keycodePressed >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->keycodePressed >> (8 * 1)) & 0xFF;
      offset += sizeof(this->keycodePressed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->keycodePressed =  ((uint16_t) (*(inbuffer + offset)));
      this->keycodePressed |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->keycodePressed);
     return offset;
    }

    virtual const char * getType() override { return "hydrosoft_ros/keynumber"; };
    virtual const char * getMD5() override { return "468ad1b93eea1baccb9954b367efbccf"; };

  };

}
#endif
