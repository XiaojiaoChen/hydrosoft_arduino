#ifndef _ROS_origarm_ros_modenumber_h
#define _ROS_origarm_ros_modenumber_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace origarm_ros
{

  class modenumber : public ros::Msg
  {
    public:
      typedef int16_t _modeNumber_type;
      _modeNumber_type modeNumber;
      typedef int16_t _status_type;
      _status_type status;

    modenumber():
      modeNumber(0),
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_modeNumber;
      u_modeNumber.real = this->modeNumber;
      *(outbuffer + offset + 0) = (u_modeNumber.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_modeNumber.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->modeNumber);
      union {
        int16_t real;
        uint16_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_modeNumber;
      u_modeNumber.base = 0;
      u_modeNumber.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_modeNumber.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->modeNumber = u_modeNumber.real;
      offset += sizeof(this->modeNumber);
      union {
        int16_t real;
        uint16_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    virtual const char * getType() override { return "origarm_ros/modenumber"; };
    virtual const char * getMD5() override { return "33f4f9bd509904443f1c21e7bded9488"; };

  };

}
#endif
