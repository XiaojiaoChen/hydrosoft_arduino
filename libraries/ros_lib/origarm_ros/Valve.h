#ifndef _ROS_origarm_ros_Valve_h
#define _ROS_origarm_ros_Valve_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace origarm_ros
{

  class Valve : public ros::Msg
  {
    public:
      typedef int16_t _pressure_type;
      _pressure_type pressure;
      typedef bool _valve_type;
      _valve_type valve;

    Valve():
      pressure(0),
      valve(0)
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
      union {
        bool real;
        uint8_t base;
      } u_valve;
      u_valve.real = this->valve;
      *(outbuffer + offset + 0) = (u_valve.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->valve);
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
      union {
        bool real;
        uint8_t base;
      } u_valve;
      u_valve.base = 0;
      u_valve.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->valve = u_valve.real;
      offset += sizeof(this->valve);
     return offset;
    }

    virtual const char * getType() override { return "origarm_ros/Valve"; };
    virtual const char * getMD5() override { return "ace3a28d9ca94f292eba61208a7b47c8"; };

  };

}
#endif
