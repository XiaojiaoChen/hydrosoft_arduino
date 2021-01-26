#ifndef _ROS_hydrosoft_ros_Command_ABL_Seg_h
#define _ROS_hydrosoft_ros_Command_ABL_Seg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hydrosoft_ros
{

  class Command_ABL_Seg : public ros::Msg
  {
    public:
      typedef float _A_type;
      _A_type A;
      typedef float _B_type;
      _B_type B;
      typedef float _L_type;
      _L_type L;

    Command_ABL_Seg():
      A(0),
      B(0),
      L(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_A;
      u_A.real = this->A;
      *(outbuffer + offset + 0) = (u_A.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_A.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_A.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_A.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->A);
      union {
        float real;
        uint32_t base;
      } u_B;
      u_B.real = this->B;
      *(outbuffer + offset + 0) = (u_B.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_B.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_B.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_B.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->B);
      union {
        float real;
        uint32_t base;
      } u_L;
      u_L.real = this->L;
      *(outbuffer + offset + 0) = (u_L.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_L.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_L.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_L.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->L);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_A;
      u_A.base = 0;
      u_A.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_A.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_A.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_A.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->A = u_A.real;
      offset += sizeof(this->A);
      union {
        float real;
        uint32_t base;
      } u_B;
      u_B.base = 0;
      u_B.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_B.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_B.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_B.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->B = u_B.real;
      offset += sizeof(this->B);
      union {
        float real;
        uint32_t base;
      } u_L;
      u_L.base = 0;
      u_L.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_L.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_L.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_L.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->L = u_L.real;
      offset += sizeof(this->L);
     return offset;
    }

    virtual const char * getType() override { return "hydrosoft_ros/Command_ABL_Seg"; };
    virtual const char * getMD5() override { return "f594ea0aeca14d238a02bd75aadb0092"; };

  };

}
#endif
