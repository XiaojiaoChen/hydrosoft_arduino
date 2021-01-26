#ifndef _ROS_hydrosoft_ros_Command_Position_Arm_h
#define _ROS_hydrosoft_ros_Command_Position_Arm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace hydrosoft_ros
{

  class Command_Position_Arm : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _pos_type;
      _pos_type pos;

    Command_Position_Arm():
      pos()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pos.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pos.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "hydrosoft_ros/Command_Position_Arm"; };
    virtual const char * getMD5() override { return "7e0f2f2813619d82be0a36f03197cf02"; };

  };

}
#endif
