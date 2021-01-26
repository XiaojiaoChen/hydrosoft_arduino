#ifndef _ROS_hydrosoft_ros_States_h
#define _ROS_hydrosoft_ros_States_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "hydrosoft_ros/Command_ABL_Arm.h"
#include "geometry_msgs/Pose.h"

namespace hydrosoft_ros
{

  class States : public ros::Msg
  {
    public:
      typedef hydrosoft_ros::Command_ABL_Arm _abl_arm_type;
      _abl_arm_type abl_arm;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;

    States():
      abl_arm(),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->abl_arm.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->abl_arm.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "hydrosoft_ros/States"; };
    virtual const char * getMD5() override { return "9dce09662d32a9875803baf455eebceb"; };

  };

}
#endif
