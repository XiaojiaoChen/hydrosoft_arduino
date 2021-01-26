#ifndef _ROS_origarm_ros_States_h
#define _ROS_origarm_ros_States_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "origarm_ros/Command_ABL.h"
#include "geometry_msgs/Pose.h"

namespace origarm_ros
{

  class States : public ros::Msg
  {
    public:
      typedef origarm_ros::Command_ABL _ABL_type;
      _ABL_type ABL;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;

    States():
      ABL(),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->ABL.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->ABL.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "origarm_ros/States"; };
    virtual const char * getMD5() override { return "420de3d9f6e97ce7d1eec1ebdd32bcbf"; };

  };

}
#endif
