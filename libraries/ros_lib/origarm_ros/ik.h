#ifndef _ROS_SERVICE_ik_h
#define _ROS_SERVICE_ik_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/States.h"
#include "origarm_ros/modenumber.h"

namespace origarm_ros
{

static const char IK[] = "origarm_ros/ik";

  class ikRequest : public ros::Msg
  {
    public:
      typedef origarm_ros::States _input_type;
      _input_type input;
      typedef origarm_ros::modenumber _mode_type;
      _mode_type mode;

    ikRequest():
      input(),
      mode()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->input.serialize(outbuffer + offset);
      offset += this->mode.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->input.deserialize(inbuffer + offset);
      offset += this->mode.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return IK; };
    virtual const char * getMD5() override { return "a20cd281dad19abaf8a2d40e3eb66ac8"; };

  };

  class ikResponse : public ros::Msg
  {
    public:
      typedef origarm_ros::Command_ABL _output_type;
      _output_type output;

    ikResponse():
      output()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->output.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->output.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return IK; };
    virtual const char * getMD5() override { return "33a9a17e2cee799efabd88609880be0d"; };

  };

  class ik {
    public:
    typedef ikRequest Request;
    typedef ikResponse Response;
  };

}
#endif
