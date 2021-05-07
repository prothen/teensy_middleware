#ifndef _ROS_marker_msgs_MarkerWithCovariance_h
#define _ROS_marker_msgs_MarkerWithCovariance_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "marker_msgs/Marker.h"

namespace marker_msgs
{

  class MarkerWithCovariance : public ros::Msg
  {
    public:
      typedef marker_msgs::Marker _marker_type;
      _marker_type marker;
      float covariance[36];

    MarkerWithCovariance():
      marker(),
      covariance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->marker.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 36; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->marker.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 36; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->covariance[i]));
      }
     return offset;
    }

    virtual const char * getType() override { return "marker_msgs/MarkerWithCovariance"; };
    virtual const char * getMD5() override { return "ace241bc4ec8f4b399c13e05763be31a"; };

  };

}
#endif
