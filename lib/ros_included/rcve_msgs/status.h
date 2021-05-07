#ifndef _ROS_rcve_msgs_status_h
#define _ROS_rcve_msgs_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rcve_msgs
{

  class status : public ros::Msg
  {
    public:
      typedef const char* _shift_type;
      _shift_type shift;
      typedef bool _motor_error_type;
      _motor_error_type motor_error;
      typedef bool _motor_operational_type;
      _motor_operational_type motor_operational;
      typedef bool _steering_operational_type;
      _steering_operational_type steering_operational;
      typedef bool _remote_takeover_type;
      _remote_takeover_type remote_takeover;

    status():
      shift(""),
      motor_error(0),
      motor_operational(0),
      steering_operational(0),
      remote_takeover(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_shift = strlen(this->shift);
      varToArr(outbuffer + offset, length_shift);
      offset += 4;
      memcpy(outbuffer + offset, this->shift, length_shift);
      offset += length_shift;
      union {
        bool real;
        uint8_t base;
      } u_motor_error;
      u_motor_error.real = this->motor_error;
      *(outbuffer + offset + 0) = (u_motor_error.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motor_error);
      union {
        bool real;
        uint8_t base;
      } u_motor_operational;
      u_motor_operational.real = this->motor_operational;
      *(outbuffer + offset + 0) = (u_motor_operational.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motor_operational);
      union {
        bool real;
        uint8_t base;
      } u_steering_operational;
      u_steering_operational.real = this->steering_operational;
      *(outbuffer + offset + 0) = (u_steering_operational.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->steering_operational);
      union {
        bool real;
        uint8_t base;
      } u_remote_takeover;
      u_remote_takeover.real = this->remote_takeover;
      *(outbuffer + offset + 0) = (u_remote_takeover.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->remote_takeover);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_shift;
      arrToVar(length_shift, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_shift; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_shift-1]=0;
      this->shift = (char *)(inbuffer + offset-1);
      offset += length_shift;
      union {
        bool real;
        uint8_t base;
      } u_motor_error;
      u_motor_error.base = 0;
      u_motor_error.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motor_error = u_motor_error.real;
      offset += sizeof(this->motor_error);
      union {
        bool real;
        uint8_t base;
      } u_motor_operational;
      u_motor_operational.base = 0;
      u_motor_operational.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motor_operational = u_motor_operational.real;
      offset += sizeof(this->motor_operational);
      union {
        bool real;
        uint8_t base;
      } u_steering_operational;
      u_steering_operational.base = 0;
      u_steering_operational.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->steering_operational = u_steering_operational.real;
      offset += sizeof(this->steering_operational);
      union {
        bool real;
        uint8_t base;
      } u_remote_takeover;
      u_remote_takeover.base = 0;
      u_remote_takeover.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->remote_takeover = u_remote_takeover.real;
      offset += sizeof(this->remote_takeover);
     return offset;
    }

    virtual const char * getType() override { return "rcve_msgs/status"; };
    virtual const char * getMD5() override { return "35197f2f753738222150945f6616e519"; };

  };

}
#endif
