#ifndef _ROS_logitech_wheel_msgs_LogitechWheelCmd_h
#define _ROS_logitech_wheel_msgs_LogitechWheelCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace logitech_wheel_msgs
{

  class LogitechWheelCmd : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _gear_name_type;
      _gear_name_type gear_name;
      typedef int16_t _steer_percent_type;
      _steer_percent_type steer_percent;
      typedef int16_t _gas_percent_type;
      _gas_percent_type gas_percent;
      typedef int16_t _brake_percent_type;
      _brake_percent_type brake_percent;
      typedef const char* _speed_mode_type;
      _speed_mode_type speed_mode;
      typedef const char* _steer_mode_type;
      _steer_mode_type steer_mode;
      typedef const char* _operation_mode_type;
      _operation_mode_type operation_mode;

    LogitechWheelCmd():
      header(),
      gear_name(""),
      steer_percent(0),
      gas_percent(0),
      brake_percent(0),
      speed_mode(""),
      steer_mode(""),
      operation_mode("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_gear_name = strlen(this->gear_name);
      varToArr(outbuffer + offset, length_gear_name);
      offset += 4;
      memcpy(outbuffer + offset, this->gear_name, length_gear_name);
      offset += length_gear_name;
      union {
        int16_t real;
        uint16_t base;
      } u_steer_percent;
      u_steer_percent.real = this->steer_percent;
      *(outbuffer + offset + 0) = (u_steer_percent.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steer_percent.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->steer_percent);
      union {
        int16_t real;
        uint16_t base;
      } u_gas_percent;
      u_gas_percent.real = this->gas_percent;
      *(outbuffer + offset + 0) = (u_gas_percent.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gas_percent.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->gas_percent);
      union {
        int16_t real;
        uint16_t base;
      } u_brake_percent;
      u_brake_percent.real = this->brake_percent;
      *(outbuffer + offset + 0) = (u_brake_percent.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_brake_percent.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->brake_percent);
      uint32_t length_speed_mode = strlen(this->speed_mode);
      varToArr(outbuffer + offset, length_speed_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->speed_mode, length_speed_mode);
      offset += length_speed_mode;
      uint32_t length_steer_mode = strlen(this->steer_mode);
      varToArr(outbuffer + offset, length_steer_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->steer_mode, length_steer_mode);
      offset += length_steer_mode;
      uint32_t length_operation_mode = strlen(this->operation_mode);
      varToArr(outbuffer + offset, length_operation_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->operation_mode, length_operation_mode);
      offset += length_operation_mode;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_gear_name;
      arrToVar(length_gear_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_gear_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_gear_name-1]=0;
      this->gear_name = (char *)(inbuffer + offset-1);
      offset += length_gear_name;
      union {
        int16_t real;
        uint16_t base;
      } u_steer_percent;
      u_steer_percent.base = 0;
      u_steer_percent.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steer_percent.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->steer_percent = u_steer_percent.real;
      offset += sizeof(this->steer_percent);
      union {
        int16_t real;
        uint16_t base;
      } u_gas_percent;
      u_gas_percent.base = 0;
      u_gas_percent.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gas_percent.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gas_percent = u_gas_percent.real;
      offset += sizeof(this->gas_percent);
      union {
        int16_t real;
        uint16_t base;
      } u_brake_percent;
      u_brake_percent.base = 0;
      u_brake_percent.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_brake_percent.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->brake_percent = u_brake_percent.real;
      offset += sizeof(this->brake_percent);
      uint32_t length_speed_mode;
      arrToVar(length_speed_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_speed_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_speed_mode-1]=0;
      this->speed_mode = (char *)(inbuffer + offset-1);
      offset += length_speed_mode;
      uint32_t length_steer_mode;
      arrToVar(length_steer_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_steer_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_steer_mode-1]=0;
      this->steer_mode = (char *)(inbuffer + offset-1);
      offset += length_steer_mode;
      uint32_t length_operation_mode;
      arrToVar(length_operation_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_operation_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_operation_mode-1]=0;
      this->operation_mode = (char *)(inbuffer + offset-1);
      offset += length_operation_mode;
     return offset;
    }

    virtual const char * getType() override { return "logitech_wheel_msgs/LogitechWheelCmd"; };
    virtual const char * getMD5() override { return "f54788124d0349a136c06204a4c383f6"; };

  };

}
#endif
