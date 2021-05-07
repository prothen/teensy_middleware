#ifndef _ROS_platooning_msgs_VehicleData_h
#define _ROS_platooning_msgs_VehicleData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"

namespace platooning_msgs
{

  class VehicleData : public ros::Msg
  {
    public:
      typedef const char* _vehicle_id_type;
      _vehicle_id_type vehicle_id;
      typedef nav_msgs::Path _path_type;
      _path_type path;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef float _speed_type;
      _speed_type speed;
      typedef bool _com_status_type;
      _com_status_type com_status;
      typedef int8_t _relation_type;
      _relation_type relation;
      typedef const char* _platoon_id_type;
      _platoon_id_type platoon_id;
      typedef const char* _leader_id_type;
      _leader_id_type leader_id;
      typedef int8_t _order_in_platoon_type;
      _order_in_platoon_type order_in_platoon;

    VehicleData():
      vehicle_id(""),
      path(),
      pose(),
      speed(0),
      com_status(0),
      relation(0),
      platoon_id(""),
      leader_id(""),
      order_in_platoon(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_vehicle_id = strlen(this->vehicle_id);
      varToArr(outbuffer + offset, length_vehicle_id);
      offset += 4;
      memcpy(outbuffer + offset, this->vehicle_id, length_vehicle_id);
      offset += length_vehicle_id;
      offset += this->path.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
      union {
        bool real;
        uint8_t base;
      } u_com_status;
      u_com_status.real = this->com_status;
      *(outbuffer + offset + 0) = (u_com_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->com_status);
      union {
        int8_t real;
        uint8_t base;
      } u_relation;
      u_relation.real = this->relation;
      *(outbuffer + offset + 0) = (u_relation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relation);
      uint32_t length_platoon_id = strlen(this->platoon_id);
      varToArr(outbuffer + offset, length_platoon_id);
      offset += 4;
      memcpy(outbuffer + offset, this->platoon_id, length_platoon_id);
      offset += length_platoon_id;
      uint32_t length_leader_id = strlen(this->leader_id);
      varToArr(outbuffer + offset, length_leader_id);
      offset += 4;
      memcpy(outbuffer + offset, this->leader_id, length_leader_id);
      offset += length_leader_id;
      union {
        int8_t real;
        uint8_t base;
      } u_order_in_platoon;
      u_order_in_platoon.real = this->order_in_platoon;
      *(outbuffer + offset + 0) = (u_order_in_platoon.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->order_in_platoon);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_vehicle_id;
      arrToVar(length_vehicle_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_vehicle_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_vehicle_id-1]=0;
      this->vehicle_id = (char *)(inbuffer + offset-1);
      offset += length_vehicle_id;
      offset += this->path.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      union {
        bool real;
        uint8_t base;
      } u_com_status;
      u_com_status.base = 0;
      u_com_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->com_status = u_com_status.real;
      offset += sizeof(this->com_status);
      union {
        int8_t real;
        uint8_t base;
      } u_relation;
      u_relation.base = 0;
      u_relation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relation = u_relation.real;
      offset += sizeof(this->relation);
      uint32_t length_platoon_id;
      arrToVar(length_platoon_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_platoon_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_platoon_id-1]=0;
      this->platoon_id = (char *)(inbuffer + offset-1);
      offset += length_platoon_id;
      uint32_t length_leader_id;
      arrToVar(length_leader_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_leader_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_leader_id-1]=0;
      this->leader_id = (char *)(inbuffer + offset-1);
      offset += length_leader_id;
      union {
        int8_t real;
        uint8_t base;
      } u_order_in_platoon;
      u_order_in_platoon.base = 0;
      u_order_in_platoon.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->order_in_platoon = u_order_in_platoon.real;
      offset += sizeof(this->order_in_platoon);
     return offset;
    }

    virtual const char * getType() override { return "platooning_msgs/VehicleData"; };
    virtual const char * getMD5() override { return "6942b5482a2abda63e0295e51def8d87"; };

  };

}
#endif
