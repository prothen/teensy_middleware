#ifndef _ROS_platooning_msgs_PlatoonData_h
#define _ROS_platooning_msgs_PlatoonData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace platooning_msgs
{

  class PlatoonData : public ros::Msg
  {
    public:
      typedef const char* _platoon_id_type;
      _platoon_id_type platoon_id;
      typedef int8_t _num_vehicles_type;
      _num_vehicles_type num_vehicles;
      typedef const char* _leader_id_type;
      _leader_id_type leader_id;
      typedef float _leader_speed_type;
      _leader_speed_type leader_speed;
      uint32_t vehicle_ids_length;
      typedef char* _vehicle_ids_type;
      _vehicle_ids_type st_vehicle_ids;
      _vehicle_ids_type * vehicle_ids;
      uint32_t spacings_length;
      typedef float _spacings_type;
      _spacings_type st_spacings;
      _spacings_type * spacings;
      uint32_t follower_speeds_length;
      typedef float _follower_speeds_type;
      _follower_speeds_type st_follower_speeds;
      _follower_speeds_type * follower_speeds;

    PlatoonData():
      platoon_id(""),
      num_vehicles(0),
      leader_id(""),
      leader_speed(0),
      vehicle_ids_length(0), st_vehicle_ids(), vehicle_ids(nullptr),
      spacings_length(0), st_spacings(), spacings(nullptr),
      follower_speeds_length(0), st_follower_speeds(), follower_speeds(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_platoon_id = strlen(this->platoon_id);
      varToArr(outbuffer + offset, length_platoon_id);
      offset += 4;
      memcpy(outbuffer + offset, this->platoon_id, length_platoon_id);
      offset += length_platoon_id;
      union {
        int8_t real;
        uint8_t base;
      } u_num_vehicles;
      u_num_vehicles.real = this->num_vehicles;
      *(outbuffer + offset + 0) = (u_num_vehicles.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->num_vehicles);
      uint32_t length_leader_id = strlen(this->leader_id);
      varToArr(outbuffer + offset, length_leader_id);
      offset += 4;
      memcpy(outbuffer + offset, this->leader_id, length_leader_id);
      offset += length_leader_id;
      union {
        float real;
        uint32_t base;
      } u_leader_speed;
      u_leader_speed.real = this->leader_speed;
      *(outbuffer + offset + 0) = (u_leader_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leader_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_leader_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_leader_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->leader_speed);
      *(outbuffer + offset + 0) = (this->vehicle_ids_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vehicle_ids_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vehicle_ids_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vehicle_ids_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vehicle_ids_length);
      for( uint32_t i = 0; i < vehicle_ids_length; i++){
      uint32_t length_vehicle_idsi = strlen(this->vehicle_ids[i]);
      varToArr(outbuffer + offset, length_vehicle_idsi);
      offset += 4;
      memcpy(outbuffer + offset, this->vehicle_ids[i], length_vehicle_idsi);
      offset += length_vehicle_idsi;
      }
      *(outbuffer + offset + 0) = (this->spacings_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->spacings_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->spacings_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->spacings_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->spacings_length);
      for( uint32_t i = 0; i < spacings_length; i++){
      union {
        float real;
        uint32_t base;
      } u_spacingsi;
      u_spacingsi.real = this->spacings[i];
      *(outbuffer + offset + 0) = (u_spacingsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_spacingsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_spacingsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_spacingsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->spacings[i]);
      }
      *(outbuffer + offset + 0) = (this->follower_speeds_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->follower_speeds_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->follower_speeds_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->follower_speeds_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->follower_speeds_length);
      for( uint32_t i = 0; i < follower_speeds_length; i++){
      union {
        float real;
        uint32_t base;
      } u_follower_speedsi;
      u_follower_speedsi.real = this->follower_speeds[i];
      *(outbuffer + offset + 0) = (u_follower_speedsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_follower_speedsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_follower_speedsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_follower_speedsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->follower_speeds[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_platoon_id;
      arrToVar(length_platoon_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_platoon_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_platoon_id-1]=0;
      this->platoon_id = (char *)(inbuffer + offset-1);
      offset += length_platoon_id;
      union {
        int8_t real;
        uint8_t base;
      } u_num_vehicles;
      u_num_vehicles.base = 0;
      u_num_vehicles.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->num_vehicles = u_num_vehicles.real;
      offset += sizeof(this->num_vehicles);
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
        float real;
        uint32_t base;
      } u_leader_speed;
      u_leader_speed.base = 0;
      u_leader_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leader_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_leader_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_leader_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->leader_speed = u_leader_speed.real;
      offset += sizeof(this->leader_speed);
      uint32_t vehicle_ids_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      vehicle_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      vehicle_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      vehicle_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->vehicle_ids_length);
      if(vehicle_ids_lengthT > vehicle_ids_length)
        this->vehicle_ids = (char**)realloc(this->vehicle_ids, vehicle_ids_lengthT * sizeof(char*));
      vehicle_ids_length = vehicle_ids_lengthT;
      for( uint32_t i = 0; i < vehicle_ids_length; i++){
      uint32_t length_st_vehicle_ids;
      arrToVar(length_st_vehicle_ids, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_vehicle_ids; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_vehicle_ids-1]=0;
      this->st_vehicle_ids = (char *)(inbuffer + offset-1);
      offset += length_st_vehicle_ids;
        memcpy( &(this->vehicle_ids[i]), &(this->st_vehicle_ids), sizeof(char*));
      }
      uint32_t spacings_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      spacings_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      spacings_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      spacings_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->spacings_length);
      if(spacings_lengthT > spacings_length)
        this->spacings = (float*)realloc(this->spacings, spacings_lengthT * sizeof(float));
      spacings_length = spacings_lengthT;
      for( uint32_t i = 0; i < spacings_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_spacings;
      u_st_spacings.base = 0;
      u_st_spacings.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_spacings.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_spacings.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_spacings.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_spacings = u_st_spacings.real;
      offset += sizeof(this->st_spacings);
        memcpy( &(this->spacings[i]), &(this->st_spacings), sizeof(float));
      }
      uint32_t follower_speeds_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      follower_speeds_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      follower_speeds_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      follower_speeds_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->follower_speeds_length);
      if(follower_speeds_lengthT > follower_speeds_length)
        this->follower_speeds = (float*)realloc(this->follower_speeds, follower_speeds_lengthT * sizeof(float));
      follower_speeds_length = follower_speeds_lengthT;
      for( uint32_t i = 0; i < follower_speeds_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_follower_speeds;
      u_st_follower_speeds.base = 0;
      u_st_follower_speeds.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_follower_speeds.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_follower_speeds.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_follower_speeds.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_follower_speeds = u_st_follower_speeds.real;
      offset += sizeof(this->st_follower_speeds);
        memcpy( &(this->follower_speeds[i]), &(this->st_follower_speeds), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "platooning_msgs/PlatoonData"; };
    virtual const char * getMD5() override { return "25cdbd2128b2ec0735cc2d54ea2e6109"; };

  };

}
#endif
