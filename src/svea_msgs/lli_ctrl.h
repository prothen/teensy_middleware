#ifndef _ROS_svea_msgs_lli_ctrl_h
#define _ROS_svea_msgs_lli_ctrl_h

#include <ros/msg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

namespace svea_msgs {

class lli_ctrl : public ros::Msg {
public:
    typedef int8_t _steering_type;
    _steering_type steering;
    typedef int8_t _velocity_type;
    _velocity_type velocity;
    typedef int8_t _trans_diff_type;
    _trans_diff_type trans_diff;
    typedef int8_t _ctrl_type;
    _ctrl_type ctrl;

    lli_ctrl() : steering(0),
                 velocity(0),
                 trans_diff(0),
                 ctrl(0) {
    }

    virtual int serialize(unsigned char *outbuffer) const override {
        int offset = 0;
        union {
            int8_t real;
            uint8_t base;
        } u_steering;
        u_steering.real = this->steering;
        *(outbuffer + offset + 0) = (u_steering.base >> (8 * 0)) & 0xFF;
        offset += sizeof(this->steering);
        union {
            int8_t real;
            uint8_t base;
        } u_velocity;
        u_velocity.real = this->velocity;
        *(outbuffer + offset + 0) = (u_velocity.base >> (8 * 0)) & 0xFF;
        offset += sizeof(this->velocity);
        union {
            int8_t real;
            uint8_t base;
        } u_trans_diff;
        u_trans_diff.real = this->trans_diff;
        *(outbuffer + offset + 0) = (u_trans_diff.base >> (8 * 0)) & 0xFF;
        offset += sizeof(this->trans_diff);
        union {
            int8_t real;
            uint8_t base;
        } u_ctrl;
        u_ctrl.real = this->ctrl;
        *(outbuffer + offset + 0) = (u_ctrl.base >> (8 * 0)) & 0xFF;
        offset += sizeof(this->ctrl);
        return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override {
        int offset = 0;
        union {
            int8_t real;
            uint8_t base;
        } u_steering;
        u_steering.base = 0;
        u_steering.base |= ((uint8_t)(*(inbuffer + offset + 0))) << (8 * 0);
        this->steering = u_steering.real;
        offset += sizeof(this->steering);
        union {
            int8_t real;
            uint8_t base;
        } u_velocity;
        u_velocity.base = 0;
        u_velocity.base |= ((uint8_t)(*(inbuffer + offset + 0))) << (8 * 0);
        this->velocity = u_velocity.real;
        offset += sizeof(this->velocity);
        union {
            int8_t real;
            uint8_t base;
        } u_trans_diff;
        u_trans_diff.base = 0;
        u_trans_diff.base |= ((uint8_t)(*(inbuffer + offset + 0))) << (8 * 0);
        this->trans_diff = u_trans_diff.real;
        offset += sizeof(this->trans_diff);
        union {
            int8_t real;
            uint8_t base;
        } u_ctrl;
        u_ctrl.base = 0;
        u_ctrl.base |= ((uint8_t)(*(inbuffer + offset + 0))) << (8 * 0);
        this->ctrl = u_ctrl.real;
        offset += sizeof(this->ctrl);
        return offset;
    }

    virtual const char *getType() override { return "svea_msgs/lli_ctrl"; };
    virtual const char *getMD5() override { return "f4c1d25e08fe7c24fca84a1ec3ad2a96"; };
};

} // namespace svea_msgs
#endif
