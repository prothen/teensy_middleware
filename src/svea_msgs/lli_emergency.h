#ifndef _ROS_svea_msgs_lli_emergency_h
#define _ROS_svea_msgs_lli_emergency_h

#include <ros/msg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

namespace svea_msgs {

class lli_emergency : public ros::Msg {
public:
    typedef bool _emergency_type;
    _emergency_type emergency;
    typedef uint32_t _sender_id_type;
    _sender_id_type sender_id;

    lli_emergency() : emergency(0),
                      sender_id(0) {
    }

    virtual int serialize(unsigned char *outbuffer) const override {
        int offset = 0;
        union {
            bool real;
            uint8_t base;
        } u_emergency;
        u_emergency.real = this->emergency;
        *(outbuffer + offset + 0) = (u_emergency.base >> (8 * 0)) & 0xFF;
        offset += sizeof(this->emergency);
        *(outbuffer + offset + 0) = (this->sender_id >> (8 * 0)) & 0xFF;
        *(outbuffer + offset + 1) = (this->sender_id >> (8 * 1)) & 0xFF;
        *(outbuffer + offset + 2) = (this->sender_id >> (8 * 2)) & 0xFF;
        *(outbuffer + offset + 3) = (this->sender_id >> (8 * 3)) & 0xFF;
        offset += sizeof(this->sender_id);
        return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override {
        int offset = 0;
        union {
            bool real;
            uint8_t base;
        } u_emergency;
        u_emergency.base = 0;
        u_emergency.base |= ((uint8_t)(*(inbuffer + offset + 0))) << (8 * 0);
        this->emergency = u_emergency.real;
        offset += sizeof(this->emergency);
        this->sender_id = ((uint32_t)(*(inbuffer + offset)));
        this->sender_id |= ((uint32_t)(*(inbuffer + offset + 1))) << (8 * 1);
        this->sender_id |= ((uint32_t)(*(inbuffer + offset + 2))) << (8 * 2);
        this->sender_id |= ((uint32_t)(*(inbuffer + offset + 3))) << (8 * 3);
        offset += sizeof(this->sender_id);
        return offset;
    }

    virtual const char *getType() override { return "svea_msgs/lli_emergency"; };
    virtual const char *getMD5() override { return "a86ab2ca3efbddadc665a4c5c1a1f723"; };
};

} // namespace svea_msgs
#endif
