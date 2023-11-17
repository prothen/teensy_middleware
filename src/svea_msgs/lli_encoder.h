#ifndef _ROS_svea_msgs_lli_encoder_h
#define _ROS_svea_msgs_lli_encoder_h

#include <ros/msg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

namespace svea_msgs {

class lli_encoder : public ros::Msg {
public:
    typedef uint8_t _right_ticks_type;
    _right_ticks_type right_ticks;
    typedef uint8_t _left_ticks_type;
    _left_ticks_type left_ticks;
    typedef uint32_t _right_time_delta_type;
    _right_time_delta_type right_time_delta;
    typedef uint32_t _left_time_delta_type;
    _left_time_delta_type left_time_delta;

    lli_encoder() : right_ticks(0),
                    left_ticks(0),
                    right_time_delta(0),
                    left_time_delta(0) {
    }

    virtual int serialize(unsigned char *outbuffer) const override {
        int offset = 0;
        *(outbuffer + offset + 0) = (this->right_ticks >> (8 * 0)) & 0xFF;
        offset += sizeof(this->right_ticks);
        *(outbuffer + offset + 0) = (this->left_ticks >> (8 * 0)) & 0xFF;
        offset += sizeof(this->left_ticks);
        *(outbuffer + offset + 0) = (this->right_time_delta >> (8 * 0)) & 0xFF;

        *(outbuffer + offset + 1) = (this->right_time_delta >> (8 * 1)) & 0xFF;
        *(outbuffer + offset + 2) = (this->right_time_delta >> (8 * 2)) & 0xFF;
        *(outbuffer + offset + 3) = (this->right_time_delta >> (8 * 3)) & 0xFF;
        offset += sizeof(this->right_time_delta);
        *(outbuffer + offset + 0) = (this->left_time_delta >> (8 * 0)) & 0xFF;
        *(outbuffer + offset + 1) = (this->left_time_delta >> (8 * 1)) & 0xFF;
        *(outbuffer + offset + 2) = (this->left_time_delta >> (8 * 2)) & 0xFF;
        *(outbuffer + offset + 3) = (this->left_time_delta >> (8 * 3)) & 0xFF;
        offset += sizeof(this->left_time_delta);
        return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override {
        int offset = 0;
        this->right_ticks = ((uint8_t)(*(inbuffer + offset)));
        offset += sizeof(this->right_ticks);
        this->left_ticks = ((uint8_t)(*(inbuffer + offset)));
        offset += sizeof(this->left_ticks);
        this->right_time_delta = ((uint32_t)(*(inbuffer + offset)));
        this->right_time_delta |= ((uint32_t)(*(inbuffer + offset + 1))) << (8 * 1);
        this->right_time_delta |= ((uint32_t)(*(inbuffer + offset + 2))) << (8 * 2);
        this->right_time_delta |= ((uint32_t)(*(inbuffer + offset + 3))) << (8 * 3);
        offset += sizeof(this->right_time_delta);
        this->left_time_delta = ((uint32_t)(*(inbuffer + offset)));
        this->left_time_delta |= ((uint32_t)(*(inbuffer + offset + 1))) << (8 * 1);
        this->left_time_delta |= ((uint32_t)(*(inbuffer + offset + 2))) << (8 * 2);
        this->left_time_delta |= ((uint32_t)(*(inbuffer + offset + 3))) << (8 * 3);
        offset += sizeof(this->left_time_delta);

        return offset;
    }

    virtual const char *getType() override { return "svea_msgs/lli_encoder"; };
    virtual const char *getMD5() override { return "816c28762606d74fcd62d1f80297848e"; };
};

} // namespace svea_msgs
#endif
