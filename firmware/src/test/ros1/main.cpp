/**
 * Basic example for blink application with rosserial_arduino library.
 *
 * Author: Philipp Rothenhäusler, Stockholm 2020
 *
*/
#include <Arduino.h>

#ifdef TMW_TEST_ROS
#include <ros.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float32.h>

#ifdef TMW_TEST_ROS_CUSTOM
#include <testbed_msgs/Cmd.h>
#endif
#endif

#define blink digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));

volatile uint64_t T = 2000;
volatile uint64_t deadline_blink = 0;

#ifdef TMW_TEST_ROS
std_msgs::UInt64 my_msg{};
std_msgs::Float32 msg_pub{};
void cb_msg(const std_msgs::UInt64&);
#ifdef TMW_TEST_ROS_CUSTOM
void cb_custom_msg(const testbed_msgs::Cmd&);
#endif

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt64> sub("/teensy_mw/sub", &cb_msg);
#ifdef TMW_TEST_ROS_CUSTOM
ros::Subscriber<testbed_msgs::Cmd> sub_custom("/teensy_mw/sub_custom", &cb_custom_msg);
#endif
ros::Publisher pub("/teensy_mw/pub",  &msg_pub);
void cb_msg(const std_msgs::UInt64& msg){
    T = msg.data;
}
#ifdef TMW_TEST_ROS_CUSTOM
void cb_custom_msg(const testbed_msgs::Cmd& msg){
        ;
}
#endif
#endif

void setup(){
    pinMode(LED_BUILTIN, OUTPUT);

#ifdef TMW_TEST_ROS
    nh.initNode();
    nh.subscribe(sub);
#ifdef TMW_TEST_ROS_CUSTOM
    nh.subscribe(sub_custom);
#endif
    nh.advertise(pub);
#endif
}

void loop() {
    if (millis() > deadline_blink){
        deadline_blink = millis() + T;
#ifdef TMW_TEST_ROS
        msg_pub.data = (float) millis();
        pub.publish(&msg_pub);
#endif
        blink;
    }
#ifdef TMW_TEST_ROS
    nh.spinOnce();
#endif
}
