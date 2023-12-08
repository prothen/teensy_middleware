#include <Arduino.h>
#include <limits>
#include <ros.h>

#include "svea_msgs/lli_ctrl.h"
#include "svea_msgs/lli_emergency.h"
#include "svea_msgs/lli_encoder.h"

#include "control/buttons.h"
#include "control/encoders.h"

#include "control/pwm_reader.h"

#include "external_ic/IMU.h"
#include "external_ic/gpio_ext.h"

#include "settings.h"
#include "svea_teensy.h"

#include "utility.h"

//! Setup ROS
void rosSetup() {
    nh.getHardware()->setBaud(SERIAL_BAUD_RATE);
    nh.initNode();
    // NOTE: Putting advertise before subscribe destroys
    //       EVERYTHING :DDDD~~~~~

    nh.subscribe(ctrl_request);
    // nh.negotiateTopics();

    nh.subscribe(emergency_request);
    // nh.negotiateTopics();

    nh.advertise(remote_pub);
    // nh.negotiateTopics();

    nh.advertise(ctrl_actuated_pub);
    // nh.negotiateTopics();

    nh.advertise(encoder_pub);
    // nh.negotiateTopics();

    nh.advertise(debug_pub);
    // nh.negotiateTopics();
}

SVEA::IMU imu_sensor(nh);

//! Arduino setup function
void setup() {
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
    Serial.println("Starting setup");
    
    while (nh.connected()) {
        nh.spinOnce();
    }
    setupActuation();
    pinMode(LED_BUILTIN, OUTPUT);
    Wire1.begin();
    setup_gpio();
    pwm_reader::setup();
    // encoders::setup();

    // FastLED.addLeds<SK9822,6>(leds, 1);

    if (!imu_sensor.open()) {
        // TODO: Handle error
    }

    rosSetup();
    Serial.println("Setup done");
}

// Servo turned off by default
static bool servo_idle = false;
int l = 0;
//! Main loop
void loop() {

    int sw_status = nh.spinOnce();
    unsigned long d_since_last_msg = millis() - SW_T_RECIEVED;
    checkEmergencyBrake();
    int8_t remote_actuations[5];
    if (pwm_reader::processPwm(remote_actuations)) {
        if (!pwm_reader::REM_IDLE) {
            publishRemoteReading(remote_actuations);
            if ((SW_IDLE && !SW_EMERGENCY) || pwm_reader::REM_OVERRIDE) {
                actuate(remote_actuations);
            }
            if (d_since_last_msg > EMERGENCY_T_CLEAR_LIMIT && pwm_reader::REM_OVERRIDE && SW_EMERGENCY) {
                SW_EMERGENCY = false;
            }
        }
    }

    if (sw_status != ros::SPIN_OK || d_since_last_msg > SW_TIMEOUT) {
        SW_IDLE = true;
    }

    if ((pwm_reader::REM_IDLE && SW_IDLE && !SW_EMERGENCY) && !servo_idle) {
        actuate(IDLE_ACTUATION);
        gpio_extender.digitalWrite(SERVO_PWR_ENABLE_PIN, LOW);
        servo_idle = true;
    } else {
        if (servo_idle) {
            gpio_extender.digitalWrite(SERVO_PWR_ENABLE_PIN, HIGH);
            servo_idle = false;
        }
    }

    imu_sensor.update();

    // PCB LED Logic
    // buttons::updateButtons();
    // if (!callibrateSteering()) {
    //    if (servo_idle && !SW_EMERGENCY) {
    //        // led::blinkLEDs();
    //    } else {
    //        // if (SW_IDLE) {
    //        //     led::setLED(0, led::color_red);
    //        // } else {
    //        //     led::setLED(0, led::color_green);
    //        // }
    //        // if (pwm_reader::REM_IDLE) {
    //        //     led::setLED(1, led::color_red);
    //        // } else {
    //        //     led::setLED(1, led::color_green);
    //        // }
    //        // if (!pwm_reader::REM_OVERRIDE) {
    //        //     led::setLED(2, led::color_red);
    //        // } else {
    //        //     led::setLED(2, led::color_green);
    //        // }
    //        // if (!SW_EMERGENCY) {
    //        //     led::setLED(3, led::color_red);
    //        // } else {
    //        //     led::setLED(3, led::color_green);
    //        // }
    //
    //    } //
    //}
    // led::updateLEDs();
}
