#ifndef SVEA_LLI
#define SVEA_LLI
#include <Arduino.h>
#include <EEPROM.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

#include "control/actuation_constants.h"
#include "svea_msgs/lli_ctrl.h"
#include "svea_msgs/lli_emergency.h"
#include "svea_msgs/lli_encoder.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"

#include "settings.h"

/*! @file svea_teensy.h*/

/*!
 * @defgroup PwmOutputChannels PWM output pins on the Teensy
 */
/*@{*/
const uint8_t PWM_OUT_STEER_PIN = 15; //!< Pwm pin for steering
const uint8_t PWM_OUT_VELOC_PIN = 5;  //!< Pwm pin for velocity
const uint8_t PWM_OUT_GEAR_PIN = 14;  //!< Pwm pin for transmission
const uint8_t PWM_OUT_FDIFF_PIN = 9;  //!< Pwm pin for front differential lock
const uint8_t PWM_OUT_RDIFF_PIN = 6;  //!< Pwm pin for rear differential lock
//! Array with mapping for the PWM channels
const uint8_t PWM_OUT_PINS[5] = {
    PWM_OUT_STEER_PIN,
    PWM_OUT_VELOC_PIN,
    PWM_OUT_GEAR_PIN,
    PWM_OUT_FDIFF_PIN,
    PWM_OUT_RDIFF_PIN};
/*@}*/

/** \addtogroup ActuationToOutput
 *  @{
 */
//! The 16-bit value corresponding to a neutral duty cycle
unsigned int PWM_OUT_NEUTRAL_TICK[5];
//! The scaling factor between actuation values and the 12-bit values sent to the pwm board.
float OUTPUT_SCALE[5];
//! Storage for the last actuated pwm register values
uint16_t ACTUATED_TICKS[5] = {0, 0, 0, 0, 0};

float STEER_PWM_OUT_MIN_PW = DEFAULT_PWM_OUT_MIN_PW[0];
float STEER_PWM_OUT_MAX_PW = DEFAULT_PWM_OUT_MAX_PW[0];

void setSteeringPwm(float desired_min_pwm, float desired_max_pwm) {
    const uint8_t steer_ix = 0;
    STEER_PWM_OUT_MIN_PW = desired_min_pwm;
    STEER_PWM_OUT_MAX_PW = desired_max_pwm;
    unsigned int min_pwm_tick = desired_min_pwm * PWM_OUT_RES * PWM_OUT_FREQUENCY;
    unsigned int max_pwm_tick = desired_max_pwm * PWM_OUT_RES * PWM_OUT_FREQUENCY;
    PWM_OUT_NEUTRAL_TICK[steer_ix] = int((min_pwm_tick + (max_pwm_tick - min_pwm_tick) * 0.5) / 1000.0 + 0.5);
    OUTPUT_SCALE[steer_ix] = ((PWM_OUT_FREQUENCY * PWM_OUT_RES * desired_min_pwm) / 1000.0 - PWM_OUT_NEUTRAL_TICK[steer_ix]) / (float)ACTUATION_MIN;
}

//! EEPROM address where the steering calibration values are stored.
const int EEP_STEERING_ADDRESS = 0;
/*
 * Load saved pwm values from EEPROM.
 * return true if the values are found
 */
bool loadSteeringValues(float &min_pwm, float &max_pwm) {
    int eeAddress = EEP_STEERING_ADDRESS;
    uint8_t data = EEPROM.read(eeAddress);
    if (data == 255) {
        return false;
    }
    EEPROM.get(eeAddress, min_pwm);
    eeAddress += sizeof(float);
    data = EEPROM.read(eeAddress);
    if (data == 255) {
        return false;
    }
    EEPROM.get(eeAddress, max_pwm);
    return true;
}

void saveSteeringValues(float min_pwm, float max_pwm) {
    int eeAddress = EEP_STEERING_ADDRESS;
    EEPROM.put(eeAddress, min_pwm);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, max_pwm);
}

void resetSteeringValues() {
    int start_addr = EEP_STEERING_ADDRESS;
    int end_addr = EEP_STEERING_ADDRESS + 2 * sizeof(float);
    for (int addr = start_addr; addr < end_addr; addr++) {
        EEPROM.write(addr, 255);
    }
    const int steer_ix = 0;
    setSteeringPwm(DEFAULT_PWM_OUT_MIN_PW[steer_ix], DEFAULT_PWM_OUT_MAX_PW[steer_ix]);
}

void setupActuation() {
    // Setup pin modes for pwm output pins
    for (int i = 0; i < 5; i++) {
        pinMode(PWM_OUT_PINS[i], OUTPUT);
        analogWriteFrequency(PWM_OUT_PINS[i], PWM_OUT_FREQUENCY);
    }
    analogWriteResolution(PWM_OUT_BITS);
    // Calculate scaling values
    for (int i = 0; i < 5; i++) {
        unsigned int min_pwm_tick = DEFAULT_PWM_OUT_MIN_PW[i] * PWM_OUT_RES * PWM_OUT_FREQUENCY;
        unsigned int max_pwm_tick = DEFAULT_PWM_OUT_MAX_PW[i] * PWM_OUT_RES * PWM_OUT_FREQUENCY;
        PWM_OUT_NEUTRAL_TICK[i] = int((min_pwm_tick + (max_pwm_tick - min_pwm_tick) * 0.5) / 1000.0 + 0.5);
        OUTPUT_SCALE[i] = ((PWM_OUT_FREQUENCY * PWM_OUT_RES * DEFAULT_PWM_OUT_MIN_PW[i]) / 1000.0 - PWM_OUT_NEUTRAL_TICK[i]) / (float)ACTUATION_MIN;
    }
    float max_pwm;
    float min_pwm;
    if (loadSteeringValues(min_pwm, max_pwm)) {
        setSteeringPwm(min_pwm, max_pwm);
    }
}

/** @}*/

/*
 * Message type definitions and related constants
 */
typedef svea_msgs::lli_ctrl lli_ctrl_in_t;    //!< Message type for incomming messages
typedef svea_msgs::lli_ctrl lli_ctrl_out_t;   //!< Message type for outgoing messages'
typedef svea_msgs::lli_encoder lli_encoder_t; //!< Message type for encoder messages

/*
 * Storage variables
 */

/*!
 * @defgroup ActuationValueStorage Actuation value storage
 * The order is Steering, velocity, gear, front differential, rear differential
 */
/*@{*/
//! Actuation values sent from the computer
int8_t SW_ACTUATION[5] = {0, 0, MSG_TO_ACT_OFF[0], MSG_TO_ACT_OFF[1], MSG_TO_ACT_OFF[2]};
// const int8_t EMERGENCY_BRAKE_ACTUATION[5] = {0,-127,-128,-128,-128};

/*!
 * @defgroup StatusVariables Status variables
 */
/*@{*/
unsigned long SW_T_RECIEVED = millis(); //!< Time when last message was recieved from the computer
bool SW_IDLE = true;                    //!< True if the computer is considered idle
// unsigned long LAST_NEGATIVE_VEL_TIME = millis();
/*!
 *  True if the computer has set an emergency.
 *  Should bloc other actuation signals from computer until cleared.
 */
bool SW_EMERGENCY = false;
//! Emergency cleared if override active and
//  no message has been recieved for this many ms.
unsigned long EMERGENCY_T_CLEAR_LIMIT = 5000;
/*@}*/

/* Function definitions */
inline void setPwmDriver(uint8_t channel, int8_t in_value);
void actuate(const int8_t actuation_values[]);
inline uint8_t getActuatedCode();
void callbackCtrlRequest(const lli_ctrl_in_t &data);
void callbackEmergency(const svea_msgs::lli_emergency &data);
// void adjustPwmFrequency();
// void processEncoderTicks();

/*!
 * @defgroup ROSSetup Variables used by ROS
 */
/*@{*/
//! NodeHandle class definition
ros::NodeHandle_<ArduinoHardware,
                 MAX_ROS_SUBSCRIBERS,
                 MAX_ROS_PUBLISHERS,
                 ROS_IN_BUFFER_SIZE,
                 ROS_OUT_BUFFER_SIZE>
    nh;
lli_ctrl_out_t MSG_REMOTE;   //!< Message used for sending the remote signals
lli_ctrl_out_t MSG_ACTUATED; //!< Message sending actuated messages
lli_encoder_t MSG_ENCODER;   //!< Message used for outgoing wheel encoder messages
lli_encoder_t MSG_DEBUG;

// BN055 Stuff
sensor_msgs::Imu MSG_IMU;
sensor_msgs::MagneticField MSG_MAG;
sensor_msgs::Temperature MSG_TEMP;

//!< Message used for misc debugging
ros::Publisher remote_pub("lli/remote", &MSG_REMOTE);                 //!< Remote message publisher
ros::Publisher ctrl_actuated_pub("lli/ctrl_actuated", &MSG_ACTUATED); //!< Actuated control message publisher
ros::Publisher encoder_pub("lli/encoder", &MSG_ENCODER);
ros::Publisher debug_pub("lli/debug", &MSG_DEBUG);

ros::Publisher imu_pub("/imu/data", &MSG_IMU);
ros::Publisher imu_mag("/imu/mag", &MSG_MAG);
ros::Publisher imu_temp("/imu/temp", &MSG_TEMP);

//!< Encoder reading publisher
ros::Subscriber<lli_ctrl_in_t> ctrl_request("lli/ctrl_request", &callbackCtrlRequest);            //!< Controll request subscriber
ros::Subscriber<svea_msgs::lli_emergency> emergency_request("lli/emergency", &callbackEmergency); //!< Controll request subscriber

/*@}*/
#endif /* SVEA_LLI */
