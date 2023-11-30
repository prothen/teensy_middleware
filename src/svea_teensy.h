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
namespace SVEA {
typedef ros::NodeHandle_<ArduinoHardware,
                         MAX_ROS_SUBSCRIBERS,
                         MAX_ROS_PUBLISHERS,
                         ROS_IN_BUFFER_SIZE,
                         ROS_OUT_BUFFER_SIZE> 
        NodeHandle;
}
SVEA::NodeHandle nh;
lli_ctrl_out_t MSG_REMOTE;   //!< Message used for sending the remote signals
lli_ctrl_out_t MSG_ACTUATED; //!< Message sending actuated messages
lli_encoder_t MSG_ENCODER;   //!< Message used for outgoing wheel encoder messages
lli_encoder_t MSG_DEBUG;

//!< Message used for misc debugging
ros::Publisher remote_pub("lli/remote", &MSG_REMOTE);                 //!< Remote message publisher
ros::Publisher ctrl_actuated_pub("lli/ctrl_actuated", &MSG_ACTUATED); //!< Actuated control message publisher
ros::Publisher encoder_pub("lli/encoder", &MSG_ENCODER);
ros::Publisher debug_pub("lli/debug", &MSG_DEBUG);

//!< Encoder reading publisher
ros::Subscriber<lli_ctrl_in_t> ctrl_request("lli/ctrl_request", &callbackCtrlRequest);            //!< Controll request subscriber
ros::Subscriber<svea_msgs::lli_emergency> emergency_request("lli/emergency", &callbackEmergency); //!< Controll request subscriber


/*
 * ACTUATION FUNCTIONS
 */

/*!
 * @brief Set actuation PWM
 * convert a 8 bit actuation value to a pwm signal and send it to the pwm board.
 * The value gets scaled to a duration that suits the servos (approximately
 * 1 to 2 milli seconds).
 * @see INPUT_SCALE
 * @see PWM_NEUTRAL_TICK
 * To avoid servo jitter at neutral a small dead zone exists around 0.
 * @see DEAD_ZONE
 * @param channel The channel (pin) of the pwm board to send to.
 * @param in_value Value, between -127 and 127. to send.
 */
inline void setPwmDriver(uint8_t channel, int8_t actuation_value) {
    if (abs_difference(actuation_value, ACTUATION_NEUTRAL) < DEAD_ZONE) {
        actuation_value = ACTUATION_NEUTRAL;
    }
    uint16_t off_tick = PWM_OUT_NEUTRAL_TICK[channel] + OUTPUT_SCALE[channel] * actuation_value;

    ACTUATED_TICKS[channel] = off_tick;
    analogWrite(PWM_OUT_PINS[channel], off_tick);
}

/*! @brief Send settings to the pwm board through setPwmDriver()
 *
 * If any setting or actuation code have changed, the current
 * actuation values and flags will be published on /lli/ctrl_actuated.
 * If nothing have been changed, nothing will be sent to the
 * pwm board or /lli/ctrl_actuated.
 * @see setPwmDriver
 * @param actuation_values array containing 5 values.
 */
void actuate(const int8_t actuation_values[]) {
    /* Set steering and velocity */
    static int8_t previous_setting[5] = {IDLE_ACTUATION[0],
                                         IDLE_ACTUATION[1],
                                         MSG_TO_ACT_OFF[0],
                                         IDLE_ACTUATION[3],
                                         IDLE_ACTUATION[4]};
    static uint8_t last_actuated_code = 0; // Code that was last sent to ROS
    int8_t has_changed = 0;
    for (int i = 0; i < 5; i++) {
        if (actuation_values[i] != previous_setting[i] && actuation_values[i] != -128) {
            setPwmDriver(i, actuation_values[i]);
            previous_setting[i] = actuation_values[i];
            has_changed++;
        }
    }
    // Send actuated values to ROS
    uint8_t actuated_code = getActuatedCode();
    if (has_changed > 0 || actuated_code ^ last_actuated_code) {
        MSG_ACTUATED.steering = STEERING_DIRECTION * previous_setting[0];
        MSG_ACTUATED.velocity = previous_setting[1];
        MSG_ACTUATED.trans_diff = bit(ENABLE_GEARCHANGE_BIT) | bit(ENABLE_FDIFCHANGE_BIT) | bit(ENABLE_RDIFCHANGE_BIT);
        for (int i = 0; i < 3; i++) {
            MSG_ACTUATED.trans_diff += previous_setting[i + 2] == MSG_TO_ACT_ON[i] ? bit(i) : 0;
        }
        MSG_ACTUATED.ctrl = actuated_code;
        ctrl_actuated_pub.publish(&MSG_ACTUATED);
        last_actuated_code = actuated_code;
    }
}

/*!
 * @brief set the control code in messages sent to the computer
 */
inline uint8_t getActuatedCode() {
    return SW_IDLE | pwm_reader::REM_IDLE << 1 | pwm_reader::REM_OVERRIDE << 2 | SW_EMERGENCY << 3;
}
// END OF ACTUATION FUNCTIONS

/*
 * SETUP ROS
 */

/*!
 * @brief Callback function for control requests from ROS
 * Interprets the message and sends the values to the pwm board
 * through actuate().
 *
 * @param data Message to be evaluated
 */
void callbackCtrlRequest(const lli_ctrl_in_t &data) {
    SW_ACTUATION[0] = STEERING_DIRECTION * data.steering;
    SW_ACTUATION[1] = data.velocity;

    // Set the on/off values
    for (int i = 0; i < 3; i++) {
        // Only change gear/diff settings if the corresponding enable change bit is set
        if (data.trans_diff & bit(ENABLE_ACT_CHANGE_BITS[i])) {
            int8_t is_on = data.trans_diff & bit(ACT_BITS[i]);
            SW_ACTUATION[i + 2] = is_on ? MSG_TO_ACT_ON[i] : MSG_TO_ACT_OFF[i];
        } else { // Otherwise use the previous value
            SW_ACTUATION[i + 2] = -128;
        }
    }
    SW_IDLE = false;
    SW_T_RECIEVED = millis();
    if (!pwm_reader::REM_OVERRIDE && !SW_EMERGENCY) {
        actuate(SW_ACTUATION);
    }
}

/*!
 * @brief Callback function for emergency requests from ROS
 * Set/clear the emergency flag depending on message content.
 * The ID field functionality is not yet implemented.
 *
 * @param data Message to be evaluated
 */
void callbackEmergency(const svea_msgs::lli_emergency &data) {
    SW_EMERGENCY = data.emergency;
    SW_IDLE = false;
    SW_T_RECIEVED = millis();
}
/*@}*/

// END OF ROS SETUP

/*!
 * @brief Check if the emergency brake should be engaged.
 * Should be called every update loop.
 *
 * The emergency brake will be activated if the SW_EMERGENCY
 * flag is true. A braking sequence is then initiated.
 * The sequence first make sures that the ESC is not in
 * a reverse state, and then applies full brakes.
 * For proper functionality all other actuation sources
 * must respect the SW_EMERGENCY flag and not send actuation
 * signals until it is cleared.
 *
 * @return true if the emergency brake is engaged, false otherwise.
 */
bool checkEmergencyBrake() {
    enum States {
        NO_EMERGENCY,
        EMERGENCY_SET,
        WAIT_FOR_UNSET_REVERSE,
        BRAKING,
        DONE_BRAKING,
    };
    static States state = NO_EMERGENCY;
    static unsigned long last_time = millis();
    const unsigned long reverse_wait_time = 50; // (ms)
    const int8_t init_brake_actuation[] = {-128, 15, -128, -128, -128};
    const int8_t brake_actuation[] = {-128, -127, -128, -128, -128};
    // const unsigned long minimum_emergency_duration = 500;
    unsigned long wait_duration = (millis() - last_time);
    if (SW_EMERGENCY == false) { // &&
        // wait_duration > minimum_emergency_duration){
        state = NO_EMERGENCY;
    }
    switch (state) {
    case NO_EMERGENCY:
        if (SW_EMERGENCY) {
            state = EMERGENCY_SET;
        } else {
            break;
        }
    case EMERGENCY_SET:
        actuate(init_brake_actuation);
        last_time = millis();
        state = WAIT_FOR_UNSET_REVERSE;
        break;
    case WAIT_FOR_UNSET_REVERSE:
        if (wait_duration > reverse_wait_time) {
            state = BRAKING;
        } else {
            break;
        }
    case BRAKING:
        actuate(brake_actuation);
        state = DONE_BRAKING;
        break;
    case DONE_BRAKING:
        break;
    default:
        break;
    }
    return state != NO_EMERGENCY;
}

void publishRemoteReading(int8_t actuation_values[5]) {
    MSG_REMOTE.steering = actuation_values[0];
    MSG_REMOTE.velocity = actuation_values[1];
    // Remote messages should always enforce change
    MSG_REMOTE.trans_diff = bit(ENABLE_GEARCHANGE_BIT) | bit(ENABLE_FDIFCHANGE_BIT) | bit(ENABLE_RDIFCHANGE_BIT);
    for (int i = 0; i < 3; i++) {
        if (actuation_values[i + 2] == MSG_TO_ACT_ON[i]) {
            MSG_REMOTE.trans_diff |= bit(i);
        }
    }
    MSG_REMOTE.ctrl = getActuatedCode();
    remote_pub.publish(&MSG_REMOTE);
}

/*!
 * @brief Steering callibration functionality. Should be called in every loop update.
 *
 * Initiate callibration by holding down button 0 for 1 second.
 * The LEDs should turn yellow. Now turn the tires as far to the left
 * as they can go without pushing against the chassis.
 * Push button 0 again. The LEDs should turn blue.
 * Turn the tire as far to the right as they can go without
 * pushing against the chassis.
 * Push button 0 again and the LEDs should blink for a short while.
 * The callibration is complet and the values have been saved to flash.
 *
 * The calibration process can be aborted by pushing button 1.
 *
 * @return true if a calibration is ongoing, false otherwise
 */
bool callibrateSteering() {
    enum CalibState {
        NOT_CALIBRATING,
        TURN_LEFT,
        TURN_RIGHT,
        DONE,
    };
    const uint8_t calib_button = 0;
    const uint8_t abort_button = 1;
    static CalibState state = NOT_CALIBRATING;
    static float max_pwm = DEFAULT_PWM_OUT_MAX_PW[0];
    static float min_pwm = DEFAULT_PWM_OUT_MIN_PW[0];
    static unsigned long done_time;
    const unsigned long done_duration = 1500; // ms
    if (buttons::readEvent(abort_button) == buttons::PRESSED) {
        state = NOT_CALIBRATING;
        if (loadSteeringValues(min_pwm, max_pwm)) {
            setSteeringPwm(min_pwm, max_pwm);
        }
    }
    switch (state) {
    case NOT_CALIBRATING:
        if (buttons::readEvent(calib_button) == buttons::LONG_PRESSED && !pwm_reader::REM_IDLE) {
            state = TURN_LEFT;
            int steer_ix = 0;
            max_pwm = DEFAULT_PWM_OUT_MAX_PW[steer_ix];
            min_pwm = DEFAULT_PWM_OUT_MIN_PW[steer_ix];
            //led::setLEDs(led::color_yelow);

        }
        break;
    case TURN_LEFT:
        if (buttons::readEvent(calib_button) == buttons::PRESSED) {
            min_pwm = 1000.0 * ACTUATED_TICKS[0] / (PWM_OUT_RES * PWM_OUT_FREQUENCY);
            //led::setLEDs(led::color_blue);
        }
        break;
    case TURN_RIGHT:
        if (buttons::readEvent(calib_button) == buttons::PRESSED) {
            max_pwm = 1000.0 * ACTUATED_TICKS[0] / (PWM_OUT_RES * PWM_OUT_FREQUENCY);
            setSteeringPwm(min_pwm, max_pwm);
            saveSteeringValues(min_pwm, max_pwm);
            done_time = millis();
            //led::pushLEDs(led::color_blue);
            state = DONE;
        }
        break;
    case DONE:
        if (millis() - done_time < done_duration) {
            //led::blinkLEDs();
        } else {
            state = NOT_CALIBRATING;
        }
        break;
    }
    return state != NOT_CALIBRATING;
}


/*@}*/
#endif /* SVEA_LLI */
