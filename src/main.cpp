#include <Arduino.h>
#include <limits>
#include <ros.h>

#include "svea_msgs/lli_ctrl.h"
#include "svea_msgs/lli_emergency.h"
#include "svea_msgs/lli_encoder.h"

#include "control/buttons.h"
#include "control/encoders.h"
#include "control/led_control.h"
#include "control/pwm_reader.h"

#include "external_ic/IMU.h"
#include "external_ic/gpio_ext.h"

#include "settings.h"
#include "svea_teensy.h"

#include "utility.h"

/*! @file svea_arduino_src.ino*/

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

void EncoderReadingToMsg(const encoders::encoder_reading_t &reading, lli_encoder_t &msg) {
    msg.right_ticks = reading.right_ticks;
    msg.left_ticks = reading.left_ticks;
    msg.right_time_delta = reading.right_time_delta;
    msg.left_time_delta = reading.left_time_delta;
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
            setSteeringPwm(min_pwm, max_pwm);
            led::setLEDs(led::color_yelow);
        }
        break;
    case TURN_LEFT:
        if (buttons::readEvent(calib_button) == buttons::PRESSED) {
            min_pwm = 1000.0 * ACTUATED_TICKS[0] / (PWM_OUT_RES * PWM_OUT_FREQUENCY);
            state = TURN_RIGHT;
            led::setLEDs(led::color_blue);
        }
        break;
    case TURN_RIGHT:
        if (buttons::readEvent(calib_button) == buttons::PRESSED) {
            max_pwm = 1000.0 * ACTUATED_TICKS[0] / (PWM_OUT_RES * PWM_OUT_FREQUENCY);
            setSteeringPwm(min_pwm, max_pwm);
            saveSteeringValues(min_pwm, max_pwm);
            done_time = millis();
            led::pushLEDs(led::color_blue);
            state = DONE;
        }
        break;
    case DONE:
        if (millis() - done_time < done_duration) {
            led::blinkLEDs();
        } else {
            state = NOT_CALIBRATING;
        }
        break;
    }
    return state != NOT_CALIBRATING;
}

//! Setup ROS
void rosSetup() {
    nh.getHardware()->setBaud(SERIAL_BAUD_RATE);
    nh.initNode();
    // NOTE: Putting advertise before subscribe destroys
    //       EVERYTHING :DDDD~~~~~

    nh.subscribe(ctrl_request);
    nh.negotiateTopics();

    nh.subscribe(emergency_request);
    nh.negotiateTopics();

    nh.advertise(remote_pub);
    nh.negotiateTopics();

    nh.advertise(ctrl_actuated_pub);
    nh.negotiateTopics();

    nh.advertise(encoder_pub);
    nh.negotiateTopics();

    nh.advertise(debug_pub);
    nh.negotiateTopics();

    nh.advertise(imu_mag);
    nh.negotiateTopics();

    nh.advertise(imu_pub);
    nh.negotiateTopics();

    nh.advertise(imu_temp);
    nh.negotiateTopics();
}

// Im sorry kaj, it will do for now
bool imuConnected = false;
//! Arduino setup function
void setup() {
    while(nh.connected()){
        nh.spinOnce();
    }
    setupActuation();
    pinMode(LED_BUILTIN, OUTPUT);
    Wire1.begin();
    setup_gpio();
    pwm_reader::setup();
    encoders::setup();
    imuConnected = setupIMU();

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

    encoders::encoder_reading_t reading;
    if (encoders::processEncoderTicks(reading)) {
        EncoderReadingToMsg(reading, MSG_ENCODER);
        encoder_pub.publish(&MSG_ENCODER);
        nh.spinOnce();
    }
    if (imuConnected) {
        IMUReadingToMsg();
    }

    // PCB LED Logic
    buttons::updateButtons();
    if (!callibrateSteering()) {
        if (servo_idle && !SW_EMERGENCY) {
            led::blinkLEDs();
        } else {
            if (SW_IDLE) {
                led::setLED(0, led::color_red);
            } else {
                led::setLED(0, led::color_green);
            }
            if (pwm_reader::REM_IDLE) {
                led::setLED(1, led::color_red);
            } else {
                led::setLED(1, led::color_green);
            }
            if (!pwm_reader::REM_OVERRIDE) {
                led::setLED(2, led::color_red);
            } else {
                led::setLED(2, led::color_green);
            }
            if (!SW_EMERGENCY) {
                led::setLED(3, led::color_red);
            } else {
                led::setLED(3, led::color_green);
            }
        }
    }
    led::updateLEDs();
}
