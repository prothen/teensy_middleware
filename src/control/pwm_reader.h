#ifndef SVEA_PWM_READER
#define SVEA_PWM_READER
#include "actuation_constants.h"
#include "utility.h"
#include <Arduino.h>
namespace pwm_reader {

/*! @file pwm_reader.h*/

/*!
 * @brief Duration (us) that decides when the remote should register as idle
 * If the registred pwm singal on channel 5 from the reciever is SHORTER than
 * this value (in microseconds), the remote is considered to be disconnected.
 *
 * The reciever will still send neutral steering and velocity even if the
 * remote is of, but channel 4 and 5 (used for differentials) turns of completely.
 * When the remote is connected again it will take approxiamtely 7 seconds until
 * the reciever starts sending on channel 5 again, and the remote can be used again.
 *
 * Some recievers instead sends a neutral (1500 micro seconds) pwm signal on chanel 5
 * if the remote is disconnected. In this case the remote is registred as idle if
 * the pulse length on channel 5 is within PWM_REM_IDLE_DETECT micro seconds of
 * the neutral value.
 */

/*
 * @defgroup PwmInputSettings PWM input settings variables
 */
/*@{*/

const uint8_t PWM_REM_IDLE_DETECT = 200; // micro seconds

//! Duration in micro seconds from last rising pwm edge to that the remote will be considered idle
const uint32_t REM_TIMEOUT = 50000; // micro seconds

//! Index of pwm pins
enum Pwm_index { steer_ind,
                 vel_ind,
                 gear_ind,
                 fdiff_ind,
                 rdiff_ind,
                 num_pwm_pins };

//! Expected minimum pwm duty cycle from the reciver (micro seconds)
const unsigned long PWM_IN_MIN_PW = 1010;
//! Expected maximum pwm duty cycle from the reciver (micro seconds)
const unsigned long PWM_IN_MAX_PW = 2000;

//! If the duty cycle are this much higher/lower than the max/min values someting is wrong with the hardware (micro seconds)
const unsigned long PWM_IN_ERROR_LIMIT = 50;

//! Time in micro seconds between the rising edge of a pwm signal and until the information is sent to ROS
const unsigned long PWM_IN_UPDATE_WAIT = PWM_IN_MAX_PW + 100; // micro seconds
/*@}*/

/*!
 * @defgroup PwmInputConstants PWM input constants
 * @{
 */
/*!
 * @defgroup ReceiverPwmPins Pins used connected the reciever to the Teensy
 * @ingroup PwmInputConstants
 * @{
 */
const uint8_t PWM_IN_STEER_PIN = 0; //!< D0,  Steering, connect to channel 1 on the receiver
const uint8_t PWM_IN_VELOC_PIN = 1; //!< D1,  Velocity, connect to channel 2 on the receiver
const uint8_t PWM_IN_GEAR_PIN = 2;  //!< D2, Transmission, connect to channel 3 on the receiver
const uint8_t PWM_IN_FDIFF_PIN = 3; //!< D3, Front differential, connect to channel 4 on the receiver
const uint8_t PWM_IN_RDIFF_PIN = 4; //!< D4, Rear differential, connect to channel 5 on the receiver
const uint8_t PWM_IN_PINS[5] = {
    PWM_IN_STEER_PIN,
    PWM_IN_VELOC_PIN,
    PWM_IN_GEAR_PIN,
    PWM_IN_FDIFF_PIN,
    PWM_IN_RDIFF_PIN};
/** @}*/ // End group ReceiverPwmPins

/*
 * Storage variables
 */

/*!
 * @defgroup ActuationValueStorage Actuation value storage
 * The order is Steering, velocity, gear, front differential, rear differential
 */
/*@{*/
//! Actuation values sent from the remote
// int8_t REM_ACTUATION[5] = {0,0,MSG_TO_ACT_OFF[0],MSG_TO_ACT_OFF[1],MSG_TO_ACT_OFF[2]};
/*@}*/

/*!
 * @defgroup PwmMeasurtement Reciever pwm duty cycle measurement variables
 */
/*@{*/
volatile long PWM_HIGH_TIME; //!< Time of the last rising edge in micro seconds
//! True if a rising edge has been observed since the latest values was sent to ROS
volatile bool PWM_HIGH_RECEIVED = false;
//!
volatile uint8_t PWM_BUFFER_IX = 0;
volatile uint8_t PREV_PWM_BUFFER_IX = 0;
//! Durations between the rising pwm edge and the falling edge for each pin
volatile uint32_t PWM_IN_DURATIONS[2][5];

/*@}*/

/*!
 * @defgroup PwmInStatusVariables Status variables
 */
/*@{*/
volatile bool REM_IDLE = true;      //!< True if the remote is considered idle
volatile bool REM_OVERRIDE = false; //!< True if the remote should override computer inputs
/*@}*/

/*!
 * @brief Set pwm buffer ix to zero
 */
inline void clearPwmBuffer(uint8_t ix) {
    ix = ix & 1;
    for (int i = 0; i < 5; i++) {
        PWM_IN_DURATIONS[ix][i] = 0;
    }
}

/*!
 * @brief Switch to the next pwm duration buffer and return the previous buffer index
 */
inline uint8_t switchPwmBuffer() {
    PREV_PWM_BUFFER_IX = PWM_BUFFER_IX;
    PWM_BUFFER_IX = PWM_BUFFER_IX ^ 1;
    clearPwmBuffer(PWM_BUFFER_IX);
    return PREV_PWM_BUFFER_IX;
}

/*!
 * Convert a pwm duration from the remote to an actuation value.
 * Returns -128 if the duration is longer or shorter than the
 *  PWM_IN_MAX_PW/PWM_IN_MIN_PW plus/minus PWM_IN_ERROR_LIMIT.
 * @param duration Duration of the high part of the pwm signal in micro seconds.
 */
int8_t pwmToActuation(unsigned long duration) {
    const static float actuation_scaling = 254.0 / (PWM_IN_MAX_PW - PWM_IN_MIN_PW);
    if (duration < PWM_IN_MIN_PW - PWM_IN_ERROR_LIMIT) {
        return -128;
    }
    if (duration > PWM_IN_MAX_PW + PWM_IN_ERROR_LIMIT) {
        return -128;
    }
    if (duration < PWM_IN_MIN_PW) {
        return ACTUATION_MIN;
    }
    if (duration > PWM_IN_MAX_PW) {
        return ACTUATION_MAX;
    }
    duration -= PWM_IN_MIN_PW;
    int8_t actuation = (duration * actuation_scaling - ACTUATION_MAX);
    return actuation;
}

/*!
 * @brief Handle pwm events
 *
 *  Reads all the values from #PWM_T_BUFFER and
 *  #PWM_S_BUFFER that has been added since the last loop
 *  and translates the pwm signal to actuation values.
 *  Calls pwmEvent() 2.2 ms after the last observed rising edge.
 *
 *  Button value directions:
 *  Ctrl  / Value
 *  Throttle pulled -> Possitve value
 *  Steering forward -> Possitive value
 *  Gear switch up -> 0 (Low gear)
 *  Gear switch down -> 1 (High Gear)
 *  Switch back -> forward diff 0 (unlocked), back diff 0 (unlocked)
 *  Switch middle -> forward diff 1 (locked), back diff 0 (unlocked)
 *  Switch middle -> forward diff 1 (locked), back diff 1 (locked) and REM_OVERRIDE = true
 */
bool processPwm(int8_t actuation_values[5]) {
    const static unsigned int pwm_middle = (PWM_IN_MIN_PW + PWM_IN_MAX_PW) * 0.5;
    /* If the duration since last rising edge is more than 2.2 ms:
     * Transmit recieved information to the computer
     * and allow new readings.
     */
    bool pwm_processed = false;
    if (PWM_HIGH_RECEIVED && micros() - PWM_HIGH_TIME > PWM_IN_UPDATE_WAIT) {
        noInterrupts();
        uint8_t buffer_ix = switchPwmBuffer();
        interrupts();
        unsigned long duration = PWM_IN_DURATIONS[buffer_ix][0]; // Steering
        actuation_values[0] = pwmToActuation(duration);
        duration = PWM_IN_DURATIONS[buffer_ix][1]; // Velocity
        actuation_values[1] = pwmToActuation(duration);
        duration = PWM_IN_DURATIONS[buffer_ix][2]; // Gear
        actuation_values[2] = duration < pwm_middle ? MSG_TO_ACT_ON[0] : MSG_TO_ACT_OFF[0];
        duration = PWM_IN_DURATIONS[buffer_ix][3]; // F. Diff
        actuation_values[3] = duration < pwm_middle ? MSG_TO_ACT_ON[1] : MSG_TO_ACT_OFF[1];
        duration = PWM_IN_DURATIONS[buffer_ix][4]; // R. Diff
        // Yes, the rear differential should be reversed
        actuation_values[4] = duration > pwm_middle ? MSG_TO_ACT_ON[2] : MSG_TO_ACT_OFF[2];
        // Check if the remote is disconnected
        // Some receivers will send a middle value on the
        // R. Diff. channel when no remote is connected.
        // Other receivers just holds the R. Diff channel low.
        unsigned long last_received_duration = abs_difference(duration, pwm_middle);
        bool durations_ok = true;
        for (int i = 0; i < 5; i++) {
            if (actuation_values[i] == -128) {
                durations_ok = false;
                break;
            }
        }
        if (duration < PWM_REM_IDLE_DETECT ||
            last_received_duration < PWM_REM_IDLE_DETECT ||
            durations_ok == false) {
            REM_IDLE = true;
            REM_OVERRIDE = false;
        } else {
            REM_IDLE = false;
            // Check if remote override is on by checking if channel 5
            // is in the rear differential lock active position.
            REM_OVERRIDE = (actuation_values[4] == MSG_TO_ACT_ON[2]);
        }
        PWM_HIGH_RECEIVED = false;
        pwm_processed = true;
    }
    return pwm_processed;
}

/*
 * INTERRUPT SERVICE ROUTINES
 */
void pwmIsrSteer(void) {
    bool pin_status = digitalRead(PWM_IN_STEER_PIN);
    if (pin_status) {
        PWM_HIGH_TIME = micros();
        PWM_HIGH_RECEIVED = true;
    } else {
        PWM_IN_DURATIONS[PWM_BUFFER_IX][0] = micros() - PWM_HIGH_TIME;
    }
}

template <int N>
void pwmIsrCommand(void) {
    bool pin_status = digitalReadFast(PWM_IN_PINS[N]);
    if (pin_status) {
        PWM_HIGH_TIME = micros();
        PWM_HIGH_RECEIVED = true;
    } else {
        PWM_IN_DURATIONS[PWM_BUFFER_IX][N] = micros() - PWM_HIGH_TIME;
    }
}

template <int N>
void pwmIsr(void) {
    PWM_IN_DURATIONS[PWM_BUFFER_IX][N] = micros() - PWM_HIGH_TIME;
}

void setup() {
    /* Enable pullups on pwm reading pins to avoid noise if reciever is not connected*/
    // Setup pwm input pins
    for (int i = 0; i < 5; i++) {
        pinMode(PWM_IN_PINS[i], INPUT_PULLUP);
    }
    /* Configure interrupts */
    attachInterrupt(digitalPinToInterrupt(PWM_IN_STEER_PIN), pwmIsr<0>, FALLING);
    attachInterrupt(digitalPinToInterrupt(PWM_IN_VELOC_PIN), pwmIsrCommand<1>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PWM_IN_GEAR_PIN), pwmIsr<2>, FALLING);
    attachInterrupt(digitalPinToInterrupt(PWM_IN_FDIFF_PIN), pwmIsr<3>, FALLING);
    attachInterrupt(digitalPinToInterrupt(PWM_IN_RDIFF_PIN), pwmIsr<4>, FALLING);
}

} // namespace pwm_reader
#endif /* SVEA_PWM_READER */
