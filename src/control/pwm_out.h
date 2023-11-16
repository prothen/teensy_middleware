#ifndef SVEA_LLI
#define SVEA_LLI
#include <Arduino.h>
#include "settings.h"
#include "utility.h"
#include "svea_msgs/lli_ctrl.h"
#include "svea_msgs/lli_encoder.h"

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
/*!
 * @brief The minimum value that can be sent to the pwm board
 * If adjustPwmFrequency() is called this value is only used temporarily.
 */
const unsigned long PWM_OUT_MIN_TICK = PWM_OUT_MIN_PW * PWM_OUT_FREQUENCY * PWM_OUT_RES;
/*!
 * @brief The maximum 12-bit value that can be sent to the pwm board
 * If adjustPwmFrequency() is called this value is only used temporarily.
 */
const unsigned long PWM_OUT_MAX_TICK = PWM_OUT_MAX_PW * PWM_OUT_FREQUENCY * PWM_OUT_RES;
//! The 12-bit value corresponding to a neutral duty cycle
unsigned int PWM_OUT_NEUTRAL_TICK = (PWM_OUT_MIN_TICK + PWM_OUT_MAX_TICK) * 0.5;
//! The scaling factor between actuation values and the 12-bit values sent to the pwm board.
float INPUT_SCALE = (PWM_OUT_MAX_PW - PWM_OUT_NEUTRAL_TICK) / (float)ACTUATION_MIN;
/** @}*/

/*!
 * @defgroup PwmInputConstants PWM input constants
 * @{
 */
/*!
 * @defgroup ReceiverPwmPins Pins used connected the reciever to the Teensy
 * @ingroup PwmInputConstants
 * @{
 */
const uint8_t PWM_IN_STEER_PIN = 0; //!< D8,  Steering, connect to channel 1 on the receiver
const uint8_t PWM_IN_VELOC_PIN = 1; //!< D9,  Velocity, connect to channel 2 on the receiver
const uint8_t PWM_IN_GEAR_PIN = 2;  //!< D10, Transmission, connect to channel 3 on the receiver
const uint8_t PWM_IN_FDIFF_PIN = 3; //!< D11, Front differential, connect to channel 4 on the receiver
const uint8_t PWM_IN_RDIFF_PIN = 4; //!< D12, Rear differential, connect to channel 5 on the receiver
const uint8_t PWM_IN_PINS[5] = {
    PWM_IN_STEER_PIN,
    PWM_IN_VELOC_PIN,
    PWM_IN_GEAR_PIN,
    PWM_IN_FDIFF_PIN,
    PWM_IN_RDIFF_PIN};
/** @}*/ // End group ReceiverPwmPins

/*
 * Input PWM Buffer constants
 */
const uint8_t PWM_BUFFER_SIZE = 8;               //!< PWM buffer size, should be a power of 2
const uint8_t PWM_IX_MASK = PWM_BUFFER_SIZE - 1; //!< Modulo mask for buffer index

//! Pin mask used for falling edge detection
const uint8_t PWM_IN_PIN_MASK =
    bit(PWM_IN_STEER_PIN) | bit(PWM_IN_VELOC_PIN) | bit(PWM_IN_GEAR_PIN) | bit(PWM_IN_FDIFF_PIN) | bit(PWM_IN_RDIFF_PIN);
/*@}*/

/*!
 * @defgroup EncoderPins Pins used connected the wheel encoders.
 * Have to be in the range of PORTD (pin D0 to D7) and D0 to D3 are reserved.
 */
/*@{*/
const uint8_t ENCODER_PIN_R = 22; // 23 //!< D22,  Right wheel encoder tick pin
const uint8_t ENCODER_PIN_L = 20; // 21 //!< D20,  Left wheel encoder tick pin
/*@}*/

/*
 * Message type definitions and related constants
 */
typedef svea_msgs::lli_ctrl lli_ctrl_in;    //!< Message type for incomming messages
typedef svea_msgs::lli_ctrl lli_ctrl_out;   //!< Message type for outgoing messages'
typedef svea_msgs::lli_encoder lli_encoder; //!< Message type for encoder messages

/*!
 * @defgroup MsgBitPositions Bit positions used for the trans_diff_ctrl field in messages
 */
/*@{*/
const uint8_t GEAR_BIT = 0;  //!< Bit used for gear value (0 unlocked, 1 locked)
const uint8_t FDIFF_BIT = 1; //!< Bit used for front differential value (0 unlocked, 1 locked)
const uint8_t RDIFF_BIT = 2; //!< Bit used for rear differential value (0 unlocked, 1 locked)
//! Vector with the bit postitions in msg.gear_diff in order: gear, front diff, rear diff
const uint8_t ACT_BITS[3] = {GEAR_BIT, FDIFF_BIT, RDIFF_BIT};
//! Bit indicating if the GEAR_BIT value should be read from incoming messages
const uint8_t ENABLE_GEARCHANGE_BIT = 3;
//! Bit indicating if the front differential values should be read from incoming messages
const uint8_t ENABLE_FDIFCHANGE_BIT = 4;
//! Bit indicating if the rear differential values should be read from incoming messages
const uint8_t ENABLE_RDIFCHANGE_BIT = 5;
//! Vector with the enable change bits in order: gear, front diff, rear diff
const uint8_t ENABLE_ACT_CHANGE_BITS[3] = {ENABLE_GEARCHANGE_BIT, ENABLE_FDIFCHANGE_BIT, ENABLE_RDIFCHANGE_BIT};

//! Value that should be actuated if the corresponding bit in msg.gear_diff is not set
const int8_t MSG_TO_ACT_OFF[3] = {ACTUATION_MIN, ACTUATION_MIN, ACTUATION_MAX};
//! Value that should be actuated if the corresponding bit in msg.gear_diff is not set
const int8_t MSG_TO_ACT_ON[3] = {ACTUATION_MAX, ACTUATION_MAX, ACTUATION_MIN};
/*@}*/

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
//! Actuation values sent from the remote
int8_t REM_ACTUATION[5] = {0, 0, MSG_TO_ACT_OFF[0], MSG_TO_ACT_OFF[1], MSG_TO_ACT_OFF[2]};
//! Actuation values sent when both the remote and the computer are idle
//  Should correspond to gear in neutral and both differentials unlocked.
const int8_t IDLE_ACTUATION[5] = {0, 0, 0, MSG_TO_ACT_OFF[1], MSG_TO_ACT_OFF[2]};
/*@}*/

/* Function definitions */
inline void setPwmDriver(uint8_t channel, int8_t in_value);
void actuate(const int8_t actuation_values[]);

/*@}*/
#endif /* SVEA_LLI */
