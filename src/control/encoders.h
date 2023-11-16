#ifndef SVEA_ENCODERS
#define SVEA_ENCODERS
#include <Arduino.h>
// #include "actuation_constants.h"
// #include "utility.h"

/*! @file svea_teensy.h*/

namespace encoders {

//! Sampling interval for the wheel encoders in micro seconds
const unsigned long ENCODER_SAMPLE_INTERVAL = 100000;

/*!
 * @defgroup EncoderPins Pins used connected the wheel encoders.
 */
/*@{*/
const uint8_t ENCODER_PIN_R1 = 22; // 22 Right wheel encoder tick pin 1
const uint8_t ENCODER_PIN_R2 = 23; // 23 Right wheel encoder tick pin 2
const uint8_t ENCODER_PIN_L1 = 20; // 20 Left wheel encoder tick pin 1
const uint8_t ENCODER_PIN_L2 = 21; // 21 Left wheel encoder tick pin 2
/*@}*/

/*!
 * @defgroup EncoderVariables Wheel encoder variables
 */
/*@{*/
volatile uint8_t ENC_TICK_COUNT[2][2] = {{0, 0}, {0, 0}}; //!< Right and left wheel tick count
volatile uint8_t ENC_BUFFER_IX = 0;                       //!< Encoder buffer index
const uint8_t RIGHT_ENC_IX = 0;
const uint8_t LEFT_ENC_IX = 1;
//! Time of last right and left tick event
volatile uint32_t ENC_TICK_TIME[2][2] = {{micros(), micros()}, {micros(), micros()}};
/*@}*/

/*!
 * @brief Detect ticks from the wheel encoders.
 * Detects changes in pin values of the encoder pins
 * and increments the tick counts in
 * ENC_TICK_COUNT accordingly.
 */
template <int IX>
void encoderIsr(void) {
    ENC_TICK_COUNT[ENC_BUFFER_IX][IX]++;
    ENC_TICK_TIME[ENC_BUFFER_IX][IX] = micros();
}

/*!
 * @brief Switch to the next pwm duration buffer and return the previous buffer index
 */
inline uint8_t switchEncoderBuffer() {
    ENC_BUFFER_IX = ENC_BUFFER_IX ^ 1;
    return ENC_BUFFER_IX ^ 1;
}

struct encoder_reading_t {
    uint8_t right_ticks = 0;
    uint8_t left_ticks = 0;
    uint32_t right_time_delta = 0;
    uint32_t left_time_delta = 0;
};

/*!
 * @brief Count the encoder ticks since last call and send the result to ROS
 *
 * The time_delta is given in micro second resolution.
 */
bool processEncoderTicks(encoder_reading_t &reading) {
    static uint32_t last_sent_time = micros();

    // Calculate and record duration
    uint32_t current_time = micros();
    uint32_t dt = current_time - last_sent_time;
    bool encoders_processed = false;
    if (dt > ENCODER_SAMPLE_INTERVAL) {
        noInterrupts(); // Temporarily disable interrupts on encoder pins
        uint8_t buffer_ix = switchEncoderBuffer();
        uint32_t previous_right_tick_time = ENC_TICK_TIME[ENC_BUFFER_IX][RIGHT_ENC_IX];
        uint32_t previous_left_tick_time = ENC_TICK_TIME[ENC_BUFFER_IX][LEFT_ENC_IX];
        interrupts(); // Enable interrupts on the encoder pins again
        // Calculate and record tick changes
        reading.right_ticks = ENC_TICK_COUNT[buffer_ix][RIGHT_ENC_IX];
        reading.left_ticks = ENC_TICK_COUNT[buffer_ix][LEFT_ENC_IX];
        // Update the times for the last ticks
        reading.right_time_delta = ENC_TICK_TIME[buffer_ix][RIGHT_ENC_IX] - previous_right_tick_time;
        reading.left_time_delta = ENC_TICK_TIME[buffer_ix][LEFT_ENC_IX] - previous_left_tick_time;
        if (reading.right_ticks == 0) {
            reading.right_time_delta = 0;
            ENC_TICK_TIME[buffer_ix][RIGHT_ENC_IX] = current_time - ENCODER_SAMPLE_INTERVAL;
        }
        if (reading.left_ticks == 0) {
            reading.left_time_delta = 0;
            ENC_TICK_TIME[buffer_ix][LEFT_ENC_IX] = current_time - ENCODER_SAMPLE_INTERVAL;
        }
        last_sent_time = current_time;
        ENC_TICK_COUNT[buffer_ix][RIGHT_ENC_IX] = 0;
        ENC_TICK_COUNT[buffer_ix][LEFT_ENC_IX] = 0;
        encoders_processed = true;
    }
    return encoders_processed;
}

void setup() {
    /* Same thing for the encoder pins */
    pinMode(ENCODER_PIN_R1, OUTPUT);
    pinMode(ENCODER_PIN_L1, OUTPUT);
    digitalWrite(ENCODER_PIN_R1, LOW);
    digitalWrite(ENCODER_PIN_L1, LOW);
    pinMode(ENCODER_PIN_R2, INPUT_PULLUP);
    // digitalWrite(ENCODER_PIN_R2, LOW);
    pinMode(ENCODER_PIN_L2, INPUT_PULLUP);
    // digitalWrite(ENCODER_PIN_L2, LOW);
    // Settings for pin change interrupts for detecting wheel encoder ticks
    attachInterrupt(ENCODER_PIN_R2, encoderIsr<RIGHT_ENC_IX>, CHANGE);
    attachInterrupt(ENCODER_PIN_L2, encoderIsr<LEFT_ENC_IX>, CHANGE);
}

} // namespace encoders
#endif // SVEA_ENCODERS
