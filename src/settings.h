/*! @file settings.h*/
#ifndef SVEA_LLI_SETTINGS
#define SVEA_LLI_SETTINGS

#define DURATION_DEBUG 0

// //! Sampling interval for the wheel encoders in micro seconds
// const unsigned long ENCODER_SAMPLE_INTERVAL = 25000;

//! Baud rate for serial transmisions
#if DURATION_DEBUG
const uint32_t SERIAL_BAUD_RATE = 115200;
#else
const uint32_t SERIAL_BAUD_RATE = 250000;
//const uint32_t SERIAL_BAUD_RATE = 115200;
#endif

/*
 * Software and remote state constants
 */
const unsigned long SW_TIMEOUT = 200; //!< Duration (ms) from last recieved computer
                                      //!< message when the computer will count as idle

// THIS IS VERY STUPID AND SHOULD BE REMOVED, but cant be removed
// Because ROS
//! Maximum number of ROS subscribers
const uint8_t MAX_ROS_SUBSCRIBERS = 50;
//! Maximum number of ROS publishers
const uint8_t MAX_ROS_PUBLISHERS = 50;
//! Maximum number of ROS subscribers
const uint16_t ROS_IN_BUFFER_SIZE = 500;
const uint16_t ROS_OUT_BUFFER_SIZE = 500;
#endif /* SVEA_LLI_SETTINGS */
