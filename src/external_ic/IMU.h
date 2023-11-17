// Deps needed to interface with the IMU
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Math functions used to interpret IMU values
#include "matrix.h"
#include "quaternion.h"
#include "vector.h"

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include "sensor_msgs/Imu.h"


