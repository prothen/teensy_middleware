#include <Arduino.h>
// Deps needed to interface with the IMU
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>
#include <utility/imumaths.h>

#include "ros/duration.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include "sensor_msgs/Imu.h"

#include "svea_teensy.h"

std_msgs::Header header;

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

bool setupIMU() {
    if (!bno.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        return false;
    }
    bno.setExtCrystalUse(true);
    Serial.println("BN0055 detected");

    header.frame_id = "imu";
    header.seq = 0;

    return true;
}

// ros::Time now,
void IMUReadingToMsg() {

    // Header stuff
    header.stamp = nh.now();
    header.seq++;

    MSG_IMU.header = header;    
    MSG_MAG.header = header;
    MSG_TEMP.header = header;

    imu::Quaternion quat = bno.getQuat();
    MSG_IMU.orientation.x = (float)quat.x();
    MSG_IMU.orientation.y = (float)quat.y();
    MSG_IMU.orientation.z = (float)quat.z();
    MSG_IMU.orientation.w = (float)quat.w();

    imu::Vector<3> gyroVec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    MSG_IMU.angular_velocity.x = (float)gyroVec.x();
    MSG_IMU.angular_velocity.y = (float)gyroVec.y();
    MSG_IMU.angular_velocity.z = (float)gyroVec.z();

    imu::Vector<3> accelVec = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    MSG_IMU.linear_acceleration.x = (float)accelVec.x();
    MSG_IMU.linear_acceleration.y = (float)accelVec.y();
    MSG_IMU.linear_acceleration.z = (float)accelVec.z();

    imu::Vector<3> magVec = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    MSG_MAG.magnetic_field.x = (float)magVec.x();
    MSG_MAG.magnetic_field.y = (float)magVec.y();
    MSG_MAG.magnetic_field.z = (float)magVec.z();

    MSG_TEMP.temperature = (float)bno.getTemp();

    Serial.print("IMU: (");
    Serial.print(MSG_IMU.orientation.x);
    Serial.print(", ");
    Serial.print(MSG_IMU.orientation.y);
    Serial.print(", ");
    Serial.print(MSG_IMU.orientation.z);
    Serial.print(", ");
    Serial.print(MSG_IMU.orientation.w);
    Serial.print(")  Angular Velocity: (");
    Serial.print(MSG_IMU.angular_velocity.x);
    Serial.print(", ");
    Serial.print(MSG_IMU.angular_velocity.y);
    Serial.print(", ");
    Serial.print(MSG_IMU.angular_velocity.z);
    Serial.print(")  Linear Acceleration: (");
    Serial.print(MSG_IMU.linear_acceleration.x);
    Serial.print(", ");
    Serial.print(MSG_IMU.linear_acceleration.y);
    Serial.print(", ");
    Serial.print(MSG_IMU.linear_acceleration.z);
    Serial.print(")  Magnetic Field: (");
    Serial.print(MSG_MAG.magnetic_field.x);
    Serial.print(", ");
    Serial.print(MSG_MAG.magnetic_field.y);
    Serial.print(", ");
    Serial.print(MSG_MAG.magnetic_field.z);
    Serial.print(")  Temperature: ");
    Serial.println(MSG_TEMP.temperature);
    
    float fakeCovariance = 0;
    for (int i = 0; i < 9; ++i) {
        MSG_IMU.angular_velocity_covariance[i] = fakeCovariance;
        MSG_IMU.linear_acceleration_covariance[i] = fakeCovariance;
        MSG_MAG.magnetic_field_covariance[i] = fakeCovariance;
    }
}

void IMU_DEBUG() {
    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    /* Board layout:
           +----------+
           |         *| RST   PITCH  ROLL  HEADING
       ADR |*        *| SCL
       INT |*        *| SDA     ^            /->
       PS1 |*        *| GND     |            |
       PS0 |*        *| 3VO     Y    Z-->    \-X
           |         *| VIN
           +----------+
    */

    /* The processing sketch expects data as roll, pitch, heading */
    Serial.print(F("Orientation: "));
    Serial.print((float)event.orientation.x);
    Serial.print(F(" "));
    Serial.print((float)event.orientation.y);
    Serial.print(F(" "));
    Serial.print((float)event.orientation.z);
    Serial.println(F(""));
}