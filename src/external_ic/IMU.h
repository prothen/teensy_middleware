#include <Arduino.h>
// Deps needed to interface with the IMU
#include "sensor_msgs/Imu.h"
#include "svea_teensy.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <utility/imumaths.h>
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
    return true;
}
// Im sorry, this is the way
uint32_t headerCntIMU = 0;
uint32_t headerCntMag = 0;
uint32_t headerCntTemp = 0;
void IMUReadingToMsg(ros::Time time, lli_imu_t &msg) {
    sensors_event_t event;
    bno.getEvent(&event);
    msg.header.seq = headerCntIMU++;
    msg.header.stamp = time;
    msg.header.frame_id = "imu";
    imu::Quaternion quat = bno.getQuat();
    msg.orientation.x = (float)quat.x();
    msg.orientation.y = (float)quat.y();
    msg.orientation.z = (float)quat.z();
    msg.orientation.w = (float)quat.w();
    imu::Vector<3> gyroVec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    msg.angular_velocity.x = (float)gyroVec.x();
    msg.angular_velocity.y = (float)gyroVec.y();
    msg.angular_velocity.z = (float)gyroVec.z();
    imu::Vector<3> accelVec = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    msg.linear_acceleration.x = (float)accelVec.x();
    msg.linear_acceleration.y = (float)accelVec.y();
    msg.linear_acceleration.z = (float)accelVec.z();
    float fakeCovariance = 0;
    for (int i = 0; i < 9; ++i) {
        msg.orientation_covariance[i] = fakeCovariance;
        msg.angular_velocity_covariance[i] = fakeCovariance;
        msg.linear_acceleration_covariance[i] = fakeCovariance;
    }
    Serial.printf("IMU: Orientation(%f, %f, %f, %f) Angular Velocity(%f, %f, %f) Linear Acceleration(%f, %f, %f)\n", msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
}
void MagReadingToMsg(ros::Time time, lli_mag_t &msg) {
    sensors_event_t event;
    bno.getEvent(&event);
    msg.header.seq = headerCntMag++;
    msg.header.stamp = time;
    msg.header.frame_id = "imu";
    msg.magnetic_field.x = (float)event.magnetic.x;
    msg.magnetic_field.y = (float)event.magnetic.y;
    msg.magnetic_field.z = (float)event.magnetic.z;
    float fakeCovariance = 0;
    for (int i = 0; i < 9; ++i) {
        msg.magnetic_field_covariance[i] = fakeCovariance;
    }
}
void TempReadingToMsg(ros::Time time, sensor_msgs::Temperature &msg) {
    msg.header.seq = headerCntTemp++;
    msg.header.stamp = time;
    msg.header.frame_id = "t";
    msg.temperature = (float)bno.getTemp();
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
