#include <Arduino.h>

// Deps needed to interface with the IMU
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"

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
// ros::Time now,
void IMUReadingToMsg() {

    // Header stuff
    header.stamp = nh.now();
    header.seq++;

    MSG_IMU.header = header;
    MSG_MAG.header = header;
    MSG_TEMP.header = header;

    imu::Quaternion quat = bno.getQuat();
    MSG_IMU.orientation.x = quat.x();
    MSG_IMU.orientation.y = quat.y();
    MSG_IMU.orientation.z = quat.z();
    MSG_IMU.orientation.w = quat.w();
    imu::Vector<3> Vec;
    Vec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    MSG_IMU.angular_velocity.x = Vec.x();
    MSG_IMU.angular_velocity.y = Vec.y();
    MSG_IMU.angular_velocity.z = Vec.z();

    Vec = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    MSG_IMU.linear_acceleration.x = Vec.x();
    MSG_IMU.linear_acceleration.y = Vec.y();
    MSG_IMU.linear_acceleration.z = Vec.z();

    Vec = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    MSG_MAG.magnetic_field.x = Vec.x();
    MSG_MAG.magnetic_field.y = Vec.y();
    MSG_MAG.magnetic_field.z = Vec.z();

    MSG_TEMP.temperature = bno.getTemp();

    int fakeCovariance = 0;
    for (int i = 0; i < 9; ++i) {
        msg.orientation_covariance[i] = fakeCovariance;
        msg.angular_velocity_covariance[i] = fakeCovariance;
        msg.linear_acceleration_covariance[i] = fakeCovariance;
    }
    //nh.spinOnce();
    //Serial.println("MSG_IMU: ");
    imu_pub.publish(&MSG_IMU);
    nh.spinOnce();
    //Serial.println("MSG_MAG: ");
    imu_mag.publish(&MSG_MAG);
    nh.spinOnce();
    //Serial.println("MSG_TEMP: ");
    imu_temp.publish(&MSG_TEMP);
    nh.spinOnce();
    //Serial.printf("IMU: Orientation(%f, %f, %f, %f) Angular Velocity(%f, %f, %f) Linear Acceleration(%f, %f, %f)\n", msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);

    return;
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
