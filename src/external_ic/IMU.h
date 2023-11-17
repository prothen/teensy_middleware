// Deps needed to interface with the IMU
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Math functions used to interpret IMU values
#include "matrix.h"
#include "quaternion.h"
#include "vector.h"

#include <ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include "sensor_msgs/Imu.h"



#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55); //, BNO055_ADDRESS, &Wire1);

bool setupIMU() {
    if (!bno.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        return false;
    }
    bno.setExtCrystalUse(false);
}
int headerCnt = 0;
void IMUReadingToMsg(ros::Time now, sensor_msgs::Imu &msg) {
    
    msg.header.frame_id = "imu";
    msg.header.stamp = now; //TODO
    msg.header.seq = headerCnt++;
    msg.orientation.x = bno.getQuat().x();
    msg.orientation.y = bno.getQuat().y();
    msg.orientation.z = bno.getQuat().z();
    msg.orientation.w = bno.getQuat().w();

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    msg.angular_velocity.x = gyro.x();
    msg.angular_velocity.y = gyro.y();
    msg.angular_velocity.z = gyro.z();

    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    msg.linear_acceleration.x = accel.x();
    msg.linear_acceleration.y = accel.y();
    msg.linear_acceleration.z = accel.z();

    float fakeCovariance = 0;
    for (int i = 0; i < 9; ++i) {
        msg.orientation_covariance[i] = fakeCovariance;
        msg.angular_velocity_covariance[i] = fakeCovariance;
        msg.linear_acceleration_covariance[i] = fakeCovariance;
    }
}