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
Adafruit_BNO055 bno = Adafruit_BNO055(0xA0); 

bool setupIMU() {
    if (!bno.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        return false;
    }
    Serial.println("BN0055 detected");

    //bno.setExtCrystalUse(false);

}
int headerCnt = 0;
//ros::Time now, 
void IMUReadingToMsg(sensor_msgs::Imu &msg) {
    Serial.println("IMUReadingToMsg");
    //msg.header.frame_id = "imu";
    //msg.header.stamp = now; //TODO
    //msg.header.seq = headerCnt++;
    
    auto qx = bno.getQuat().x();
    auto qy = bno.getQuat().y();
    auto qz = bno.getQuat().z();
    auto qw = bno.getQuat().w();
    Serial.println(String(qx) + " " + String(qy) + " " + String(qz) + " " + String(qw));
    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;
    Serial.println("IMUReadingToMsg2");
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    Serial.println("IMUReadingToMsg3");
    Serial.println(String(gyro.x()) + " " + String(gyro.y()) + " " + String(gyro.z()));
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

void IMU_DEBUG(){
    Serial.println(String(bno.getQuat().x()));
}