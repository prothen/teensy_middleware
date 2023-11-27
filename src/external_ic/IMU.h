#include <Arduino.h>

// Deps needed to interface with the IMU
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
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

std_msgs::Header header;

#define BNO055_SAMPLERATE_DELAY_MS (100)

namespace SVEA {

class IMU {
private:
    // Class variables for calibration
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
    int16_t mag_offset_x;
    int16_t mag_offset_y;
    int16_t mag_offset_z;
    uint8_t accel_radius;
    uint8_t mag_radius;
    
    Adafruit_BNO055 bno;

    SVEA::NodeHandle &nh;
    ros::Publisher imu_pub;
    ros::Publisher imu_mag;
    ros::Publisher imu_temp;

    std_msgs::Header header;
    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;
    sensor_msgs::Temperature temp_msg;

public:
    IMU(SVEA::NodeHandle &nh) : bno(55, 0x28, &Wire1),
                                nh(nh),
                                imu_pub("imu/data", &imu_msg),
                                imu_mag("imu/mag", &mag_msg),
                                imu_temp("imu/temp", &temp_msg) {
        nh.advertise(imu_pub);
        nh.advertise(imu_mag);
        nh.advertise(imu_temp);

        header.frame_id = "imu";
        header.seq = 0;
    };

    bool open() {
        bool succ = bno.begin(OPERATION_MODE_NDOF);
        if (succ) {
            bno.setExtCrystalUse(true);
            Serial.println("BN0055 detected");
        } else {
            Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        }
        return succ;
    }

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

        vec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu_msg.angular_velocity.x = vec.x();
        imu_msg.angular_velocity.y = vec.y();
        imu_msg.angular_velocity.z = vec.z();

        vec = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        mag_msg.magnetic_field.x = vec.x();
        mag_msg.magnetic_field.y = vec.y();
        mag_msg.magnetic_field.z = vec.z();

    // TODO, make more efficient or make sensible covarience, or both
    int fakeCovariance = 0;
    for (int i = 0; i < 9; ++i) {
        MSG_IMU.orientation_covariance[i] = fakeCovariance;
        MSG_IMU.angular_velocity_covariance[i] = fakeCovariance;
        MSG_IMU.linear_acceleration_covariance[i] = fakeCovariance;
        MSG_MAG.magnetic_field_covariance[i] = fakeCovariance;
    }

        imu_pub.publish(&imu_msg);
        imu_mag.publish(&mag_msg);
        imu_temp.publish(&temp_msg);
    }
};

} // namespace SVEA