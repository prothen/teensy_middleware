#include <Arduino.h>
#include <string.h>

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

#define BNO055_SAMPLERATE_DELAY_MS (100)

namespace SVEA {
class IMU {
private:
    bool hasCalibrated = false;
    bool hasWrittenToEEPROM = false; // Safety bool, dont change

    bool magCalibrated = false; // Magnometer is more succeptible to interference, so it needs to be calibrated more often

    uint8_t calData[22];

    uint8_t startByte = 100; // Teensy has 1080 bytes of EEPROM, this is for saving some calib data to eeprom

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

    void loadCalibration() {
        if (EEPROM.read(startByte) != 0x55) {
            Serial.println("No calibration data found in EEPROM");
            return; // Check for a flag value to see if we have saved data
        }
        uint8_t calData[22];
        for (int i = startByte; i < 22; i++) {
            calData[i] = EEPROM.read(i + 1);
        }
        Serial.println("Found calibration data in EEPROM");
        bno.setSensorOffsets(calData);
        hasCalibrated = true;
    }
    void saveCalibration() {
        // if (!hasCalibrated) {
        //     Serial.println("No calibration data to save, already calibrated from saved values");
        //     return;
        // }
        //if (!bno.getSensorOffsets(calData)) {
        //    serialPrintCalibStatus();
        //    return;
        //}

        // VERY DANGEROUS, DONT WRITE TOO MUCH TO THE EEPROM, please keep the "safety if"
        // this makes it so one only writes to the EEPROM once per power cycle
        if (!hasWrittenToEEPROM) {
            Serial.println("Writing to EEPROM");
            EEPROM.write(startByte, 0x55); // Write a flag value to indicate that cal data follows
            for (int i = startByte; i < 22; i++) {
                EEPROM.write(i + 1, calData[i]);
            }
            hasWrittenToEEPROM = true;
        }
        loadCalibration();
    }
    void writeMagCalibration() {
        if (magCalibrated) {
            return;
        }
        bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_MAGONLY);

        if (!bno.getSensorOffsets(calData)) {
            serialPrintCalibStatus();
            return;
        }
        calData[16] = mag_offset_z;
        calData[17] = accel_radius;
        calData[18] = mag_radius;
        bno.setSensorOffsets(calData);
        magCalibrated = true;
        bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);
        Serial.println("Mag Calibrated");
    }
    void serialPrintCalibStatus() {
        // Print the calibration status for each sensor
        uint8_t system, gyro, accel, mag;
        system = gyro = accel = mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);

        Serial.print("Sys=");
        Serial.print(system, DEC);
        Serial.print(" Gyro=");
        Serial.print(gyro, DEC);
        Serial.print(" Accel=");
        Serial.print(accel, DEC);
        Serial.print(" Mag=");
        Serial.println(mag, DEC);
        return;
    }
    void debugPrint(){
       
    
    }
public:
    IMU(SVEA::NodeHandle &nh) : bno(55, 0x28, &Wire1),
                                nh(nh),
                                imu_pub("imu/data", &imu_msg),
                                imu_mag("imu/mag", &mag_msg),
                                imu_temp("imu/temp", &temp_msg) {
        nh.advertise(imu_pub);
        nh.advertise(imu_mag);
        nh.advertise(imu_temp);

        header.frame_id = "imu-uncalibrated";
        header.seq = 0;
    }

    bool open() {
        bool succ = bno.begin();
        if (succ) {
            bno.setExtCrystalUse(true);
            Serial.println("BNO055 detected");
        } else {
            Serial.print("Oops, no BNO055 detected ... Check your wiring or I2C ADDR!");
            header.frame_id = "imu-disconnected";
        }
        return succ;
    }

    void update() {
        // Header stuff
        header.stamp = nh.now();
        header.seq++;

        imu_msg.header = header;
        mag_msg.header = header;
        temp_msg.header = header;

        imu::Quaternion quat = bno.getQuat();
        imu_msg.orientation.x = quat.x();
        imu_msg.orientation.y = quat.y();
        imu_msg.orientation.z = quat.z();
        imu_msg.orientation.w = quat.w();

        imu::Vector<3> vec;
        vec = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu_msg.linear_acceleration.x = vec.x();
        imu_msg.linear_acceleration.y = vec.y();
        imu_msg.linear_acceleration.z = vec.z();

        vec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu_msg.angular_velocity.x = vec.x();
        imu_msg.angular_velocity.y = vec.y();
        imu_msg.angular_velocity.z = vec.z();

        vec = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        mag_msg.magnetic_field.x = vec.x();
        mag_msg.magnetic_field.y = vec.y();
        mag_msg.magnetic_field.z = vec.z();

        temp_msg.temperature = bno.getTemp();
        // TODO, make more efficient or make sensible covariance, or both
        int fakeCovariance = 0;
        for (int i = 0; i < 9; ++i) {
            imu_msg.orientation_covariance[i] = fakeCovariance;
            imu_msg.angular_velocity_covariance[i] = fakeCovariance;
            imu_msg.linear_acceleration_covariance[i] = fakeCovariance;
            mag_msg.magnetic_field_covariance[i] = fakeCovariance;
        }

        imu_pub.publish(&imu_msg);
        imu_mag.publish(&mag_msg);
        imu_temp.publish(&temp_msg);
        //saveCalibration();
        writeMagCalibration();
    }
};
} // namespace SVEA