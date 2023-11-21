#include <Arduino.h>
// Deps needed to interface with the IMU
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>
#include <utility/imumaths.h>

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include "sensor_msgs/Imu.h"

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

int headerCnt = 0;
// ros::Time now,
void IMUReadingToMsg(sensor_msgs::Imu &msg) {
    sensors_event_t event;
    bno.getEvent(&event);

    Serial.print(F("Orientation: "));
    Serial.print((float)event.orientation.x);
    msg.orientation.x = (float)event.orientation.x;
    Serial.print(F(" "));
    Serial.print((float)event.orientation.y);
    msg.orientation.y = (float)event.orientation.y;
    Serial.print(F(" "));
    Serial.print((float)event.orientation.z);
    Serial.println(F(""));

    float fakeCovariance = 0;
    for (int i = 0; i < 9; ++i) {
        msg.orientation_covariance[i] = fakeCovariance;
        msg.angular_velocity_covariance[i] = fakeCovariance;
        msg.linear_acceleration_covariance[i] = fakeCovariance;
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