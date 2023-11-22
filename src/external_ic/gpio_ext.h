#include <Adafruit_MCP23X08.h>
#include <Arduino.h>

/* GPIO extender variables */
constexpr int GPIO_ADDRESS = 0x20;
constexpr uint8_t SERVO_PWR_ENABLE_PIN = 3;
Adafruit_MCP23X08 gpio_extender;

void setup_gpio() {
    gpio_extender.begin_I2C(GPIO_ADDRESS, &Wire1);
    gpio_extender.pinMode(SERVO_PWR_ENABLE_PIN, OUTPUT);
    buttons::setup(gpio_extender);
    led::setup(gpio_extender);
}