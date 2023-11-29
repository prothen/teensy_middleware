#include <Arduino.h>
#include <Adafruit_MCP23X08.h>
#include <SPI.h>
// APA102 LEDs
//  see https://cpldcpu.wordpress.com/2014/08/27/apa102/
namespace led {

const uint8_t LED_CH_1 = 1; // Out pins on GPIO extender
const uint8_t LED_CH_2 = 2;
const uint32_t blink_interval = 200;  // ms
const uint32_t update_interval = 100; // ms
const uint32_t data_rate = 4000000;
const uint8_t global_brightness = 1 | (0b11100000); // 5 bit value, 0 to 31, but beigins with 111
const uint8_t start_frame[4] = {0, 0, 0, 0};        // start with 32 zeros
const uint8_t end_frame[4] = {0, 0, 0, 0};          // start with 32 zeros
const uint8_t frame_len = 4;
const uint8_t on_frame[4] = {global_brightness, 255, 0, 50}; // Global, R, B, G
const uint8_t off_frame[4] = {global_brightness, 0, 0, 0};
const uint16_t num_leds = 4;
uint8_t retbuff[4 * num_leds]; // Dummy return buffer
Adafruit_MCP23X08 gpio_extender;
SPISettings settingsLED(data_rate, MSBFIRST, SPI_MODE0);

struct irgb_t {
    uint8_t intensity = global_brightness;
    uint8_t r = 0;
    uint8_t b = 0;
    uint8_t g = 0;
};

const irgb_t color_off = {.intensity = global_brightness,
                          .r = 0,
                          .b = 0,
                          .g = 0};
const irgb_t color_red = {.intensity = global_brightness,
                          .r = 255,
                          .b = 0,
                          .g = 0};
const irgb_t color_blue = {.intensity = global_brightness,
                           .r = 0,
                           .b = 255,
                           .g = 0};
const irgb_t color_green = {.intensity = global_brightness,
                            .r = 0,
                            .b = 0,
                            .g = 255};
const irgb_t color_white = {.intensity = global_brightness,
                            .r = 255,
                            .b = 255,
                            .g = 255};
const irgb_t color_purple = {.intensity = global_brightness,
                             .r = 255,
                             .b = 255,
                             .g = 0};
const irgb_t color_orange = {.intensity = global_brightness,
                             .r = 255,
                             .b = 0,
                             .g = 50};
const irgb_t color_yelow = {.intensity = global_brightness,
                            .r = 255,
                            .b = 0,
                            .g = 100};

irgb_t LED_COLOR_VALUES[num_leds];

void set_led_channel(uint8_t channel) {
    uint8_t state_1 = channel & 1;        // first bit
    uint8_t state_2 = (channel & 2) >> 1; // second bit

    gpio_extender.digitalWrite(LED_CH_1, state_1);
    gpio_extender.digitalWrite(LED_CH_2, state_2);
    // gpio_extender->forceUpdate();
    return;
}

void setLED(uint8_t led_index, irgb_t colors) {
    if (led_index < num_leds) {
        LED_COLOR_VALUES[led_index] = colors;
    }
}

void setLEDs(irgb_t colors) {
    for (int i = 0; i < num_leds; i++) {
        setLED(i, colors);
    }
}

void pushLEDs(irgb_t color) {
    for (int i = num_leds - 1; i > 0; i--) {
        LED_COLOR_VALUES[i] = LED_COLOR_VALUES[i - 1];
    }
    setLED(0, color);
}

void blinkLEDs() {
    static uint32_t last_change = millis();
    static int led_status = 0; // false = off, true = on
    uint32_t current_time = millis();
    if (current_time - last_change > blink_interval) {
        last_change = current_time;
        irgb_t color;
        if (led_status == 0) {
            digitalWrite(LED_BUILTIN, LOW);
            color = color_green;
            led_status++;
        } else if (led_status == 1) {
            digitalWrite(LED_BUILTIN, HIGH);
            color = color_blue;
            led_status++;
        } else if (led_status == 2) {
            digitalWrite(LED_BUILTIN, LOW);
            color = color_red;
            led_status++;
        } else if (led_status == 3) {
            digitalWrite(LED_BUILTIN, HIGH);
            color = color_purple;
            led_status = 0;
        }
        // color.intensity = 255;
        pushLEDs(color);
    }
}

void updateLEDs(bool force = false) {
    static uint32_t last_change = millis();
    uint32_t current_time = millis();
    if (current_time - last_change > update_interval || force) {
        last_change = current_time;
        SPI1.beginTransaction(settingsLED);
        SPI1.transfer(start_frame, retbuff, frame_len);
        const uint8_t *out_frame = (uint8_t *)LED_COLOR_VALUES;
        SPI1.transfer(out_frame, retbuff, num_leds * frame_len);
        SPI1.transfer(end_frame, retbuff, frame_len);
        SPI.endTransaction();
    }
}

void setup(Adafruit_MCP23X08 &_gpio_extender) {
    SPI1.begin(); // The LEDs are connected to SPI1, on pin 26 and 27
                  // gpio_extender = &_gpio_extender;
    gpio_extender.begin_I2C();
    gpio_extender.pinMode(LED_CH_1, OUTPUT);
    gpio_extender.pinMode(LED_CH_2, OUTPUT);
    set_led_channel(0);
    updateLEDs(true);
}

} // namespace led
void ledLogic(bool is_calibrating, bool all_idle) {
    // LED logic

}