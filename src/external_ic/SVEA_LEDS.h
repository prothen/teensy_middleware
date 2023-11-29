//#include <Arduino.h>
//#include <FastLED.h>
//
//namespace SVEA {
//enum LED_CONNECTOR {
//    INTERNAL,
//    J6,
//    J7,
//    J8
//};
//
//class SVEA_Leds {
//private:
//    // Default Values
//    uint8_t LED_CH_1 = 1; // Out pins on GPIO extender
//    uint8_t LED_CH_2 = 2;
//    uint8_t NUM_LEDS = 4;
//    //CRGB leds[NUM_LEDS]; // Update the size of the array based on NUM_LEDS
//    ESPIChipsets chipset = SK9822;
//
//public:
//    SVEA_Leds(LED_CONNECTOR connector) {
//        switch (connector) {
//        case LED_CONNECTOR::INTERNAL:
//            // These are the defaults
//            break;
//        case LED_CONNECTOR::J6:
//            break;
//        case LED_CONNECTOR::J7:
//            break;
//        case LED_CONNECTOR::J8:
//            break;
//        }
//        //FastLED.addLeds<SK9822, 6, GRB>(leds, NUM_LEDS); // Update the addLeds function call with the correct arguments
//    };
//};
//} // namespace SVEA