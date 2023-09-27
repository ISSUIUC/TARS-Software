
/**
 * If an error during initialization was detected, some combination of the blue, green, and orange LEDs will be on,
 * and the red LED will flash at some frequency. The Blue, Green, and Orange LEDs tells you what sensor is errored.
 *
 *
 * GREEN => KX134
 * GREEN, BLUE => GPS
 * ORANGE => SD
 * RED, BLUE, ORANGE => RADIO
 * BLUE => MAGNETOMETER
 * BLUE, ORANGE => BNO,
 * NONE (and RED is flashing) => Something is fucked, call whoever wrote this code
 *
 * RED, 10 Hz => Connection Error
 * RED, 2 Hz => Initialization Error
 */

#include "mcu_main/error.h"
#include "mcu_main/Rt.h"
#include "mcu_main/pins.h"

void handleError(ErrorCode code) {
    switch (code) {
        case NO_ERROR:
            return;
        case CANNOT_CONNECT_KX134_CS:
            digitalWrite(LED_GREEN, HIGH);
            while (true) {
                Serial.println("Failed to communicate with KX134. Stalling Program");
                digitalWrite(LED_RED, HIGH);
                delay(100);
                digitalWrite(LED_RED, LOW);
                delay(100);
            }
        case CANNOT_INIT_KX134_CS:
            digitalWrite(LED_GREEN, HIGH);
            while (true) {
                Serial.println("Could not initialize KX134. Stalling Program");
                digitalWrite(LED_RED, HIGH);
                delay(500);
                digitalWrite(LED_RED, LOW);
                delay(100);
            }
        case CANNOT_CONNECT_GPS:
            digitalWrite(LED_GREEN, HIGH);
            digitalWrite(LED_BLUE, HIGH);
            while (true) {
                Serial.println("Failed to communicate with ZOEM8Q0 gps. Stalling Program");
                digitalWrite(LED_RED, HIGH);
                delay(100);
                digitalWrite(LED_RED, LOW);
                delay(100);
            }
        case CANNOT_CONNECT_LSM9DS1:
            digitalWrite(LED_GREEN, HIGH);
            digitalWrite(LED_BLUE, HIGH);
            while (true) {
                Serial.println("Failed to communicate with ZOEM8Q0 gps. Stalling Program");
                digitalWrite(LED_RED, HIGH);
                delay(500);
                digitalWrite(LED_RED, LOW);
                delay(500);
            }
        case SD_BEGIN_FAILED:
            digitalWrite(LED_ORANGE, HIGH);
            while (true) {
                Serial.println("SD Begin Failed. Stalling Program");
                digitalWrite(LED_RED, HIGH);
                delay(500);
                digitalWrite(LED_RED, LOW);
                delay(500);
            }
        case RADIO_INIT_FAILED:
            digitalWrite(LED_RED, HIGH);
            digitalWrite(LED_ORANGE, HIGH);
            digitalWrite(LED_BLUE, HIGH);
            while (true) {
                Serial.println("Radio Initialization Failed");
                digitalWrite(LED_RED, HIGH);
                delay(100);
                digitalWrite(LED_RED, LOW);
                delay(100);
            }
        case RADIO_SET_FREQUENCY_FAILED:
            digitalWrite(LED_RED, HIGH);
            digitalWrite(LED_ORANGE, HIGH);
            digitalWrite(LED_BLUE, HIGH);
            while (true) {
                Serial.println("[ERROR]: Default setFrequency Failed");
                digitalWrite(LED_RED, HIGH);
                delay(500);
                digitalWrite(LED_RED, LOW);
                delay(500);
            }
        case CANNOT_CONNECT_MAGNETOMETER:
            digitalWrite(LED_BLUE, HIGH);
            while (true) {
                Serial.println("[ERROR]: Magnetometer connection failed");
                digitalWrite(LED_RED, HIGH);
                delay(100);
                digitalWrite(LED_RED, LOW);
                delay(100);
            }
        case CANNOT_CONNECT_BNO:
            digitalWrite(LED_ORANGE, HIGH);
            digitalWrite(LED_BLUE, HIGH);
            while (true) {
                Serial.println("[ERROR]: BNO086 connection failed");
                digitalWrite(LED_RED, HIGH);
                delay(500);
                digitalWrite(LED_RED, LOW);
                delay(500);
            }
        case CANNOT_INIT_BNO:
            digitalWrite(LED_ORANGE, HIGH);
            digitalWrite(LED_BLUE, HIGH);
            while (true) {
                Serial.println("[ERROR]: BNO086 initialization failed");
                digitalWrite(LED_RED, HIGH);
                delay(100);
                digitalWrite(LED_RED, LOW);
                delay(100);
            }

        default:
            while (true) {
                Serial.println("Unknown error");
                digitalWrite(LED_RED, HIGH);
                delay(100);
                digitalWrite(LED_RED, LOW);
                delay(100);
            }
    }
}