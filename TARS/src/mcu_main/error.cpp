#include "mcu_main/error.h"

#include <ChRt.h>

#include "mcu_main/pins.h"

void handleError(ErrorCode code) {
    switch (code) {
        case NO_ERROR:
            return;
        case CANNOT_CONNECT_KX134_CS:
            Serial.println("Failed to communicate with KX134. Stalling Program");
            digitalWrite(LED_RED, HIGH);
            while (true) { }
        case CANNOT_INIT_KX134_CS:
            Serial.println("Could not initialize KX134. Stalling Program");
            digitalWrite(LED_BLUE, HIGH);
            while (true) { }
        case CANNOT_CONNECT_ZOEM8Q0:
            Serial.println("Failed to communicate with ZOEM8Q0 gps. Stalling Program");
            while (true) { }
        case CANNOT_CONECT_LSM9DS1:
            Serial.println("Failed to communicate with LSM9DS1 gps. Stalling Program");
            digitalWrite(LED_ORANGE, HIGH);
            while (true) { }
        case SD_BEGIN_FAILED:
            digitalWrite(LED_RED, HIGH);
            digitalWrite(LED_ORANGE, HIGH);
            digitalWrite(LED_BLUE, HIGH);
            Serial.println("SD Begin Failed. Stalling Program");
            while (true) {
                digitalWrite(LED_RED, HIGH);
                delay(100);
                digitalWrite(LED_RED, LOW);
                delay(100);
            }
        case RADIO_INIT_FAILED:
            while (true) {
                Serial.println("Radio Initialization Failed");
            }
        case RADIO_SET_FREQUENCY_FAILED:
            while (true) {
                Serial.println("[ERROR]: Default setFrequency Failed");
            }
    }

}

