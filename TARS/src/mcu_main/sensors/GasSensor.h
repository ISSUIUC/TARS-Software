#pragma once

#include <Adafruit_BME680.h>

class GasSensor {
public:
    GasSensor();

    void init();
    void refresh();

    float readTemperature();

private:
    Adafruit_BME680 bme;
};