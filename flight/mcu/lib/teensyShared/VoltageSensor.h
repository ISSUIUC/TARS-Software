#ifndef VOLTAGESENSOR_H
#define VOLTAGESENSOR_H

#include "Arduino.h"
#include "ChRt.h"

#include "packet.h"

class VoltageSensor {
public:
    MUTEX_DECL(mutex);

    explicit VoltageSensor(HardwareSerial& serial) : serial(serial) {
        serial.begin(115200);
        serial.setTimeout(3);
    };

    VoltageData read();

    float getVoltage(char src) const {
        switch (src) {
            case 'B': return v_battery;
            case '1': return v_servo1;
            case '2': return v_servo2;
            case '3': return v_3_3;
            case '5': return v_5;
            case '9': return v_9;
            default: return -1.0;
        }
    }

private:
    /**
     * @param voltage_src
     * '1' = servo1 line
     * '2' = servo2 line
     * '3' = 3.3 volt line
     * '5' = 5 volt line
     * '9' = 9 volt line
     * 'B' = battery line
     */
    float read_voltage(char voltage_src) {
        // reading the voltage actually takes a long time, so skip reading
        // voltages we don't really need
        if (voltage_src != 'B') return 0;
        serial.print(voltage_src);
        int val = 0;
        int read_num = static_cast<int>(serial.readBytes((char*)&val, sizeof(val)));
        if (read_num != sizeof(val)) {
            return -1;
        }

        // analog_read reads 10 bits with 0 = 0 volts, 1024 = 3.3 volts;
        return static_cast<float>(val) * 3.3f / 1024;
    }

    HardwareSerial& serial;
    float v_battery = 0.0;
    float v_servo1 = 0.0;
    float v_servo2 = 0.0;
    float v_3_3 = 0.0;
    float v_5 = 0.0;
    float v_9 = 0.0;
    systime_t timestamp = 0;
};

#endif  // VOLTAGESENSOR_H
