#ifndef VOLTAGESENSOR_H
#define VOLTAGESENSOR_H

#include<Arduino.h>

struct VoltageData {
    float v_battery;
    float v_servo1;
    float v_servo2;
    float v_3_3;
    float v_5;
    float v_9;
    systime_t timestamp;
};

class VoltageSensor {
public:
    VoltageSensor(HardwareSerial& serial): serial(serial){
        serial.begin(115200);
        serial.setTimeout(3);
    };
    VoltageData read(){
        return {
            .v_battery = read_voltage('B'),
            .v_servo1 = read_voltage('1'),
            .v_servo2 = read_voltage('2'),
            .v_3_3 = read_voltage('3'),
            .v_5 = read_voltage('5'),
            .v_9 = read_voltage('9'),
            .timestamp = chVTGetSystemTime()
        };
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
    float read_voltage(char voltage_src){
        if(voltage_src != 'B') return 0;
        serial.print(voltage_src);
        int val = 0;
        int read_num = serial.readBytes((char*)&val, sizeof(val));
        if(read_num != sizeof(val)) {
            return -1;
        }

        // analog read reads 10 bits with 0 = 0 volts, 1024 = 3.3 volts;
        return val * 3.3 / 1024;
    }

    HardwareSerial& serial;
};

#endif //VOLTAGESENSOR_H