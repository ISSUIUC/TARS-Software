#include "Arduino.h"
#include <SPI.h>

#ifndef _ADXL355_H_
#define _ADXL355_H_

class Adxl355 {
   private:
    // 8 byte wide registers

    // READ / WRITE REGISTERS
    static const int READ_BYTE = 0x01;
    static const int WRITE_BYTE = 0x00;

    // Device Specific Registers
    static const uint8_t DEVID_AD = 0x00;
    static const uint8_t DEVID_MST = 0x01;
    static const uint8_t PARTID = 0x02;
    static const uint8_t REVID = 0x03;
    static const uint8_t STATUS = 0x04;

    // FIFO (First In First Out) Sample Registers
    static const uint8_t FIFO_ENTRIES = 0x05;

    // Interrupt Pin Function Map Register
    static const uint8_t INT_MAP = 0x2a;

    // Only using SPI
    // static const uint8_t I2CSPEED_INTPOLARITY_RANGE = 0x2c;
    static const uint8_t POWER_CTL = 0x2d;
    static const uint8_t RESET = 0x2f;
    static const uint8_t RESET_VALUE = 0x52;

    // Temperature Registers
    static const uint8_t TEMP2 = 0x06;
    static const uint8_t TEMP1 = 0x07;

    // Acceleration Data Registers
    static const uint8_t XDATA3 = 0x08;
    static const uint8_t XDATA2 = 0x09;
    static const uint8_t XDATA1 = 0x0a;
    static const uint8_t YDATA3 = 0x0b;
    static const uint8_t YDATA2 = 0x0c;
    static const uint8_t YDATA1 = 0x0d;
    static const uint8_t ZDATA3 = 0x0e;
    static const uint8_t ZDATA2 = 0x0f;
    static const uint8_t ZDATA1 = 0x10;

    // FIFO Register
    static const uint8_t FIFO_DATA = 0x11;

    // Offset Registers
    static const uint8_t OFFSET_X_H = 0x1e;
    static const uint8_t OFFSET_X_L = 0x1f;
    static const uint8_t OFFSET_Y_H = 0x20;
    static const uint8_t OFFSET_Y_L = 0x21;
    static const uint8_t OFFSET_Z_H = 0x22;
    static const uint8_t OFFSET_Z_L = 0x23;

    // Activity Registers
    static const uint8_t ACT_EN = 0x24;
    static const uint8_t ACT_THRESH_H = 0x25;
    static const uint8_t ACT_THRESH_X_L = 0x26;
    static const uint8_t ACT_COUNT = 0x27;

    // Filter Setting Register
    static const uint8_t FILTER = 0x28;

    enum POWER_CTL_VALUES {
        POWER_CTL_OFF = 0x01,
        POWER_CTL_ON = ~POWER_CTL_OFF,
        POWER_CTL_TEMP_OFF = 0x02,
        POWER_CTL_TEMP_ON = ~POWER_CTL_TEMP_OFF
    };



    SPIClass *spi_obj = NULL;

    uint8_t read8(uint8_t reg);
    uint16_t read16(uint8_t reg);
    uint32_t readBlock(uint8_t reg, uint8_t length, uint8_t *output);
    void write8(uint8_t reg, uint8_t value);

    int _csPin;
    int16_t _x_accel;
    int16_t _y_accel;
    int16_t _z_accel;
    int16_t _temp;

   public:
    Adxl355(int chipSelectPin);
    ~Adxl355();

    void initSPI(SPIClass &spi);
    int start();
    int stop();
    void update();
    int getRawAccel(long *x, long *y, long *z);
};

#endif  // _ADXL355_H_