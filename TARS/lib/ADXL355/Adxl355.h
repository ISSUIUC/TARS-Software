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
    static const uint8_t I2CSPEED_INTPOLARITY_RANGE = 0x2c;
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

    enum STATUS_VALUES { NVM_BUSY = 0x10, ACTIVITY = 0x08, FIFO_OVERRUN = 0x04, FIFO_FULL = 0x02, DATA_READY = 0x01 };

    enum RANGE_VALUES { RANGE_2G = 0x01, RANGE_4G = 0x02, RANGE_8G = 0x03, RANGE_MASK = 0x03 };

    enum HPF_CORNER {
        NOT_ENABLED = 0b000,
        ODR_X_2_47 = 0b001,
        ODR_X_62_084 = 0b010,
        ODR_X_15_545 = 0b011,
        ODR_X_3_862 = 0b100,
        ODR_X_0_954 = 0b101,
        ODR_X_0_238 = 0b110,
        HPF_CORNER_MASK = 0b01110000
    };

    enum ODR_LPF {
        ODR_4000_AND_1000 = 0b0000,
        ODR_2000_AND_500 = 0b0001,
        ODR_1000_AND_250 = 0b0010,
        ODR_500_AND_125 = 0b0011,
        ODR_250_AND_62_5 = 0b0100,
        ODR_125_AND_31_25 = 0b0101,
        ODR_62_5_AND_15_625 = 0b0110,
        ODR_31_25_AND_7_813 = 0b0111,
        ODR_15_625_AND_3_906 = 0b1000,
        ODR_7_813_AND_1_953 = 0b1001,
        ODR_3_906_AND_0_977 = 0b1010,
        ODR_LPF_MASK = 0b1111
    };

    enum POWER_CTL_VALUES {
        POWER_CTL_OFF = 0x01,
        POWER_CTL_ON = ~POWER_CTL_OFF,
        POWER_CTL_TEMP_OFF = 0x02,
        POWER_CTL_TEMP_ON = ~POWER_CTL_TEMP_OFF
    };

    SPIClass *spi_obj = NULL;

    uint8_t read8(uint8_t reg);
    uint16_t read16(uint8_t reg);
    uint8_t readBlock(uint8_t reg, uint8_t length, uint8_t *output);
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

    bool isDeviceRecognized();
    bool isRunning();
    void initializeSensor(RANGE_VALUES range, ODR_LPF odr_lpf);
    void setRange(RANGE_VALUES value);
    void setIntMap(uint8_t value);
    int getFIFOCount();
    STATUS_VALUES getStatus();
    bool isDataReady();
    bool isFIFOFull();
    bool isFIFOOverrun();
    bool isTempSensorOn();
    void startTempSensor();
    void stopTempSensor();
    double getTempC();
    double getTempF();
    ODR_LPF getOdrLpf();
    void setOdrLpf(ODR_LPF odr_lpf);
    bool isRunning();
    RANGE_VALUES getRange();
    void setRange(RANGE_VALUES range_value);
    long twosComplement(unsigned long value);
    int getRawAxis(long *x, long *y, long *z);
    void calibrateSensor(int fifoReadCount);
    void setTrim(int32_t x, int32_t y, int32_t z);
    int readFIFOEntries(long *output);
};

#endif  // _ADXL355_H_