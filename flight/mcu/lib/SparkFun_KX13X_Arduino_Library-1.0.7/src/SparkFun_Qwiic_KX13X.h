/******************************************************************************
SparkFun_Qwiic_KX13X.h
Elias Santistevan @ SparkFun Electronics
Original Creation Date: March 2021

This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __SparkFun_Qwiic_KX13X_H__
#define __SparkFun_Qwiic_KX13X_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SPI.h>
#include <Wire.h>

#define KX13X_DEFAULT_ADDRESS 0x1F
#define KX13X_ALT_ADDRESS 0x1E

#define KX132_WHO_AM_I 0x3D
#define KX134_WHO_AM_I 0x46
#define TOTAL_ACCEL_DATA_16BIT 6
#define TOTAL_ACCEL_DATA_8BIT 3
#define MAX_BUFFER_LENGTH 32

#define XLSB 0
#define XMSB 1
#define YLSB 2
#define YMSB 3
#define ZLSB 4
#define ZMSB 5

#define SPI_READ 0x80   // OR'ed at most sig BIT with register address
#define SPI_WRITE 0x00  // OR'ed at most sig BIT with register address

#define DEFAULT_SETTINGS 0xC0
#define INT_SETTINGS 0xE0
#define SOFT_INT_SETTINGS 0xE1
#define BUFFER_SETTINGS 0xE2
#define TILT_SETTINGS 0xE3

#define COTR_DEF_STATE 0x55
#define COTR_POS_STATE 0xAA

#define BUFFER_16BIT_SAMPLES 0x01
#define BUFFER_8BIT_SAMPLES 0x00
#define BUFFER_MODE_FIFO 0x00
#define BUFFER_MODE_STREAM 0x01
#define BUFFER_MODE_TRIGGER 0x02

struct outputData {
    float xData;
    float yData;
    float zData;
};

struct rawOutputData {
    int16_t xData;
    int16_t yData;
    int16_t zData;
};

// Accelerometer Data
enum KX13X_REGISTERS {

    KX13X_MAN_ID = 0x00,  //      Retuns "KION" in ASCII
    KX13X_PART_ID,        //            Who am I + Silicon specific ID
    KX13X_XADP_L,         //     ------- X, Y, and Z - High and Low bytes -----
    KX13X_XADP_H,
    KX13X_YADP_L,
    KX13X_YADP_H,
    KX13X_ZADP_L,
    KX13X_ZADP_H,
    KX13X_XOUT_L,
    KX13X_XOUT_H,
    KX13X_YOUT_L,
    KX13X_YOUT_H,
    KX13X_ZOUT_L,
    KX13X_ZOUT_H,  //     --------------^^--------------------------
    //                          0x0E - 0x11 Reserved
    KX13X_COTR = 0x12,  //        Command Test Register
    KX13X_WHO_AM_I,     //          Who am I: 0x3D-KX132, 0x46-KX134
    KXI3X_TSCP,         //        -------Tilt Register----------------------
    KX13X_TSPP,         //        -----------^^-----------------------------
    KX13X_INS1,         //        -------Interrupt Registers ---------------
    KX13X_INS2,
    KX13X_INS3,
    KX13X_STATUS_REG,
    KX13X_INT_REL,  //    ------------^^----------------------------
    KX13X_CNTL1,    //       --------Control Registers-----------------
    KX13X_CNTL2,
    KX13X_CNTL3,
    KX13X_CNTL4,
    KX13X_CNTL5,
    KX13X_CNTL6,  //        -------------^^---------------------------
    KX13X_ODCNTL,
    KX13X_INC1,  // Controls settings for INT1
    KX13X_INC2,  // Defines behavior for Wake-Up Function and Back To Sleep
    KX13X_INC3,  // Defines which axes can cause a tap based interrupt
    KX13X_INC4,  // Controls which function triggers INT1
    KX13X_INC5,
    KX13X_INC6,  // Controls which function triggers INT2
    // 0x28 Reserved
    KX13X_TILT_TIMER = 0x29,
    KX13X_TDTRC,  // Tap Control Regs -----
    KX13X_TDTC,
    KX13X_TTH,
    KX13X_TTL,
    KX13X_FTD,
    KX13X_STD,
    KX13X_TLT,
    KX13X_TWS,
    KX13X_FFTH,
    KX13X_FFC,
    KX13X_FFCNTL,
    // 0x35 - 0x36 Reserved
    KX13X_TILT_ANGLE_LL = 0x37,
    KX13X_TILT_ANGLE_HL,
    KX13X_HYST_SET,
    KX13X_LP_CNTL1,
    KX13X_LP_CNTL2,
    // 0x3C - 0x48 Reserved
    KX13X_WUFTH = 0x49,
    KX13X_BTSWUFTH,
    KX13X_BTSTH,
    KX13X_BTSC,
    KX13X_WUFC,
    // 0x4E - 0x5C Reserved
    KX13X_SELF_TEST = 0x5D,
    KX13X_BUF_CNTL1,
    KX13X_BUF_CNTL2,
    KX13X_BUF_STATUS_1,
    KX13X_BUF_STATUS_2,
    KX13X_BUF_CLEAR,
    KX13X_BUF_READ,
    KX13X_ADP_CNTL1,
    KX13X_ADP_CNTL2,
    KX13X_ADP_CNTL3,
    KX13X_ADP_CNTL4,
    KX13X_ADP_CNTL5,
    KX13X_ADP_CNTL6,
    KX13X_ADP_CNTL7,
    KX13X_ADP_CNTL8,
    KX13X_ADP_CNTL9,
    KX13X_ADP_CNTL10,
    KX13X_ADP_CNTL11,
    KX13X_ADP_CNTL12,
    KX13X_ADP_CNTL13,
    KX13X_ADP_CNTL14,
    KX13X_ADP_CNTL15,
    KX13X_ADP_CNTL16,
    KX13X_ADP_CNTL17,
    KX13X_ADP_CNTL18,
    KX13X_ADP_CNTL19
    // Reserved 0x77 - 0x7F
};

typedef enum {

    KX13X_SUCCESS = 0x00,
    KX13X_GENERAL_ERROR,
    KX13X_I2C_ERROR,

} KX13X_STATUS_t;

enum HARDWARE_INTERRUPT {

    HI_TILT_POSITION = 0x01,
    HI_WAKE_UP = 0x02,
    HI_TAP_DOUBLE_TAP = 0x04,
    HI_BACK_TO_SLEEP = 0x08,
    HI_DATA_READY = 0x10,
    HI_WATERMARK = 0x20,
    HI_BUFFER_FULL = 0x40,
    HI_FREEFALL = 0x80

};

class QwiicKX13xCore {
   public:
    QwiicKX13xCore();

    uint8_t beginCore(uint8_t, TwoWire &wirePort);
    uint8_t beginSPICore(uint8_t, uint32_t, SPIClass &spiPort);
    bool initialize(uint8_t settings = DEFAULT_SETTINGS);
    bool runCommandTest();
    bool accelControl(bool);
    uint8_t readAccelState();
    bool setRange(uint8_t);
    bool setOutputDataRate(uint8_t);
    float readOutputDataRate();
    bool setInterruptPin(bool enable, uint8_t polarity = 0,
                         uint8_t pulseWidth = 0, bool latchControl = false);
    bool routeHardwareInterrupt(uint8_t, uint8_t pin = 1);
    bool clearInterrupt();
    bool dataTrigger();
    bool setBufferThreshold(uint8_t);
    bool setBufferOperation(uint8_t, uint8_t);
    bool enableBuffer(bool, bool);

    bool getRawAccelData(rawOutputData *);

    KX13X_STATUS_t readRegister(uint8_t *, uint8_t);
    KX13X_STATUS_t writeRegister(uint8_t, uint8_t, uint8_t, uint8_t);
    KX13X_STATUS_t readMultipleRegisters(uint8_t, uint8_t dataBuffer[],
                                         uint16_t);
    KX13X_STATUS_t overBufLenI2CRead(uint8_t, uint8_t dataBuffer[], int16_t);

    // CPOL and CPHA are demonstrated on pg 25 of Specification Data sheet
    // CPOL = 0, CPHA = 0 SPI_MODE0
    SPISettings kxSPISettings;
    rawOutputData rawAccelData;
    outputData userAccel;

   private:
    TwoWire *_i2cPort;   // The generic connection to user's chosen I2C hardware
    SPIClass *_spiPort;  // The generic connection to user's chosen SPI hardware

    uint8_t _deviceAddress;
    uint32_t _spiPortSpeed;  // max port speed is 10MHz
    uint8_t _cs;
};

class QwiicKX132 : public QwiicKX13xCore {
   public:
    QwiicKX132();
    bool begin(uint8_t kxAddress = KX13X_DEFAULT_ADDRESS,
               TwoWire &i2cPort = Wire);
    bool beginSPI(uint8_t, uint32_t spiPortSpeed = 10000000,
                  SPIClass &spiPort = SPI);
    outputData getAccelData();
    bool convAccelData(outputData *, rawOutputData *);

   private:
    const double convRange2G = .00006103518784142582;
    const double convRange4G = .0001220703756828516;
    const double convRange8G = .0002441407513657033;
    const double convRange16G = .0004882811975463118;

#define KX132_RANGE2G 0x00
#define KX132_RANGE4G 0x01
#define KX132_RANGE8G 0x02
#define KX132_RANGE16G 0x03
};

class QwiicKX134 : public QwiicKX13xCore {
   public:
    QwiicKX134();
    bool begin(uint8_t kxAddress = KX13X_DEFAULT_ADDRESS,
               TwoWire &i2cPort = Wire);
    bool beginSPI(uint8_t, uint32_t spiPortSpeed = 10000000,
                  SPIClass &spiPort = SPI);
    outputData getAccelData();
    bool convAccelData(outputData *, rawOutputData *);

   private:
    const double convRange8G = .000244140751365703299;
    const double convRange16G = .000488281197546311838;
    const double convRange32G = .000976523950926236762;
    const double convRange64G = .001953125095370342112;

#define KX134_RANGE8G 0x00
#define KX134_RANGE16G 0x01
#define KX134_RANGE32G 0x02
#define KX134_RANGE64G 0x03
};

#endif /* QWIIC_KX13X */
