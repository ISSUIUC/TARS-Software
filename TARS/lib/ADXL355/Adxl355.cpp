#include "Adxl355.h"

Adxl355::Adxl355(int chipSelectPin) : _csPin(chipSelectPin){};

Adxl355::~Adxl355() {
    if (isRunning()) {
        stop();
    }
}

void Adxl355::initSPI(SPIClass &spi) {
    spi_obj = &spi;
    if (spi_obj) {
        spi_obj->begin();
    }
}

// Puts Sensor in Standby Mode
int Adxl355::stop() {
    int power = read8(POWER_CTL);

    if (!(power & POWER_CTL_VALUES::POWER_CTL_OFF)) {
        power = power | (int)POWER_CTL_VALUES::POWER_CTL_OFF;
        write8(POWER_CTL, power);
    }

    return power;
}

int Adxl355::start() {
    int result = 0;

    if (!isDeviceRecognized()) {
        result = -1;
    } else {
        uint8_t power = read8(POWER_CTL);

        if (power & POWER_CTL_VALUES::POWER_CTL_OFF) {
            power = power & (int)POWER_CTL_VALUES::POWER_CTL_ON;
            write8(POWER_CTL, power);
        }
    }

    return result;
}

bool Adxl355::isDeviceRecognized() {
    uint16_t check = read16(0x01);
    return (check == 0x1ded);
}

bool Adxl355::isRunning() {
    bool check = false;
    int data = read8(POWER_CTL);

    check = (data & POWER_CTL_VALUES::POWER_CTL_OFF) ? false : true;

    return check;
}

void Adxl355::update() { uint8_t data[8]; }

uint8_t Adxl355::read8(uint8_t reg) {
    uint8_t value = 0;

    uint8_t registerToSend = (reg << 1) | READ_BYTE;

    digitalWrite(_csPin, LOW);
    spi_obj->transfer(registerToSend);
    value = spi_obj->transfer(0x00);
    digitalWrite(_csPin, HIGH);
    return value;
}

uint16_t Adxl355::read16(uint8_t reg) {
    uint16_t value = 0;
    uint8_t byte = 0;

    uint8_t registerToSend = (reg << 1) | READ_BYTE;
    digitalWrite(_csPin, LOW);

    spi_obj->transfer(registerToSend);
    for (int i = 2; i >= 0; i--) {
        byte = spi_obj->transfer(0x00);
        value = value |= (byte << (i * 8));
    }

    digitalWrite(_csPin, HIGH);

    return value;
}

uint8_t Adxl355::readBlock(uint8_t reg, uint8_t length, uint8_t *output) {
    uint8_t registerToSend = (reg << 1) | READ_BYTE;

    digitalWrite(_csPin, LOW);
    spi_obj->transfer(registerToSend);

    int i = length;

    while (i) {
        *output++ = spi_obj->transfer(0x00);
        i--;
    }

    digitalWrite(_csPin, HIGH);

    return length - i;
}

void Adxl355::write8(uint8_t reg, uint8_t value) {
    uint8_t registerToSend = (reg << 1) | WRITE_BYTE;

    digitalWrite(_csPin, LOW);
    spi_obj->transfer(registerToSend);
    spi_obj->transfer(value);
    digitalWrite(_csPin, HIGH);
}

void Adxl355::setIntMap(uint8_t value) { write8(INT_MAP, value); }

int Adxl355::getFIFOCount() {
    uint8_t data = read8(FIFO_ENTRIES);

    return data;
}

Adxl355::STATUS_VALUES Adxl355::getStatus() {
    uint8_t data = read8(STATUS);

    return (STATUS_VALUES)data;
}

bool Adxl355::isDataReady() {
    STATUS_VALUES data = getStatus();

    return (data & STATUS_VALUES::DATA_READY) ? true : false;
}

bool Adxl355::isFIFOFull() {
    STATUS_VALUES data = getStatus();

    return (data & STATUS_VALUES::FIFO_FULL) ? true : false;
}

bool Adxl355::isFIFOOverrun() {
    STATUS_VALUES data = getStatus();

    return (data & STATUS_VALUES::FIFO_OVERRUN) ? true : false;
}

bool Adxl355::isTempSensorOn() {
    bool result = false;

    uint8_t work = read8(POWER_CTL);

    result = ((work & POWER_CTL_VALUES::POWER_CTL_OFF) || (work & POWER_CTL_VALUES::POWER_CTL_TEMP_OFF)) ? false : true;

    return result;
}

void Adxl355::startTempSensor() {
    uint8_t data = read8(POWER_CTL);

    if (data & POWER_CTL_VALUES::POWER_CTL_TEMP_OFF) {
        data = data & (int)POWER_CTL_VALUES::POWER_CTL_TEMP_ON;
        write8(POWER_CTL, data);
    }
}

void Adxl355::stopTempSensor() {
    uint8_t data = read8(POWER_CTL);

    if (!(data & POWER_CTL_VALUES::POWER_CTL_TEMP_OFF)) {
        data = data | (int)POWER_CTL_VALUES::POWER_CTL_TEMP_OFF;
        write8(POWER_CTL, data);
    }
}

double Adxl355::getTempC() {
    uint16_t value = read16(TEMP2);
    double temp = ((double)(1852 - value)) / 9.05 + 19.21;

    return temp;
}

double Adxl355::getTempF() {
    double result = getTempC();

    return result * 9 / 5 + 32;
}

Adxl355::RANGE_VALUES Adxl355::getRange() {
    int range = read8(I2CSPEED_INTPOLARITY_RANGE);

    return (RANGE_VALUES)(range & RANGE_VALUES::RANGE_MASK);
}

void Adxl355::setRange(RANGE_VALUES value) {
    // Cannot setRange while running the sensor
    if (isRunning()) {
        return;
    }

    uint8_t range = read8(I2CSPEED_INTPOLARITY_RANGE);

    range &= ~(RANGE_VALUES::RANGE_MASK);
    range |= (int)value;

    write8(I2CSPEED_INTPOLARITY_RANGE, range);
}

bool Adxl355::isRunning() {
    bool result = false;
    int work = read8(POWER_CTL);

    result = (work & POWER_CTL_VALUES::POWER_CTL_OFF) ? false : true;

    return result;
}

Adxl355::ODR_LPF Adxl355::getOdrLpf() {
    uint8_t work = read8(FILTER);

    return (ODR_LPF)(work & ODR_LPF::ODR_LPF_MASK);
}

void Adxl355::setOdrLpf(ODR_LPF value) {
    // Cannot setOdrLpf while running the sensor
    if (isRunning()) {
        return;
    }

    uint8_t work = read8(FILTER);

    work = (work & ~(ODR_LPF::ODR_LPF_MASK)) | ((int)value);

    write8(FILTER, work);
}

void Adxl355::initializeSensor(RANGE_VALUES range, ODR_LPF odr_lpf) {
    setRange(Adxl355::RANGE_VALUES::RANGE_2G);

    Adxl355::RANGE_VALUES rangeValue = getRange();

    // Set the ODR and LPF
    setOdrLpf(odr_lpf);

    // Set the interrupt to FIFO FULL on INT1
    setIntMap(0x01);
}

long Adxl355::twosComplement(unsigned long value) {
    // If the most significant bit is set we negate the value
    value = -(value & (1 << (20 - 1))) + (value & ~(1 << (20 - 1)));

    return value;
}

// Convert raw data to signed integer value
int Adxl355::getRawAxis(long *x, long *y, long *z) {
    uint8_t output[9];
    // Fills memory with 0's
    memset(output, 0, 9);

    // Reads 9 bytes
    int result = readBlock(XDATA3, 9, (uint8_t *)output);

    unsigned long value_x = 0;
    unsigned long value_y = 0;
    unsigned long value_z = 0;

    // Check if all bytes were successfully read into
    if (result == 9) {
        // Converts into 20 bit value for each axis
        value_x = (output[0] << 12) | (output[1] << 4) | (output[2] >> 4);
        value_y = (output[3] << 12) | (output[4] << 4) | (output[5] >> 4);
        value_z = (output[6] << 12) | (output[7] << 4) | (output[8] >> 4);
    }

    // Binary to

    *x = twosComplement(value_x);
    *y = twosComplement(value_y);
    *z = twosComplement(value_z);

    return result;
}

void Adxl355::setTrim(int32_t x, int32_t y, int32_t z) {
    // Cannot setTrim while running the sensor
    if (isRunning()) {
        return;
    }

    int16_t value_x = (x >> 4);
    int16_t value_y = (y >> 4);
    int16_t value_z = (z >> 4);

    uint8_t high_x = (value_x & 0xff00) >> 8;
    uint8_t low_x = value_x & 0x00ff;
    uint8_t high_y = (value_y & 0xff00) >> 8;
    uint8_t low_y = value_y & 0x00ff;
    uint8_t high_z = (value_z & 0xff00) >> 8;
    uint8_t low_z = value_z & 0x00ff;

    write8(OFFSET_X_H, high_x);
    write8(OFFSET_X_L, low_x);
    write8(OFFSET_Y_H, high_y);
    write8(OFFSET_Y_L, low_y);
    write8(OFFSET_Z_H, high_z);
    write8(OFFSET_Z_L, low_z);
}

int Adxl355::readFIFOEntries(long *output) {
    int fifoCount = getFIFOCount();
    uint8_t data[9];
    memset(data, 0, 9);

    unsigned long work[3];

    for (int i = 0; i < fifoCount / 3; i++) {
        int result = readBlock(FIFO_DATA, 9, (uint8_t *)data);

        if (result > 0) {
            for (int j = 0; j < 9; j += 3) {
                work[j / 3] = (data[0 + j] << 12) | (data[1 + j] << 4) | (data[2 + j] >> 4);
                output[i * 3 + j / 3] = twosComplement(work[j / 3]);
            }
        } else {
            return -1;
        }
    }

    return fifoCount / 3;
}

void Adxl355::calibrateSensor(int fifoReadCount) {
    long FIFOOut[32][3];
    int result;
    int readings = 0;

    long total_x = 0;
    long total_y = 0;
    long total_z = 0;

    // Fill FIFOOut with 0s in memory
    memset(FIFOOut, 0, sizeof(FIFOOut));

    stop();

    setTrim(0, 0, 0);

    start();

    for (int j = 0; j < fifoReadCount; j++) {
        if (-1 != (result = readFIFOEntries((long *)FIFOOut))) {
            readings += result;

            for (int i = 0; i < result; i++) {
                total_x += FIFOOut[i][0];
                total_y += FIFOOut[i][1];
                total_z += FIFOOut[i][2];
            }
        } else {
            // Failed the read
            // Error out
        }
    }

    long avg_x = total_x / readings;
    long avg_y = total_y / readings;
    long avg_z = total_z / readings;

    stop();

    // Set new trim and start
    setTrim(avg_x, avg_y, avg_z);
    start();
}
