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
    if (isRunning()) {
        return;
    }

    uint8_t work = read8(FILTER);

    work = (work & ~(ODR_LPF::ODR_LPF_MASK)) | ((int)value);

    write8(FILTER, work);
}

// Set up the Adxl355 with our required values
void Adxl355::initializeSensor(RANGE_VALUES range, ODR_LPF odr_lpf) {
    setRange(Adxl355::RANGE_VALUES::RANGE_2G);

    Adxl355::RANGE_VALUES rangeValue = getRange();

    // Set the ODR and LPF
    setOdrLpf(odr_lpf);

    // Set the interrupt to FIFO FULL on INT1
    setIntMap(0x01);
}