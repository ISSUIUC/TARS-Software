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
    int work = read8(POWER_CTL);

    check = (work & POWER_CTL_VALUES::POWER_CTL_OFF) ? false : true;

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
    uint8_t work = read8(FIFO_ENTRIES);

    return work;
}

Adxl355::STATUS_VALUES Adxl355::getStatus() {
    uint8_t work = read8(STATUS);

    return (STATUS_VALUES)work;
}

bool Adxl355::isDataReady() {
    STATUS_VALUES work = getStatus();

    return (work & STATUS_VALUES::DATA_READY) ? true : false;
}

bool Adxl355::isFIFOFull() {
    STATUS_VALUES work = getStatus();

    return (work & STATUS_VALUES::FIFO_FULL) ? true : false;
}

bool Adxl355::isFIFOOverrun() {
    STATUS_VALUES work = getStatus();

    return (work & STATUS_VALUES::FIFO_OVERRUN) ? true : false;
}
