#include "Adxl355.h"

Adxl355::Adxl355(int chipSelectPin) : _csPin(chipSelectPin) {};

Adxl355::~Adxl355() {
    stop();
}


void Adxl355::initSPI(SPIClass &spi) {
    spi_obj = &spi;
    if (spi_obj) {
        spi_obj->begin();
    }
}

int Adxl355::stop() {
}


// int getRawAccel(long *x, long *y, long *z) {
//     if ();
// }

void Adxl355::update() {
    uint8_t data[8];
    
}

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
    for (int i=2; i>=0; i--) {
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