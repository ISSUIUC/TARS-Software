/* rfm95.cpp
 *
 * HopeRF RFM95W device driver for TARS-MK1 (wrapping spidev)
 *
 * Illinois Space Society - IREC 2021 Avioinics Team
 *
 * Ayberk Yaraneri
 *
 */

#include "rfm95.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

RFM95::RFM95() {

    _spi_open();

    /* Set to LoRa mode */
    RFM_write(REG_OP_MODE, LORA_MODE);

    /* Set power registers */
    RFM_write(REG_OCP, OCP_SETTING);
    RFM_write(REG_PA_DAC, PA_DAC_SETTING);
    RFM_write(REG_PA_CONFIG, PA_CFG_SETTING);

    /* Set FIFO base pointers */
    RFM_write(REG_FIFO_RX_BASE_ADDR, 0x00);
    RFM_write(REG_FIFO_TX_BASE_ADDR, 0x80);
}

void RFM95::_spi_open() {

    int ret;

    _fd = open(_device, O_RDWR);
	if (_fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(_fd, SPI_IOC_WR_MODE, &_mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(_fd, SPI_IOC_RD_MODE, &_mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(_fd, SPI_IOC_WR_BITS_PER_WORD, &_bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(_fd, SPI_IOC_RD_BITS_PER_WORD, &_bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(_fd, SPI_IOC_WR_MAX_SPEED_HZ, &_speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(_fd, SPI_IOC_RD_MAX_SPEED_HZ, &_speed);
	if (ret == -1)
		pabort("can't get max speed hz");

#ifdef DEBUG
    printf("\n##### RFM95 file opened with fd = %d #####\n", _fd);
#endif

}

bool RFM95::RFM_write(uint8_t RegAddr, uint8_t data) {

#ifdef DEBUG
    printf("##### Write to RFM95 ADDRESS: 0x%.2X\tDATA: 0x%.2X #####\n",
            RegAddr, data);
#endif

    int ret;

    _spi_txBuf[0] = (RegAddr | 0x80); /* Set MSB to 1 to indicate write */
    _spi_txBuf[1] = data;
    _spi_transfer.len = 2;

    ret = ioctl(_fd, SPI_IOC_MESSAGE(1), &_spi_transfer);
    if (ret < 1) {
        printf("##### RFM95 WRITE FAILED #####\n");
        return false;
    }

    return true;
}

uint8_t RFM95::RFM_read(uint8_t RegAddr) {

#ifdef DEBUG
    printf("##### Read from RFM95 ADDRESS: 0x%.2X ", RegAddr);
#endif

    int ret;

    _spi_txBuf[0] = (RegAddr & 0x7F); /* Set MSB to 0 to indicate read */
    _spi_transfer.len = 2;

    ret = ioctl(_fd, SPI_IOC_MESSAGE(1), &_spi_transfer);
    if (ret < 1) {
        printf("##### RFM95 READ FAILED #####\n");
        return 0x00;
    }

#ifdef DEBUG
    printf("\tDATA: 0x%.2X\n", _spi_rxBuf[1]);
#endif

    return _spi_rxBuf[1];
}

bool RFM95::RFM_transmit(uint8_t* txBuf, uint8_t length) {

    if (txBuf == NULL) return false;
    if (length == 0 || length > 64) return false;

#ifdef DEBUG
    printf("##### Transmitting package of length: .2d bytes #####\n", length);
#endif

    uint8_t i;

    RFM_write(REG_OP_MODE, MODE_STDBY);     /* Switch to standby mode */
    RFM_write(REG_PAYLOAD_LENGTH, length);  /* Set payload length */
    RFM_write(REG_FIFO_ADDR_PTR, 0x80);     /* Set FIFO pointer location */

    /* Copy <length> bytes from txBuf into RFM95 FIFO */
    for (i = 0; i < length; ++i) {
        RFM_write(REG_FIFO, *txBuf);
        ++txBuf;
    }

    RFM_write(REG_OP_MODE, MODE_TX);    /* Trigger a transmission */

    /* Block by spinning (interrupt support would be MUCH better here) */
    while ((RFM_read(REG_OP_MODE) & 0x7F) != MODE_STDBY) {
        for (volatile int q = 0; q < 10000; ++q) {}
    }

    return true;
}


bool RFM95::RFM_test() {

    printf("##### Performing RFM95 Checks #####\n");

    if (RFM_read(REG_VERSION) != RFM9X_VER) return false;

    printf("\t--> ID CHECK PASS\n");

    if (RFM_read(REG_OCP) != OCP_SETTING) return false;
    if (RFM_read(REG_PA_DAC) != PA_DAC_SETTING) return false;
    if (RFM_read(REG_PA_CONFIG) != PA_CFG_SETTING) return false;

    printf("\t--> POWER SETTING CHECK PASS\n");

    if (RFM_read(REG_FIFO_RX_BASE_ADDR) != 0x00) return false;
    if (RFM_read(REG_FIFO_TX_BASE_ADDR) != 0x80) return false;

    printf("\t--> FIFO BASE ADDR CHECK PASS\n");

    // TODO - Add more tests?

    return true;

}
