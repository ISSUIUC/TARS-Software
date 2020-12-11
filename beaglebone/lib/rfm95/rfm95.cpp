/* rfm95.cpp
 *
 * HopeRF RFM95W device driver for TARS-MK1 (wrapping spidev)
 *
 * Illinois Space Society - IREC 2021 Avioinics Team
 *
 * Ayberk Yaraneri
 *
 */

#include <stdio.h>

#include "rfm95.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

/** RFM95::RFM95 Constructor
 * DESCRIPTION: Opens spidev file
 * INPUTS:      none
 * RETURNS:     none
 */
RFM95::RFM95() {

    _spi_open();

}

/** RFM95::RFM_init_TX
 * DESCRIPTION: Writes necessary settings to registers for transmit mode
 * INPUTS:      none
 * RETURNS:     none
 */
void RFM95::RFM_init_TX() {

    /* Set to LoRa mode */
    RFM_write(REG_OP_MODE, LORA_MODE);

    /* Set power registers */
    RFM_write(REG_OCP, OCP_SETTING);
    RFM_write(REG_PA_DAC, PA_DAC_SETTING);
    RFM_write(REG_PA_CONFIG, PA_CFG_SETTING);

    /* Set FIFO base pointers */
    RFM_write(REG_FIFO_RX_BASE_ADDR, 0x00);
    RFM_write(REG_FIFO_TX_BASE_ADDR, 0x80);

    /* Set internal state to show TX mode */
    _rfm_mode = MODE_TX;

}

/** RFM95::RFM_init_RX
 * DESCRIPTION: Writes necessary settings to registers for receive mode
 * INPUTS:      none
 * RETURNS:     none
 */
void RFM95::RFM_init_RX() {

    /* Set to LoRa mode */
    RFM_write(REG_OP_MODE, LORA_MODE);

    /* Set FIFO base pointers */
    RFM_write(REG_FIFO_RX_BASE_ADDR, 0x00);
    RFM_write(REG_FIFO_TX_BASE_ADDR, 0x80);

    RFM_write(REG_OP_MODE, MODE_RXCONTINUOUS);

    /* Set internal state to show TX mode */
    _rfm_mode = MODE_RXCONTINUOUS;

}

/** RFM95::RFM_write
 * DESCRIPTION: Writes a byte to an RFM register via SPI
 * INPUTS:      RegAddr - address of register in RFM95 module to write to
 *              data - byte of data to be written
 * RETURNS:     false on failure
 */
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

/** RFM95::RFM_read
 * DESCRIPTION: Reads a byte from an RFM register via SPI
 * INPUTS:      RegAddr - address of register in RFM95 module to read from
 * RETURNS:     byte that was read, 0x00 on failure
 */
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

/** RFM95::RFM_transmit
 * DESCRIPTION: Transfers a sequence of bytes to the RFM module's FIFO buffer,
 *              then puts module into transmit mode.
 * NOTE:        Function blocks until RFM module goes back into standby mode!
 *              i.e. it will not return until transmission completes.
 * INPUTS:      txBuf - pointer to an array of bytes that will be sent
 *              length - number of bytes to be transmitted
 * RETURNS:     false on failure
 */
bool RFM95::RFM_transmit(uint8_t* txBuf, uint8_t length) {

    if (txBuf == NULL) return false;
    if (length == 0 || length > 64) return false;

#ifdef DEBUG
    printf("##### Transmitting package of length: %.2d bytes #####\n", length);
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

/** RFM95::RFM_test
 * DESCRIPTION: Performans a sequence of tests to ensure that the RFM module is
 *              functioning, and that the proper settings have been written.
 * INPUTS:      none
 * RETURNS:     false on any failure, true if all tests pass
 */
bool RFM95::RFM_test() {

    printf("##### Performing RFM95 Checks #####\n");

    if (_rfm_mode == MODE_TX) {
        if (RFM_read(REG_VERSION) != RFM9X_VER) return false;
        printf("\t--> ID CHECK PASS\n");
        if (RFM_read(REG_OCP) != OCP_SETTING) return false;
        if (RFM_read(REG_PA_DAC) != PA_DAC_SETTING) return false;
        if (RFM_read(REG_PA_CONFIG) != PA_CFG_SETTING) return false;
        printf("\t--> POWER SETTING CHECK PASS\n");
        if (RFM_read(REG_FIFO_RX_BASE_ADDR) != 0x00) return false;
        if (RFM_read(REG_FIFO_TX_BASE_ADDR) != 0x80) return false;
        printf("\t--> FIFO BASE ADDR CHECK PASS\n");
    }

    if (_rfm_mode == MODE_RXCONTINUOUS) {
        if (RFM_read(REG_VERSION) != RFM9X_VER) return false;
        printf("\t--> ID CHECK PASS\n");
        if (RFM_read(REG_FIFO_RX_BASE_ADDR) != 0x00) return false;
        if (RFM_read(REG_FIFO_TX_BASE_ADDR) != 0x80) return false;
        printf("\t--> FIFO BASE ADDR CHECK PASS\n");
        // TODO - implement more receiver mode tests
    }

    // TODO - Add more tests?

    return true;

}

/** RFM95::RFM_reset
 * DESCRIPTION:     Cycles RFM module reset line to trigger reset
 * INPUTS: none
 * RETURNS: none
 */
void RFM95::RFM_reset() {

    FILE *fp;

    fp = fopen("/sys/class/gpio/gpio48/direction", "w");
    fputs("out", fp);
    fclose(fp);

    fp = fopen("/sys/class/gpio/gpio48/value", "w");
    fputs("0", fp);
    fclose(fp);

    for (volatile int i = 0; i < 1000000 ; ++i) {}

    fp = fopen("/sys/class/gpio/gpio48/value", "w");
    fputs("1", fp);
    fclose(fp);
}

/** RFM95::_spi_open
 * DESCRIPTION: Opens spi file in linux, stores file descriptor in private class
 *              variable _fd
 * INPUTS:      none
 * RETURNS:     none
 */
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
