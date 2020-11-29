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
    printf("\n##### Write to RFM95 ADDRESS: 0x%X\tDATA: 0x%X #####",
            RegAddr, data);
#endif

    int ret;

    _spi_txBuf[0] = (RegAddr | 0x80); /* Set MSB to 1 to indicate write */
    _spi_txBuf[1] = data;
    _spi_transfer.len = 2;

    ret = ioctl(_fd, SPI_IOC_MESSAGE(1), &_spi_transfer);
    if (ret < 1) {
        printf("\n##### RFM95 WRITE FAILED #####\n");
        return false;
    }

    return true;
}

uint8_t RFM95::RFM_read(uint8_t RegAddr) {

#ifdef DEBUG
    printf("\n##### Read from RFM95 ADDRESS: 0x%X #####\n", RegAddr);
#endif

    int ret;

    _spi_txBuf[0] = (RegAddr & 0x7F); /* Set MSB to 0 to indicate read */
    _spi_transfer.len = 2;

    ret = ioctl(_fd, SPI_IOC_MESSAGE(1), &_spi_transfer);
    if (ret < 1) {
        printf("\n##### RFM95 READ FAILED #####\n");
        return 0x00;
    }

    return _spi_rxBuf[1];
}

bool RFM95::RFM_test() {
    
    printf("\n##### Performing RFM95 Checks #####\n");

    uint8_t rfm_id = RFM_read(REG_VERSION);

    if (rfm_id != RFM9X_VER) return false;


    printf("\t--> ID CHECK PASS\n");

    // TODO - Add more tests?

    return true;

}
