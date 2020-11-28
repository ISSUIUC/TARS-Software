/*
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

int RFM95::spi_open() {

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

    return _fd;
}

bool RFM95::spi_test() {

    int ret;

    _spi_txBuf[0] = 0xC2;

    _spi_transfer.len = 2;

    ret = ioctl(_fd, SPI_IOC_MESSAGE(1), &_spi_transfer);
    if (ret < 1) return false;

    if (_spi_rxBuf[1] != 0x12) return false;

    return true;

}
