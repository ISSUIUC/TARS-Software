
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

class RFM95 {
    public:

        int spi_open();
        int spi_close();
        bool spi_test();

    private:

        int _fd;                  /* Linux file descriptor */

        const char *_device = "/dev/spidev1.0";
        uint8_t _mode = 0;
        uint8_t _bits = 8;
        uint32_t _speed = 500000;
        uint16_t _delay;

        uint8_t _spi_txBuf[64];
        uint8_t _spi_rxBuf[64];


        struct spi_ioc_transfer _spi_transfer = {
            (unsigned long) _spi_txBuf,
            (unsigned long) _spi_rxBuf,
            0,
            _speed,
            _delay,
            _bits
        };

        uint8_t* _txFifo;       /* Pointer to transmit FIFO buffer */
        uint8_t* _rxFifo;       /* Pointer to receive FIFO buffer */
        uint32_t _txFifo_size;  /* Maximum size of txBuf */
        uint32_t _rxFifo_size;  /* Maximum size of rxBuf */
        uint32_t _txFifo_len;   /* Current length of transmit FIFO */
        uint32_t _rxFifo_len;   /* Current length of receive FIFO */

};
