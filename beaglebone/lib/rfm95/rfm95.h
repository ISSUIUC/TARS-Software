/* rfm95.h
 *
 * HopeRF RFM95W device driver for TARS-MK1 (wrapping spidev)
 *
 * Illinois Space Society - IREC 2021 Avioinics Team
 *
 * Ayberk Yaraneri
 *
 */

 #ifndef RFM95_H
 #define RFM95_H

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

// uncomment for debug output
#define DEBUG

#define RFM9X_VER 0x12      /* Expected RFM9X RegVersion */

/******************************************************************************/
/* MODEM SETTINGS - (up to our choosing, written to RFM in constructor)*/

#define OCP_SETTING 0x1F    /* Set over-current protection to 240mA */
#define PA_DAC_SETTING 0x84 /* Setting for non +20dBm power */
#define PA_CFG_SETTING 0xFF /* PA Boost ON and +17dBm power */

/******************************************************************************/
/* REGISTER ADDRESS AND BITMAP DEFINITIONS - (not exhaustive) */

#define REG_FIFO        0x00 /* FIFO read/write access */
#define REG_OP_MODE     0x01 /* Operating mode & LoRa/FSK selection */
#define REG_FRF_MSB     0x06 /* RF Carrier Freq Most Significant Bits */
#define REG_FRF_MID     0x07 /* RF Carrier Freq Intermediate Bits */
#define REG_FRF_LSB     0x08 /* RF Carrier Freq Least Significant Bits */
#define REG_PA_CONFIG   0x09 /* PA selection and Output Power control */
#define REG_PA_RAMP     0x0A /* PA ramp time, low phase noise PLL */
#define REG_OCP         0x0B /* Over Current Protection control */
#define REG_LNA         0x0C /* LNA settings */
#define REG_FIFO_ADDR_PTR       0x0D /* FIFO SPI pointer */
#define REG_FIFO_TX_BASE_ADDR   0x0E /* Start Tx data */
#define REG_FIFO_RX_BASE_ADDR   0x0F /* Start Rx data */
#define FIFO_RX_CURRENT_ADDR    0x10 /* Start address of last packet received */
#define REG_IRQ_FLAGS_MASK      0x11 /* Optional IRQ flag mask */
#define REG_IRQ_FLAGS           0x12 /* IRQ flags */
#define REG_RX_NB_BYTES         0x13 /* Number of received bytes */
#define REG_MODEM_STAT          0x18 /* Live LoRa modem status */
#define REG_HOP_CHANNEL         0x1C /* FHSS start channel */
#define REG_MODEM_CONFIG1       0x1D /* Modem PHY config 1 */
#define REG_MODEM_CONFIG2       0x1E /* Modem PHY config 2 */
#define REG_SYMB_TIMEOUT_LSB    0x1F /* Receiver timeout value */
#define REG_PREAMBLE_MSB        0x20 /* Size of premable */
#define REG_PREAMBLE_LSB        0x21 /* Size of preamble */
#define REG_PAYLOAD_LENGTH      0x22 /* LoRa paylaod length */
#define REG_MAX_PAYLOAD_LENGTH  0x23 /* LoRa maximum payload length */
#define REG_HOP_PERIOD          0x24 /* FHSS Hop period */
#define REG_FIFO_RX_BYTE_ADDR   0x25 /* Address of last byte written in FIFO */
#define REG_MODEM_CONFIG3       0x26 /* Modem PHY config 3 */
#define REG_DIO_MAPPING1        0x40 /* Mapping of pins DIO0 to DIO3 */
#define REG_DIO_MAPPING2        0x41 /* Mapping of pins DIO4 to DIO5 */
#define REG_VERSION             0x42 /* Semtech ID relating the silicon rev */
#define REG_PA_DAC              0x4D /* Higher power settings of the PA */

// REG_OP_MODE Register Bitmap
#define LORA_MODE           0x80
#define ACCESS_SHARED_REG   0x40
#define LOW_FREQ_MODE_ON    0x08
#define MODE_SLEEP          0x00
#define MODE_STDBY          0x01
#define MODE_FSTX           0x02
#define MODE_TX             0x03
#define MODE_FSRX           0x04
#define MODE_RXCONTINUOUS   0x05
#define MODE_RXSINGLE       0x06
#define MODE_CAD            0x07

/******************************************************************************/
/* RFM95 CLASS DEFINITION */

class RFM95 {
    public:

        RFM95();

        /* RFM module register access API */
        bool RFM_write(uint8_t RegAddr, uint8_t data);
        uint8_t RFM_read(uint8_t RegAddr);

        /* Copy to FIFO and transmit */
        bool RFM_transmit(uint8_t* txBuf, uint8_t length);

        /* Perform a suite of tests */
        bool RFM_test();

    private:

        void _spi_open();
        void _spi_close();

        int _fd;    /* Linux file descriptor */

        /* Default SPI bus settings (can be tweaked)*/
        const char *_device = "/dev/spidev1.0";
        uint8_t _mode = 0;
        uint8_t _bits = 8;
        uint32_t _speed = 500000;
        uint16_t _delay = 10;
        uint32_t _len = 0;

        /*
         * Buffers to store SPI data to transmit and received. Kept at only 4
         * bytes as SPI transactions to RFM module only ever happen 2 bytes at
         * a time. (1 byte address + 1 byte data)
         */
        uint8_t _spi_txBuf[4];
        uint8_t _spi_rxBuf[4];

        /* SPI struct as defined in spidev.h that dicates transaction params */
        struct spi_ioc_transfer _spi_transfer = {
            (unsigned long) _spi_txBuf,
            (unsigned long) _spi_rxBuf,
            _len,
            _speed,
            _delay,
            _bits
        };
};


#endif
