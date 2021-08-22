/* main.cpp
 *
 * Beaglebone ground station software
 *
 * Illinois Space Society - IREC 2021 Avioinics Team
 *
 * Ayberk Yaraneri
 *
 */


#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "../lib/rfm95/rfm95.h"

int main(void)
{

    RFM95 rfm95;

    rfm95.RFM_init_RX();

    rfm95.RFM_test();

    uint8_t rxBuf[256];

    while (1) {
        uint8_t numBytes = rfm95.RFM_receive(rxBuf, 256);

        if (numBytes > 0) {
            printf("##### Received %d Bytes:\n", numBytes);

            for (int q = 0; q < numBytes; ++q) {
                printf("0x%.2X\t", rxBuf[q]);
            }
            printf("\n");
        }

        for (volatile int i = 0; i < 1000000; ++i) {}

        // printf("##### hmmm: %d\n", rfm95.RFM_read(REG_RX_NB_BYTES));
    }

	return 0;
}
