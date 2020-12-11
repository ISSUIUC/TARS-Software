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

    printf("Sending data\n");

    while (1) {
        uint8_t val = rfm95.RFM_read(REG_RX_NB_BYTES);
        printf("Received Bytes: %d\n", val);

        for (volatile int i = 0; i < 1000000; ++i) {}
    }

	return 0;
}
