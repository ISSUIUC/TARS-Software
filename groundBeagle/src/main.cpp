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

    rfm95.RFM_test();

    printf("Sending data\n");

    uint8_t buf[4] = {0xDE, 0xAD, 0xBE, 0xEF};

    // rfm95.RFM_transmit(buf, 4);

    printf("transmit complete\n");

	return 0;
}
