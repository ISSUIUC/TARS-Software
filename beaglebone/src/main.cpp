
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

    int rfm95_fd = rfm95.spi_open();

	printf("RFM95 open success with fd: %d\n", rfm95_fd);

    rfm95.spi_test();

	return 0;
}
