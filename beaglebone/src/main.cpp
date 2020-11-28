
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

    if (rfm95.spi_test()) {
        printf("### RFM95 ID Check PASS ###\n");
    } else {
        printf("### RFM95 ID Check FAIL ###\n");
    }

	return 0;
}
