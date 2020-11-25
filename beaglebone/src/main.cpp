
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
	int rfm95_fd;

    rfm95_fd = rfm95_open();

	printf("RFM95 Open Success.\n");

    if (rfm95_test(rfm95_fd)) {
        printf("### RFM95 ID Check PASS ###\n");
    } else {
        printf("### RFM95 ID Check FAIL ###\n");
    }

	return 0;
}
