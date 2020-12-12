/* main.cpp
 *
 * TARS-MK1 beaglebone flight software
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
#include "../lib/uart/uart.h"

int main(void)
{

    RFM95 rfm95;
    rfm95.RFM_test();

    UART acUART;
    acUART.uart_init("/dev/ttyO1");
    
    uint8_t data[64];

    while (1) {
	
	dataPacket_t* packet = acUART.uart_readPacket();

	if (packet == NULL) {
		printf("Packet is NULL\n");
	}
	else {
		printf("Packet received!\n");
	}

	printf("Altitude:\t %f\n", packet->altitude);
	printf("az:\t\t %f\n", packet->az);
	printf("lattitude:\t %f\n", packet->lattitude);
	printf("longitude:\t %f\n", packet->longitude);
	printf("roll_rate:\t %f\n", packet->roll_rate);
	printf("velocity:\t %f\n", packet->velocity);
	printf("pt1:\t\t %f\n", packet->pt1);
	printf("pt2:\t\t %f\n", packet->pt2);
	printf("pt3:\t\t %f\n", packet->pt3);
	printf("timestamp:\t %d\n", packet->timestamp);

	/*
	int nBytes = acUART.uart_read(data, 64);
	
	printf("Received %d bytes:\n", nBytes);
	for (int i = 0; i < 64; ++i) {
		printf("0x%.2X\t", data[i]);
	}
	printf("\n\n");
	*/
    }
    

    return 0;
}
