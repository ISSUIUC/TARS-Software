/* uart.cpp
 *
 * UART wrapper library that abstracts linux ioctls
 *
 * Illinois Space Society - IREC 2021 Avioinics Team
 *
 * Ayberk Yaraneri
 *
 */

 #include <stdint.h>
 #include <stdio.h>
 #include <string.h>

 // Linux headers
 #include <fcntl.h> // Contains file controls like O_RDWR
 #include <errno.h> // Error integer and strerror() function
 #include <termios.h> // Contains POSIX terminal control definitions
 #include <unistd.h> // write(), read(), close()

 #include "uart.h"


 /* UART::uart_init
  * DESCRIPTION:    opens file for uart hardware and performs initialization
  * INPUTS:         port -- linux port filename
  * RETURNS:        linux file descriptor on success, -1 on failure
  */
int UART::uart_init(char* port) {


    _fd = open(port, O_RDWR);

    struct termios tty;

    if(tcgetattr(_fd, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600. NEEDS SAME BAUD RATE AS TEENSY
    // Remember we will need to test for the optimal speed once the teensy's are set up.
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(_fd, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    return _fd;
}

/* UART::uart_read
 * DESCRIPTION:     reads bytes from UART rx
 * INPUTS:          buf -- buffer to populate with read bytes
 *                  numBytes -- number of bytes to read
 * RETURNS:         number of bytes read, -1 on failure
 */
int32_t UART::uart_read(uint8_t* buf, uint32_t numBytes) {
    int32_t ret;
    ret = read(_fd, buf, numBytes);
    if (ret < 0) {
        printf("Error reading to UART: %s", strerror(errno));
        return -1;
    }
    return ret;
}

/* UART::uart_write
 * DESCRIPTION:     writes bytes to UART tx
 * INPUTS:          buf -- buffer to write from
 *                  numBytes -- number of bytes to write
 * RETURNS:         number of bytes written, -1 on failure
 */
int32_t UART::uart_write(uint8_t* buf, uint32_t numBytes) {
    int32_t ret;
    ret = write(_fd, buf, numBytes);
    if (ret < 0) {
        printf("Error writing to UART: %s", strerror(errno));
        return -1;
    }
    return ret;
}
