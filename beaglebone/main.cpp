/*
This is the preliminary code for aquiring the sensor data through the UART protocol.
Six floats are sent as one-24 byte (unsigned char) array with a specified order
(altitude, az, lattitude, longitude, roll_rate, and finally velocity). The raw data is split into 4 byte arrays
for each float value since a float is made up of 4 bytes. They are finally converted back to floats.
-Matt

PS shoutout to Ayberk for the help with all the linux commands involved.
*/

#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

float altitude; //current altitude from altimiter
float az; //Acceleration in the z direction
float lattitude; //current lattitude from gps
float longitude; //current longitude from gps
float roll_rate; //angular velocity in the z direction
float velocity; //current velocity of the rocket


int main() {
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open("/dev/ttyO1", O_RDWR);

  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
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
  cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B9600);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  
  //Write 24 bytes to serial port-
  //This is the order in which data is sent from the teensy!
  //float sensorData[6] = {altitude, az, lattitude, longitude, roll_rate, velocity};
  //For testing
  float sensorData[6] = {0.55, 2.5, 40.110558, -88.228333, 3.1415, 9.7235};
  write(serial_port, sensorData, 24);


  // Allocate memory for read buffer, set size according to your needs
  unsigned char dataBuffer[24];

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int num_bytes = read(serial_port, &dataBuffer, sizeof(dataBuffer));


  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
      return 1;
  }

  
  //This isthe order in which data is sent from the teensy!
  //float sensorData[6] = {altitude, az, lattitude, longitude, roll_rate, velocity};
  
  unsigned char altitude_byte_array[4];
  unsigned char az_byte_array[4];
  unsigned char lattitude_byte_array[4];
  unsigned char longitude_byte_array[4];
  unsigned char rr_byte_array[4];
  unsigned char velocity_byte_array[4];

  //Unpacking respective bytes.
  for(int i = 0; i < 24; i++) {
    if (i < 4) {
      altitude_byte_array[i % 4] = dataBuffer[i];
    } else if (i < 8) { 
      az_byte_array[i % 4] = dataBuffer[i];
    } else if (i < 12) {
      lattitude_byte_array[i % 4] = dataBuffer[i];
    } else if (i < 16) {
      longitude_byte_array[i % 4] = dataBuffer[i];
    } else if (i < 20) {
      rr_byte_array[i % 4] = dataBuffer[i];
    } else {
      velocity_byte_array[i % 4] = dataBuffer[i];
    }
  }

  //Converting all from byte arrays (4 bytes each) to floats!
  altitude = *( (float*) altitude_byte_array ); 
  az = *( (float*) az_byte_array ); 
  lattitude = *( (float*) lattitude_byte_array ); 
  longitude = *( (float*) longitude_byte_array );
  roll_rate = *( (float*) rr_byte_array );
  velocity = *( (float*) velocity_byte_array);


  
  //For debugging
  printf("Read %i bytes\n", num_bytes);
  printf("Altitude: %f\n", altitude);
  printf("Az: %f\n", az);
  printf("Lattitude: %f\n", lattitude);
  printf("Longitude: %f\n", longitude);
  printf("Roll Rate: %f\n", roll_rate);
  printf("Velocity: %f\n", velocity);


  close(serial_port);
  return 0; // success
}