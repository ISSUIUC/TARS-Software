/* uart.h
 *
 * UART wrapper library that abstracts linux ioctls
 *
 * Illinois Space Society - IREC 2021 Avioinics Team
 *
 * Ayberk Yaraneri
 *
 */

#include <stdint.h>

#define SENTINEL_1 0x49     // ascii I
#define SENTINEL_2 0x53     // ascii S
#define SENTINEL_3 0x53     // ascii S

#define INPUT_BUF_SIZE  128
#define OUTPUT_BUF_SIZE 128

typedef struct dataPacket_t {
    float altitude;
    float az;
    float lattitude;
    float longitude;
    float roll_rate;
    float velocity;
    float pt1;
    float pt2;
    float pt3;
    uint32_t timestamp;
};

#define FIFO_SIZE      32  // Number of dataPackets to be stored in FIFO
#define FIFO_SIZE_B    FIFO_SIZE * sizeof(dataPacket_t)

class UART {
    public:



    private:

        /* UART interface */
        int uart_init(char* port);
        int32_t uart_read(uint8_t* buf, uint32_t numBytes);
        int32_t uart_write(uint8_t* buf, uint32_t numBytes);

        int _fd;    // Linux file descriptor

        uint8_t input_buf[INPUT_BUF_SIZE];
        uint8_t output_buf[OUTPUT_BUF_SIZE];

        /* Packet FIFO buffer interface */
		int32_t sentinel_detect();
        void push_fifo();
        void pop_fifo();

        dataPacket_t packetFifo[FIFO_SIZE_B];
        uint32_t fifoHead;
        uint32_t fifoTail;
        uint32_t fifoLen;
		bool sentinel_detected;
		uint8_t fsm_state;
};
