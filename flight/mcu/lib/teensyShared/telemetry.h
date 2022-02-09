#include <SPI.h>
#include <RH_RF95.h>
#include <ChRt.h>

/*
Data to be sent (in order of priority):
    - Commands
        - Set frequency
        - Set trasmitting power
        - 
    - GPS (x, y, z)
    - Barometer (telemetrum/stratologger) (altitude)
    - Rocket FSM state ()
    - IMU ()
    Possibilities:
        - Battery voltage
*/

//Make sure to change these pinout depending on wiring
//Don't forget to change the ini file to build the correct main file
#define RFM95_CS 10
#define RFM95_RST 15
#define RFM95_INT 17
#define RFM95_EN 14

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

enum TLM_state {
    STATE_PRE_LAUNCH,
    STATE_FLIGHT,
    STATE_POST_LAUNCH
};

class Telemetry {
    public:
        Telemetry();
        void receive();
        void transmit();
    private:
        TLM_state telemetryState = STATE_PRE_LAUNCH;
        int packetnum;
        RH_RF95 rf95;
};