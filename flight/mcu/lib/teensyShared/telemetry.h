#include <SPI.h>
#include <RH_RF95.h>

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

enum TLM_state {
    STATE_PRE_LAUNCH,
    STATE_FLIGHT,
    STATE_POST_LAUNCH
}

class Telemetry {
    public:
    
    
    private:

};