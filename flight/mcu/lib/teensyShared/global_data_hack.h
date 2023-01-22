#include<chrt.h>
#include<cstdint>
struct SensorState {
    uint32_t time_ms{};
    uint32_t time_ch{};
    float ax{};
    float ay{};
    float az{};
    float gx{};
    float gy{};
    float gz{};
    float mx{};
    float my{};
    float mz{};
    float latitude{};
    float longitude{};
    float altitude{};
    float satellite_count{};
    float position_lock{};
    float temperature{};
    float pressure{};
    float barometer_altitude{};
    float highg_ax{};
    float highg_ay{};
    float highg_az{};
    float rocket_state0{};
    float rocket_state1{};
    float rocket_state2{};
    float rocket_state3{};
    float flap_extension{};
    float state_est_x{};
    float state_est_vx{};
    float state_est_ax{};
    float state_est_apo{};
    float battery_voltage{};
};

extern SensorState global_state;