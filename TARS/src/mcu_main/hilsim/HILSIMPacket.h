#include <ChRt.h>
#include <cstdint>

/**
 * @struct HILSIMPacket.h
 * @brief Structure to hold data received serially from a desktop computer
 * 
 * The simulated/past-launch data is streamed through serial row-by-row to TARS. TARS receives it in the HILSIM thread and populates
 * data that would otherwise be read from sensors via the received HILSIM packet. 
*/
struct HILSIMPacket {
    uint32_t timestamp{};

    // High-G IMU data
    float imu_high_ax{};
    float imu_high_ay{};
    float imu_high_az{};

    // Barometer data
    float barometer_altitude{};
    float barometer_temperature{};
    float barometer_pressure{};

    // Low-G IMU data
    float imu_low_ax{};
    float imu_low_ay{};
    float imu_low_az{};
    float imu_low_mx{};
    float imu_low_my{};
    float imu_low_mz{};
    float imu_low_gx{};
    float imu_low_gy{};
    float imu_low_gz{};

    // GPS data
    float gps_latitude{};
    float gps_longitude{};
    float gps_altitude{};
    float gps_sats_in_view{};

    // 
};