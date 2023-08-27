#pragma once

#include <ChRt.h>

#include <cstdint>

/**
 * @struct HILSIMPacket
 * @brief Structure to hold data received serially from a desktop computer
 *
 * The simulated/past-launch data is streamed through serial row-by-row to TARS. TARS receives it in the HILSIM thread
 * and populates data that would otherwise be read from sensors via the received HILSIM packet. Used for rapid testing
 * and iteration of onboard hardware, GNC, and telemetry systems.
 */
typedef struct HILSIMPacket {
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
    float imu_low_gx{};
    float imu_low_gy{};
    float imu_low_gz{};

    // Mag data
    float mag_x{};
    float mag_y{};
    float mag_z{};

    // Orientation data
    float ornt_roll{};
    float ornt_pitch{};
    float ornt_yaw{};
} HILSIMPacket;
