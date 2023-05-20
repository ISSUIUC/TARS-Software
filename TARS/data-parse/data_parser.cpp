#if defined(__CLION_IDE__) || defined(__INTELLISENSE__)  // to make the IDE happy
#include "common/packet.h"
#else
#include PACKET_INCLUDE_PATH
#endif

#include <fstream>
#include <iostream>


int main(int argc, char** argv) {
    std::ifstream input{argv[1], std::ios::binary};
    std::ostream& output = std::cout;

    input.ignore(41);

    output
        << "has_lowG_data" << ","
        << "lowG_data.ax" << ","
        << "lowG_data.ay" << ","
        << "lowG_data.az" << ","
        << "lowG_data.gx" << ","
        << "lowG_data.gy" << ","
        << "lowG_data.gz" << ","
        << "lowG_data.mx" << ","
        << "lowG_data.my" << ","
        << "lowG_data.mz" << ","
        << "lowG_data.timeStamp_lowG" << ",";

    output
        << "has_highG_data" << ","
        << "highG_data.ax" << ","
        << "highG_data.ay" << ","
        << "highG_data.az" << ","
        << "highG_data.timeStamp_highG" << ",";


    output
        << "has_gps_data" << ","
        << "gps_data.altitude" << ","
        << "gps_data.fix_type" << ","
        << "gps_data.latitude" << ","
        << "gps_data.longitude" << ","
        << "gps_data.posLock" << ","
        << "gps_data.siv_count" << ","
        << "gps_data.timeStamp_GPS" << ",";

    output
        << "has_barometer_data" << ","
        << "barometer_data.temperature" << ","
        << "barometer_data.altitude" << ","
        << "barometer_data.pressure" << ","
        << "barometer_data.timeStamp_barometer" << ",";

    output
        << "has_rocketState_data" << ",";
    for (int i = 0; i < 4; i++) {
        output << "rocketState_data.rocketState" << i << ",";
    }
    output << "rocketState_data.timeStamp_RS" << ",";

    output
        << "has_flap_data" << ","
        << "flap_data.extension" << ","
        << "flap_data.timeStamp_flaps" << ",";

    output
        << "has_state_data" << ","
        << "state_data.state_x" << ","
        << "state_data.state_vx" << ","
        << "state_data.state_ax" << ","
        << "state_data.state_apo" << ","
        << "state_data.timeStamp_state" << ",";

    output
        << "has_voltage_data" << ","
        << "voltage_data.v_battery" << "\n";

    sensorDataStruct_t data;
    while (input.read(reinterpret_cast<char*>(&data), sizeof(data))) {
        output
            << data.has_lowG_data << ","
            << data.lowG_data.ax << ","
            << data.lowG_data.ay << ","
            << data.lowG_data.az << ","
            << data.lowG_data.gx << ","
            << data.lowG_data.gy << ","
            << data.lowG_data.gz << ","
            << data.lowG_data.mx << ","
            << data.lowG_data.my << ","
            << data.lowG_data.mz << ","
            << data.lowG_data.timeStamp_lowG << ",";

        output
            << data.has_highG_data << ","
            << data.highG_data.hg_ax << ","
            << data.highG_data.hg_ay << ","
            << data.highG_data.hg_az << ","
            << data.highG_data.timeStamp_highG << ",";


        output
            << data.has_gps_data << ","
            << data.gps_data.altitude << ","
            << data.gps_data.fix_type << ","
            << data.gps_data.latitude << ","
            << data.gps_data.longitude << ","
            << data.gps_data.posLock << ","
            << data.gps_data.siv_count << ","
            << data.gps_data.timeStamp_GPS << ",";

        output
            << data.has_barometer_data << ","
            << data.barometer_data.temperature << ","
            << data.barometer_data.altitude << ","
            << data.barometer_data.pressure << ","
            << data.barometer_data.timeStamp_barometer << ",";

        output
            << data.has_rocketState_data << ",";

        for (int i = 0; i < 4; i++) {
            output << (uint8_t) data.rocketState_data.rocketStates[i] << ",";
        }

        output << data.rocketState_data.timestamp << ",";

        output
            << data.has_flap_data << ","
            << data.flap_data.extension << ","
            << data.flap_data.timeStamp_flaps << ",";

        output
            << data.has_kalman_data << ","
            << data.kalman_data.kalman_x << ","
            << data.kalman_data.kalman_vx << ","
            << data.kalman_data.kalman_ax << ","
            << data.kalman_data.kalman_apo << ","
            << data.kalman_data.timeStamp_state << ",";

        output
            << data.has_voltage_data << ","
            << data.voltage_data.v_battery;

        output << '\n';
    }
}