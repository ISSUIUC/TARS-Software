/**
 * @file telemetry.cpp
 *
 * @brief This file defines the telemetry class used to facilitate
 * telemetry commands and data transfer between the on-board flight
 * computer and the ground station.
 *
 * Spaceshot Avionics 2021-22
 * Illinois Space Society - Telemetry Team
 * Gautam Dayal
 * Nicholas Phillips
 * Patrick Marschoun
 * Peter Giannetos
 */

#define SERIAL_PLOTTING

#include <telemetry.h>
#include <limits>

#include "dataLog.h"

Telemetry tlm;

/**
 * @brief This function maps an input value onto within a particular range into a fixed point value of a certin binary size
 *
 * @param val: number to map into target range, values outside of the range will be clamped
 * 
 * @param range: range to map number into. For unsigned output, [0, range). For signed output [-range/2, range)
 *
 * @return fixed point value represing val mapped onto the target range
 */
template<typename T>
T inv_convert_range(float val, float range){
  size_t numeric_range = (int64_t)std::numeric_limits<T>::max() - (int64_t)std::numeric_limits<T>::min() + 1;
  float converted = val * (float)numeric_range / range;
  return std::max(std::min((float)std::numeric_limits<T>::max(), converted), (float)std::numeric_limits<T>::min());
}

void Telemetry::init() {
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);

    // manual reset

    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init()) {
        while (true) {
            Serial.println("Radio Initialization Failed");
        }
    }
    Serial.println("[DEBUG]: Radio Initialized");

    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf =
    // 128chips/symbol, CRC on

    if (!rf95.setFrequency(RF95_FREQ)) {
        while (true) {
            Serial.println("[ERROR]: Default setFrequency Failed");
        }
    }

    /*
     * The default transmitter power is 13dBm, using PA_BOOST.
     * If you are using RFM95/96/97/98 modules which uses the PA_BOOST
     * transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
     */
    rf95.setTxPower(23, false);
}

Telemetry::Telemetry() : rf95(RFM95_CS, RFM95_INT) { }

/**
 * @brief  This function handles commands sent from the ground station
 * to TARS. The effects of this function depend on the command
 * sent.
 *
 * @param cmd: struct containing information necessary to process
 *             ground station command.
 *
 * @return void
 */
void Telemetry::handleCommand(const telemetry_command &cmd) {
    /* Check if the security code is present and matches on ground and on the
     * rocket */
    if (cmd.verify != std::array<char, 6>{'A', 'Y', 'B', 'E', 'R', 'K'}) {
        return;
    }
    /* Check if lasted command ID matched current command ID */
    if (last_command_id == cmd.cmd_id) {
        return;
    }
    last_command_id = (int16_t) cmd.cmd_id;

    /*
     * Write frequency to SD card to save
     * between runs
     */
    if (cmd.command == SET_FREQ) {
        freq_status.should_change = true;
        freq_status.new_freq = cmd.freq;
    }

    if (cmd.command == SET_CALLSIGN) {
        memcpy(callsign, cmd.callsign, sizeof(cmd.callsign));
        Serial.println("[DEBUG]: Got callsign");
    }

    if (cmd.command == ABORT) {
        if (!abort) {
            abort = !abort;
        }
        Serial.println("[DEBUG]: Got abort");
    }
}

/**
 * @brief This function transmits data from the struct provided as
 * the parameter (data collected from sensor suite) to the
 * ground station. The function also switches to a new commanded
 * frequency based on a previously received command and waits for
 * a response from the ground station.
 *
 * @param sensor_data: struct of data from the sensor suite to be
 *                     transmitted to the ground station.
 *
 * @return void
 */
void Telemetry::transmit() {
    static bool blue_state = false;
    digitalWrite(LED_BLUE, blue_state);
    blue_state = !blue_state;

    TelemetryPacket packet = makePacket(dataLogger.read());
    rf95.send((uint8_t *)&packet, sizeof(packet));

    chThdSleepMilliseconds(170);

    rf95.waitPacketSent();

    // change the frequency after we acknowledge
    if (freq_status.should_change) {
        rf95.setFrequency(freq_status.new_freq);
        freq_status.should_change = false;
    }

    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.available() && rf95.recv(buf, &len)) {
        telemetry_command received{};
        memcpy(&received, buf, sizeof(received));

        handleCommand(received);
    }
}

void Telemetry::serialPrint(const sensorDataStruct_t& sensor_data) {
    Serial.print(R"({"type": "data", "value": {)");
    Serial.print(R"("response_ID":)");
    Serial.print(000);
    Serial.print(',');
    Serial.print(R"("gps_lat":)");
    Serial.print(0);
    Serial.print(",");
    Serial.print(R"("gps_long":)");
    Serial.print(0);
    Serial.print(",");
    Serial.print(R"("gps_alt":)");
    Serial.print(0);
    Serial.print(",");
    Serial.print(R"("barometer_alt":)");
    Serial.print(sensor_data.barometer_data.altitude, 5);
    Serial.print(',');
    Serial.print(R"("KX_IMU_ax":)");
    Serial.print(sensor_data.highG_data.hg_ax, 5);
    Serial.print(',');
    Serial.print(R"("KX_IMU_ay":)");
    Serial.print(sensor_data.highG_data.hg_ay, 5);
    Serial.print(',');
    Serial.print(R"("KX_IMU_az":)");
    Serial.print(sensor_data.highG_data.hg_az, 5);
    Serial.print(',');
    // Serial.print(R"("H3L_IMU_ax":)"); Serial.print(data.H3L_IMU_ax);
    // Serial.print(','); Serial.print(R"("H3L_IMU_ay":)");
    // Serial.print(data.H3L_IMU_ay); Serial.print(',');
    // Serial.print(R"("H3L_IMU_az":)"); Serial.print(data.H3L_IMU_az);
    // Serial.print(',');
    Serial.print(R"("LSM_IMU_ax":)");
    Serial.print(sensor_data.lowG_data.ax, 5);
    Serial.print(',');
    Serial.print(R"("LSM_IMU_ay":)");
    Serial.print(sensor_data.lowG_data.ay, 5);
    Serial.print(',');
    Serial.print(R"("LSM_IMU_az":)");
    Serial.print(sensor_data.lowG_data.az, 5);
    Serial.print(',');
    Serial.print(R"("LSM_IMU_gx":)");
    Serial.print(sensor_data.lowG_data.gx, 5);
    Serial.print(',');
    Serial.print(R"("LSM_IMU_gy":)");
    Serial.print(sensor_data.lowG_data.gy, 5);
    Serial.print(',');
    Serial.print(R"("LSM_IMU_gz":)");
    Serial.print(sensor_data.lowG_data.gz, 5);
    Serial.print(',');
    Serial.print(R"("LSM_IMU_mx":)");
    Serial.print(sensor_data.lowG_data.mx, 5);
    Serial.print(',');
    Serial.print(R"("LSM_IMU_my":)");
    Serial.print(sensor_data.lowG_data.my, 5);
    Serial.print(',');
    Serial.print(R"("LSM_IMU_mz":)");
    Serial.print(sensor_data.lowG_data.mz, 5);
    Serial.print(',');
    Serial.print(R"("FSM_state":)");
    Serial.print(1);
    Serial.print(',');
    Serial.print(R"("sign":")");
    Serial.print("SIGN");
    Serial.print("\",");
    Serial.print(R"("RSSI":)");
    Serial.print(rf95.lastRssi());
    Serial.print(',');
    Serial.print(R"("Voltage":)");
    Serial.print(sensor_data.voltage_data.v_battery, 5);
    Serial.print(',');
    Serial.print(R"("frequency":)");
    Serial.print(RF95_FREQ);
    Serial.print(',');
    Serial.print(R"("flap_extension":)");
    Serial.print(sensor_data.flap_data.extension, 5);
    Serial.print(",");
    Serial.print(R"("STE_ALT":)");
    Serial.print(sensor_data.kalman_data.kalman_x, 5);
    Serial.print(",");
    Serial.print(R"("STE_VEL":)");
    Serial.print(sensor_data.kalman_data.kalman_vx, 5);
    Serial.print(",");
    Serial.print(R"("STE_ACC":)");
    Serial.print(sensor_data.kalman_data.kalman_ax, 5);
    Serial.print(",");
    Serial.print(R"("TEMP":)");
    Serial.print(sensor_data.barometer_data.temperature);
    Serial.print(",");
    Serial.print(R"("pressure":)");
    Serial.print(sensor_data.barometer_data.pressure, 5);
    Serial.print(",");
    Serial.print(R"("STE_APO":)");
    Serial.print(sensor_data.kalman_data.kalman_apo, 5);
    Serial.print("");
    Serial.println("}}");
}


TelemetryPacket Telemetry::makePacket(const sensorDataStruct_t &data_struct) {
    TelemetryPacket packet{};
    packet.gps_lat = data_struct.gps_data.latitude;
    packet.gps_long = data_struct.gps_data.longitude;
    packet.gps_alt = data_struct.gps_data.altitude;

    packet.gnc_state_ax = data_struct.kalman_data.kalman_ax;
    packet.gnc_state_vx = data_struct.kalman_data.kalman_vx;
    packet.gnc_state_x = data_struct.kalman_data.kalman_x;
    packet.gns_state_apo = data_struct.kalman_data.kalman_apo;

    packet.response_ID = last_command_id;
    packet.rssi = rf95.lastRssi();
    packet.voltage_battery = inv_convert_range<uint8_t>(data_struct.voltage_data.v_battery, 16);
    packet.FSM_State = (uint8_t)data_struct.rocketState_data.rocketStates[0];

    TelemetryDataLite data{};
    packet.datapoint_count = 0;
    for(int8_t i = 0; i < 4 && buffered_data.pop(data); i++){
        packet.datapoints[i] = data;
        packet.datapoint_count = i + (int8_t) 1;
    }
    return packet;
}

void Telemetry::bufferData() {
    sensorDataStruct_t sensor_data = dataLogger.read();
    // Serial.println(dataLogger.count);
    TelemetryDataLite data{};
    data.timestamp = TIME_I2MS(chVTGetSystemTime());
    data.barometer_pressure = inv_convert_range<uint16_t>(sensor_data.barometer_data.pressure, 4096);

    data.highG_ax = inv_convert_range<int16_t>(sensor_data.highG_data.hg_ax, 256);
    data.highG_ay = inv_convert_range<int16_t>(sensor_data.highG_data.hg_ay, 256);
    data.highG_az = inv_convert_range<int16_t>(sensor_data.highG_data.hg_az, 256);

    data.gyro_x = inv_convert_range<int16_t>(sensor_data.lowG_data.gx, 8192);
    data.gyro_y = inv_convert_range<int16_t>(sensor_data.lowG_data.gy, 8192);
    data.gyro_z = inv_convert_range<int16_t>(sensor_data.lowG_data.gz, 8192);

    data.flap_extension = (uint8_t)sensor_data.flap_data.extension;
    data.barometer_temp = inv_convert_range<uint8_t>(sensor_data.barometer_data.temperature, 128);

    buffered_data.push(data);

#ifdef SERIAL_PLOTTING
    serialPrint(sensor_data);
#endif
}
