/*
 * This file defines the telemetry class used to facilitate
 * telemetry commands and data transfer between the on-board flight
 * computer and the ground station.
 * 
 * Spaceshot Avionics 2021-22
 * Illinois Space Society
 */

#include <telemetry.h>

Telemetry::Telemetry(): rf95(RFM95_CS, RFM95_INT) {

    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);

    // manual reset

    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init()) {
        Serial.println("Radio Initialization Failed");
        while (1);
    }
    Serial.println("[DEBUG]: Radio Initialized");

    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("[ERROR]: Default setFrequency Failed");
        while (1);
    } 

    /*
     * Load previous frequency from freq.txt.
     * If successful, the default frequency is 
     * disregarded.
     */
    // read_file = SD.open("freq.txt", O_READ);
    // if (read_file) {
    //     Serial.println("[DEBUG]: Reading data from freq.txt");
    //     Serial.print("[DEBUG]: Frequency from SD freq.txt file: ");
    //     float freq_to_set;
    //     read_file.read(& freq_to_set, 4);
    //     // memcpy(&freq_to_set, read_file.read(), sizeof(read_file.read()));
    //     Serial.println(freq_to_set); 
    //     if (!rf95.setFrequency(freq_to_set)) {
    //         Serial.println("[WARNING]: Failed to set saved frequency, going back to default frequency.");
    //     } 
        
    // } else {
    //     Serial.println("[ERROR]: Failed to open freq file while read, using default frequency.");
    // }
    // read_file.close();

    /*
     * The default transmitter power is 13dBm, using PA_BOOST.
     * If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
     * you can set transmitter powers from 5 to 23 dBm:
     */
    rf95.setTxPower(23, false);
}

/**
 * This function handles commands sent from the ground station
 * to TARS. The effects of this function depend on the command
 * sent.
 * 
 * @param cmd: struct containing information necessary to process
 *             ground station command.
 * 
 * @return void
 */
void Telemetry::handle_command(const telemetry_command & cmd){
    /* Check if lasted command ID matched current command ID */
    if (cmd.verify != std::array<char, 6>{'A', 'Y', 'B', 'E', 'R','K'}) {
        return;
    }
    if(last_command_id == cmd.cmd_id){
        return;
    }
    last_command_id = cmd.cmd_id;

    /*
     * Write frequency to SD card to save 
     * between runs
     */
    if (cmd.command == SET_FREQ) {
        freq_status.should_change = true;
        freq_status.new_freq = cmd.freq;
        // Serial.println("Got freq");  //don't want serial prints in flight code
        

        // Writing freq to file
        // Fix this to overwriteHave to overwrite and not truncate? @gautamdayal please test.
        // write_file = SD.open("freq.txt", FILE_WRITE | O_TRUNC);
        // if (write_file) {
        //     Serial.println("[DEBUG] WRITE freq to SD card.");
        //     write_file.write(& cmd.freq, 4);
        // } else {
        //     Serial.println("[ERROR] Failed to open freq file while writing");
        //     // TODO: possibly add functionality here to create the file
        //     // and store the default frequency in it.
        //     // write_file.write(& "434", 4);
        // }
        // write_file.close();
    } 

    if (cmd.command == SET_CALLSIGN) {
        memcpy(callsign, cmd.callsign, sizeof(cmd.callsign));
        Serial.println("[DEBUG]: Got callsign");
    }

    if (cmd.command == ABORT) {
        if (abort == false) {
            abort = !abort;
        }
        Serial.println("[DEBUG]: Got abort");
    }

    if (cmd.command == TEST_FLAPS) {
        Serial.println("[DEBUG]: Got test flaps");
        testing = !testing;
    }
}

/**
 * This function transmits data from the struct provided as
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
void Telemetry::transmit(const sensorDataStruct_t &sensor_data) {
    static bool blue_state = false;
    digitalWrite(LED_BLUE, blue_state);
    blue_state = !blue_state;
    telemetry_data d{};
  
    d.gps_lat = sensor_data.gps_data.latitude;
    d.gps_long = sensor_data.gps_data.longitude;
    d.gps_alt = sensor_data.gps_data.altitude;
    d.barometer_alt = sensor_data.barometer_data.altitude;
    d.barometer_temp = sensor_data.barometer_data.temperature;
    d.barometer_pressure = sensor_data.barometer_data.pressure;
    d.KX_IMU_ax = sensor_data.highG_data.hg_ax;
    d.KX_IMU_ay = sensor_data.highG_data.hg_ay;
    d.KX_IMU_az = sensor_data.highG_data.hg_az;
    d.H3L_IMU_ax = sensor_data.highG_data.hg_ax;
    d.H3L_IMU_ay = sensor_data.highG_data.hg_ay;
    d.H3L_IMU_az = sensor_data.highG_data.hg_az;
    d.LSM_IMU_ax = sensor_data.lowG_data.ax;    
    d.LSM_IMU_ay = sensor_data.lowG_data.ay;
    d.LSM_IMU_az = sensor_data.lowG_data.az;
    d.LSM_IMU_gx = sensor_data.lowG_data.gx;    
    d.LSM_IMU_gy = sensor_data.lowG_data.gy;
    d.LSM_IMU_gz = sensor_data.lowG_data.gz;
    d.LSM_IMU_mx = sensor_data.lowG_data.mx;
    d.LSM_IMU_my = sensor_data.lowG_data.my;
    d.LSM_IMU_mz = sensor_data.lowG_data.mz;
    d.flap_extension = sensor_data.flap_data.l1;
    d.voltage_battry = sensor_data.voltage_data.v_battery;
    d.state_x = sensor_data.state_data.state_x;
    d.state_vx = sensor_data.state_data.state_vx;
    d.state_ax = sensor_data.state_data.state_ax;
    d.state_apo = sensor_data.state_data.state_apo;
    d.rssi = rf95.lastRssi();
    d.response_ID = last_command_id;
    d.FSM_state = sensor_data.rocketState_data.rocketState;
    memcpy(d.sign, callsign, sizeof(callsign));
    
    rf95.send((uint8_t *)&d, sizeof(d));
  
    chThdSleepMilliseconds(170);
  
    rf95.waitPacketSent();
  
    //change the freqencey after we acknowledge
    if(freq_status.should_change){
        rf95.setFrequency(freq_status.new_freq);
        freq_status.should_change = false;
    }

    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.available() && rf95.recv(buf, &len))
    { 
        telemetry_command received;
        memcpy(&received, buf, sizeof(received));

        handle_command(received);
    }
}