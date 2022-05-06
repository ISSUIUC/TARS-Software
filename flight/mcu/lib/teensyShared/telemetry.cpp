#include <telemetry.h>


Telemetry::Telemetry(): rf95(RFM95_CS, RFM95_INT) {

    pinMode(RFM95_RST, OUTPUT);
    //pinMode(RFM95_EN, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    //digitalWrite(RFM95_EN, HIGH);
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
    //Serial.println("Radio Initialized");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency Failed");
        while (1);
    }    
    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
    // you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(23, false);
}

void Telemetry::handle_command(const telemetry_command & cmd){
/* Check if lasted command ID matched current command ID */
      if (cmd.verify != std::array<char, 6>{'A', 'Y', 'B', 'E', 'R','K'}) {
        return;
      }
      if(last_command_id == cmd.cmd_id){
        return;
      }
      last_command_id = cmd.cmd_id;
      if (cmd.command == SET_FREQ) {
        freq_status.should_change = true;
        freq_status.new_freq = cmd.freq;
        // Serial.println("Got freq");  //don't want serial prints in flight code
      } 

      if (cmd.command == SET_CALLSIGN) {
        memcpy(callsign, cmd.callsign, sizeof(cmd.callsign));
        Serial.println("Got callsign");
      }

      if (cmd.command == ABORT) {
        if (abort == false) {
          abort = !abort;
        }
        Serial.println("Got abort");
      }

      if (cmd.command == TEST_FLAPS) {
        Serial.println("Got test flaps");
        testing = !testing;
      }
}

void Telemetry::transmit(const sensorDataStruct_t &sensor_data) {
  static bool blue_state = false;
  digitalWrite(LED_BLUE, blue_state);
  blue_state = !blue_state;
  telemetry_data d{};

  d.gps_lat = sensor_data.gps_data.latitude;
  d.gps_long = sensor_data.gps_data.longitude;
  d.gps_alt = sensor_data.gps_data.altitude;
  d.barometer_alt = sensor_data.barometer_data.altitude;
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