#include <telemetry.h>


// input value for sine function
double dummy_input = 0; 

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
  telemetry_data d{};

  // Looping input value from 0 to 2pi over and over 
  if (dummy_input > 628) {
    dummy_input = 0;
  } else {
    dummy_input+=30;
  }

  // Computing sine value
  // double sin_value = sin(dummy_input/100);
  // double cos_value = cos(dummy_input/100);
  // double tan_value = tan(dummy_input/100);


  // Setting each sensor value to the current sine value
  // d.gps_lat=sin_value;
  // d.gps_long=cos_value;
  // d.gps_alt=-1 * dummy_input/100;
  // d.barometer_alt=sin_value;
  // d.KX_IMU_ax=cos_value;
  // d.KX_IMU_ay=sin_value;
  // d.KX_IMU_az=dummy_input/100;
  // d.H3L_IMU_ax=sin_value;
  // d.H3L_IMU_ay=cos_value;
  // d.H3L_IMU_az=sin_value+cos_value;
  // d.LSM_IMU_ax=cos_value;    
  // d.LSM_IMU_ay=sin_value;
  // d.LSM_IMU_az=((dummy_input/100)*(dummy_input/100))/2;
  // d.LSM_IMU_gx=tan_value;    
  // d.LSM_IMU_gy=sin_value;
  // d.LSM_IMU_gz=cos_value;
  // d.LSM_IMU_mx=sin_value;
  // d.LSM_IMU_my=sin_value;
  // d.LSM_IMU_mz=sin_value;

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

  d.rssi = rf95.lastRssi();

  d.response_ID = last_command_id;
  memcpy(d.sign, callsign, sizeof(callsign));
  
  //Serial.println("Sending sample sensor data..."); delay(10);
  rf95.send((uint8_t *)&d, sizeof(d));

  rf95.waitPacketSent();

  //change the freqencey after we acknowledge
  if(freq_status.should_change){
    // Serial.println(freq_status.new_freq);
    rf95.setFrequency(freq_status.new_freq);
    freq_status.should_change = false;
  }
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
                                          //test without delay
  // Serial.println("Waiting for reply..."); //delay(10);
  if (rf95.available())
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      telemetry_command received;
      memcpy(&received, buf, sizeof(received));
      
      handle_command(received);
    }
    else
    {
      // Serial.println("Receive failed");
    }
  }
  else
  {
    // Serial.println("No reply, is there a listener around?");
  } 
}