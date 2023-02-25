#include <Arduino.h>

#define PWM_BUZZ 11

#define SD_THING
// #define BNO086
// #define SCANNER_I2C
// #define KX134
// #define BUZZER
// #define TLM


#ifdef BNO086
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give 
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.  
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9


// #define FAST_MODE

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1



struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
arduino::MbedI2C wire(18,19);

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setup(void) {

  Serial.begin(115200);
  delay(2000);

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C(0x4B, &wire)) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop() {

  if (bno08x.wasReset()) {
    // bno08x.begin_I2C(0x4B, &wire);
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    static long last = 0;
    long now = micros();
    Serial.print(now - last);             Serial.print("\t");
    last = now;
    Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);                Serial.print("\t");
    Serial.print(ypr.pitch);              Serial.print("\t");
    Serial.println(ypr.roll);
  }

}

#endif BNO086



#ifdef SCANNER_I2C
#include <Wire.h>
arduino::MbedI2C wire(18,19);

void setup()
{
  wire.begin();

  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}


void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    wire.beginTransmission(address);
    error = wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
#endif

#ifdef KX134


#include <Wire.h>
#include "SparkFun_Qwiic_KX13X.h"
arduino::MbedI2C wire(18,19);
QwiicKX134 IMU;
outputData a;

void setup() {
  Serial.begin(9600);
  while (!Serial){delay(10);}
  Serial.println("In kx");
  wire.begin();
  Serial.println("IMU BEGIN CODE");
  Serial.println(IMU.begin(0x1e, wire));
  pinMode(PWM_BUZZ, OUTPUT);
  IMU.initialize();

  float f1 = 261.63;
  float f2 = 392;
    // analogWrite(PWM_BUZZ, 128);

  // while(true){
  //   auto t = micros();
  //   bool a = fmod(t, f1) > f1/2;
  //   bool b = fmod(t, f2) > f2/2;
  //   digitalWrite(PWM_BUZZ, a|b);
  // }
  // for (int i = 8; i < 512; i++) {
  //   Serial.println(i);
  //   for(int j = 0; j < 10000; j++){
  //     delayMicroseconds(1000000/(1<<i));
  //     digitalWrite(PWM_BUZZ, HIGH);
  //     delayMicroseconds(1000000/(1<<i));
  //     digitalWrite(PWM_BUZZ, LOW);
  //   }
  //   // analogWrite(PWM_BUZZ, i);
  //   // delay(100);
  //   // analogWrite(PWM_BUZZ, 0);  
  //   // delay(100);
  // }
  // digitalWrite(PWM_BUZZ, HIGH);

}

void loop() {
  a = IMU.getAccelData();
  // Serial.println(a.yData);
  // Serial.println(a.xData);
  Serial.println(a.zData);
  delay(20);

}

#endif KX134


#ifdef BUZZER

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0


// change this to make the song slower or faster
int tempo = 140;

// change this to whichever pin you want to use
int buzzer = 11;

// notes of the moledy followed by the duration.
// a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
// !!negative numbers are used to represent dotted notes,
// so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!
int melody[] = {
  
  // Cantina BAnd - Star wars 
  // Score available at https://musescore.com/user/6795541/scores/1606876
  NOTE_B4,-4, NOTE_E5,-4, NOTE_B4,-4, NOTE_E5,-4, 
  NOTE_B4,8,  NOTE_E5,-4, NOTE_B4,8, REST,8,  NOTE_AS4,8, NOTE_B4,8, 
  NOTE_B4,8,  NOTE_AS4,8, NOTE_B4,8, NOTE_A4,8, REST,8, NOTE_GS4,8, NOTE_A4,8, NOTE_G4,8,
  NOTE_G4,4,  NOTE_E4,-2, 
  NOTE_B4,-4, NOTE_E5,-4, NOTE_B4,-4, NOTE_E5,-4, 
  NOTE_B4,8,  NOTE_E5,-4, NOTE_B4,8, REST,8,  NOTE_AS4,8, NOTE_B4,8,

  NOTE_A4,-4, NOTE_A4,-4, NOTE_GS4,8, NOTE_A4,-4,
  NOTE_D5,8,  NOTE_C5,-4, NOTE_B4,-4, NOTE_A4,-4,
  NOTE_B4,-4, NOTE_E5,-4, NOTE_B4,-4, NOTE_E5,-4, 
  NOTE_B4,8,  NOTE_E5,-4, NOTE_B4,8, REST,8,  NOTE_AS4,8, NOTE_B4,8,
  NOTE_D5,4, NOTE_D5,-4, NOTE_B4,8, NOTE_A4,-4,
  NOTE_G4,-4, NOTE_E4,-2,
  NOTE_E4, 2, NOTE_G4,2,
  NOTE_B4, 2, NOTE_D5,2,

  NOTE_F5, -4, NOTE_E5,-4, NOTE_AS4,8, NOTE_AS4,8, NOTE_B4,4, NOTE_G4,4, 
  
  
  

};

// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
int notes = sizeof(melody) / sizeof(melody[0]) / 2;

// this calculates the duration of a whole note in ms
int wholenote = (60000 * 2) / tempo;

int divider = 0, noteDuration = 0;

void setup() {
  // iterate over the notes of the melody. 
  // Remember, the array is twice the number of notes (notes + durations)
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(buzzer, 100, 100);

    // Wait for the specief duration before playing the next note.
    delay(100);
    
    // stop the waveform generation before the next note.
    noTone(buzzer);
    delay(10);
  }
}

void loop() {}

#endif

#ifdef TLM
#include <SPI.h>
#include <RH_RF95.h>

//Make sure to change these pinout depending on wiring
//Don't forget to change the ini file to build the correct main file
#define RFM95_CS 1
#define RFM95_RST 5
#define RFM95_INT 4
// #define RFM95_EN 1

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 440.0

#define MAX_CMD_LEN 10

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


//For reading from 
char incomingCmd[MAX_CMD_LEN];

// input value for sine function
double dummy_input = 0; 



// Data transmitted from rocket to ground station
struct telemetry_data {
  double gps_lat;
  double gps_long;
  double gps_alt;
  double barometer_alt;
  // KX134 (highg) IMU DATA
  double KX_IMU_ax;   // acceleration (in G's)
  double KX_IMU_ay;
  double KX_IMU_az;
  // H3LIS331DL (highg) IMU DATA
  double H3L_IMU_ax;
  double H3L_IMU_ay;
  double H3L_IMU_az;
  // LSM9DS1 (lowg) IMU DATA
  double LSM_IMU_ax;    // acceleration (in G's)
  double LSM_IMU_ay;
  double LSM_IMU_az;
  double LSM_IMU_gx;    // Gyro data (in degrees/sec)
  double LSM_IMU_gy;
  double LSM_IMU_gz;
  double LSM_IMU_mx;
  double LSM_IMU_my;
  double LSM_IMU_mz;
  
  int FSM_state;
  char sign[8] = "HITHERE";
  int rssi;
  double battery_voltage;
  int response_ID;
};

// Commands transmitted from ground station to rocket
enum CommandType {
  SET_FREQ,
  SET_CALLSIGN,
  ABORT,
  EMPTY
};

struct telemetry_command {
  CommandType command;
  int cmd_id;
  union {
    char callsign[8];
    int freq;
    bool do_abort;
  };
};

void setup() 
{
  Serial.begin(9600);
  Serial.println("Telemetry Test");
  pinMode(RFM95_RST, OUTPUT);
  // pinMode(RFM95_EN, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  // digitalWrite(RFM95_EN, HIGH);
 
  //Serial.begin(9600, SERIAL_8N1, 19, 20);
  //select ports to use for serial (usb are 19, 20?)
  //while(!Serial);



 

  

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("Radio Initialization Failed");
    // neopixelWrite(RGB_BUILTIN, 255,0,0); //red led if radio failed
    // delay(1000);
    // neopixelWrite(RGB_BUILTIN, 0,0,0);
    while (1);
  }
  // neopixelWrite(RGB_BUILTIN, 0,255,0); //green led if radio success
  delay(1000);
  // neopixelWrite(RGB_BUILTIN, 0,0,0);
  Serial.println("Radio Initialized");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency Failed");
    while (1);
  }
  Serial.print("Frequency set to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission
int last_command_id = -1;
char callsign[8] = "NO SIGN";

struct {
  bool should_change{};
  int new_freq{};
} freq_status;

void handle_command(const telemetry_command & cmd){
/* Check if lasted command ID matched current command ID */
      if(last_command_id == cmd.cmd_id){
        return;
      }
      last_command_id = cmd.cmd_id;
      if (cmd.command == SET_FREQ) {
        freq_status.should_change = true;
        freq_status.new_freq = cmd.freq;
      } 

      if (cmd.command == SET_CALLSIGN) {
        memcpy(callsign, cmd.callsign, sizeof(cmd.callsign));
      }
      
      Serial.println("Got Commands:");
      Serial.print("Call Sign: ");
      Serial.println(cmd.callsign);

      Serial.print("Abort? ");
      Serial.println(cmd.do_abort);
      Serial.print("Frequency: ");
      Serial.println(cmd.freq);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);  
}

void loop()
{
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  
  
  telemetry_data d{};

  // Looping input value from 0 to 2pi over and over 
  if (dummy_input > 628) {
    dummy_input = 0;
  } else {
    dummy_input+=30;
  }

  // Computing sine value
  double sin_value = sin(dummy_input/100);
  double cos_value = cos(dummy_input/100);
  double tan_value = tan(dummy_input/100);


  // Setting each sensor value to the current sine value
  d.gps_lat=sin_value;
  d.gps_long=cos_value;
  d.gps_alt=-1 * dummy_input/100;
  d.barometer_alt=sin_value;
  d.KX_IMU_ax=cos_value;
  d.KX_IMU_ay=sin_value;
  d.KX_IMU_az=dummy_input/100;
  d.H3L_IMU_ax=sin_value;
  d.H3L_IMU_ay=cos_value;
  d.H3L_IMU_az=sin_value+cos_value;
  d.LSM_IMU_ax=cos_value;    
  d.LSM_IMU_ay=sin_value;
  d.LSM_IMU_az=((dummy_input/100)*(dummy_input/100))/2;
  d.LSM_IMU_gx=tan_value;    
  d.LSM_IMU_gy=sin_value;
  d.LSM_IMU_gz=cos_value;
  d.LSM_IMU_mx=sin_value;
  d.LSM_IMU_my=sin_value;
  d.LSM_IMU_mz=sin_value;

  d.rssi = rf95.lastRssi();

  d.response_ID = last_command_id;
  memcpy(d.sign, callsign, sizeof(callsign));
  
  Serial.println("Sending sample sensor data..."); delay(10);
  rf95.send((uint8_t *)&d, sizeof(d));

  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();

  //change the freqencey after we acknowledge
  if(freq_status.should_change){
    Serial.println(freq_status.new_freq);
    rf95.setFrequency(freq_status.new_freq);
    freq_status.should_change = false;
  }
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
                                          //test without delay
  Serial.println("Waiting for reply..."); //delay(10);
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
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
  delay(100);
}
#endif

#ifdef SD_THING

#include <SPI.h>
#include <SD.h>

File myFile;
arduino::MbedSPI SD_SPI(12, 15, 14);
void setup() {
  // Open serial communications and wait for port to open:
  SD_SPI.begin();
  // SD_SPI.begin();
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(13)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void loop() {
  // nothing happens after setup
}

#endif