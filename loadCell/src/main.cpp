#include "HX711.h"
#include <SPI.h>
#include <SD.h>

//File myFile;
HX711 loadcell;

// 1. HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 0;
const int LOADCELL_SCK_PIN = 1;
const int buzzer = 2;

// 2. Adjustment settings
const long LOADCELL_OFFSET = 50682624;
const long LOADCELL_DIVIDER = 5895655;

//create file for SD card
File myFile;
char fileName[12];
float base;

void setup() {
  SPI.begin();
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.print("Initializing SD card...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    while (1);
  }
  strcpy(fileName, "loadCell.csv");

  Serial.println("initialization done.");
  myFile = SD.open(fileName, FILE_WRITE);
  
  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  loadcell.set_scale();

  //setting the base for the loadcell. Differences in readings will determine weight calculations
  float x = 0;
  float i = 0;
  float intervals = 250;
  while (i < intervals) {
    Serial.print((i/intervals) * 100);
    Serial.println(" %");
    x += loadcell.get_units();
    i++;
  }
  base = x / intervals;

  // loadcell.set_offset(LOADCELL_OFFSET);
}


void loop() {
  // Serial.print("Weight: ");
  Serial.print("Weight: ");
  float reading = ((loadcell.get_units() - base) * 11/34) / 1000;



  Serial.println(reading);

  if (myFile) {
    myFile.print(reading);
    myFile.print(",");

    Serial.println("got here");

    myFile.println("doot");
    myFile.flush();
  }
}