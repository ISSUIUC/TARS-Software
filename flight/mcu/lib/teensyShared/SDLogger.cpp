#include "SDLogger.h"
#include "pins.h"
#include "FS.h"

SDLogger sd_logger(dataLogger); // NOLINT(cppcoreguidelines-interfaces-global-init)

SDLogger::SDLogger(DataLogBuffer& buffer) : view(buffer) {}

#define MAX_FILES 999

/**
 * @brief Creates the name for a file to be written to SD card.
 *
 * @param fileName Pointer to char[] containing intended name of file. Do not
 * include number or file extension at end of name. Make sure this is longer
 * than it needs to be.
 * @param fileExtension Pointer to char[] containing the file extension for the
 * file.
 * @return char* Pointer to inputted char[]. It now contains number (if
 * duplicate file existed) and .csv file extension.
 */
char* sdFileNamer(char* fileName, char* fileExtensionParam) {
    char fileExtension[strlen(fileExtensionParam) + 1];
    strcpy(fileExtension, fileExtensionParam);

    char inputName[strlen(fileName) + 1];
    strcpy(inputName, fileName);

    strcat(fileName, fileExtension);

    // checks to see if file already exists and adds 1 to filename if it does.
    if (SD.exists(fileName)) {
        bool fileExists = false;
        int i = 1;
        while (!fileExists) {
            if (i > MAX_FILES) {
                // max number of files reached. Don't want to overflow
                // fileName[]. Will write new data to already existing
                // data999.csv
                strcpy(fileName, inputName);
                strcat(fileName, "999");
                strcat(fileName, fileExtension);
                break;
            }

            // converts int i to char[]
            char iStr[16];
            itoa(i, iStr, 10);

            // writes "(sensor)_data(number).csv to fileNameTemp"
            char fileNameTemp[strlen(inputName) + strlen(iStr) + 6];
            strcpy(fileNameTemp, inputName);
            strcat(fileNameTemp, iStr);
            strcat(fileNameTemp, fileExtension);

            if (!SD.exists(fileNameTemp)) {
                strcpy(fileName, fileNameTemp);
                fileExists = true;
            }

            i++;
        }
    }

    return fileName;
}

void SDLogger::init() {
    if (SD.begin(BUILTIN_SDCARD)) {
        char file_extension[6] = ".dat";

        char data_name[16] = "data";
        sdFileNamer(data_name, file_extension);
        // Initialize SD card
        sd_file = SD.open(data_name, FILE_WRITE_BEGIN);
        // print header to file on sd card that lists each variable that is logged
        sd_file.println("binary logging of sensor_data_t");
        sd_file.flush();

        Serial.println(sd_file.name());
    } else {
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_ORANGE, HIGH);
        digitalWrite(LED_BLUE, HIGH);
        Serial.println("SD Begin Failed. Stalling Program");
        while (true) {
            digitalWrite(LED_RED, HIGH);
            delay(100);
            digitalWrite(LED_RED, LOW);
            delay(100);
        }
    }
}

void SDLogger::update() {
    while (true) {
        sensorDataStruct_t current_data = view.read();
        if (!current_data.hasData()) {
            return;
        }

        logData(&current_data);
    }
}

template<typename T>
void SDLogger::logData(T* data) {
    sd_file.write((const uint8_t*) data, sizeof(T));
    // Flush data once for every 50 writes
    // Flushing data is the step that actually writes to the card
    // Flushing more frequently incurs more of a latency penalty, but less
    // potential data loss
    if (writes_since_flush >= 50) {
        sd_file.flush();
        writes_since_flush = 0;
    } else {
        writes_since_flush++;
    }
}

#undef MAX_FILES
