#ifndef SERIALPARSER_H
#define SERIALPARSER_H

#include <Arduino.h>

typedef void (*SuccessCallback)(const char* key, const char* value);
typedef void (*ErrorCallback)();

class SerialParser {
   public:
    /*
     * SuccessCallback : (const char * key, const char * value) -> void
     * ErrorCallback : () -> void
     */
    SerialParser(SuccessCallback success, ErrorCallback error) : success(success), error(error) {}
    void read() {
        while (Serial.available()) {
            char c = Serial.read();
            if (write_head < 256) {
                input_buffer[write_head++] = c;
            } else {
                write_overflow = true;
            }
            if (c == terminator) {
                if (write_overflow) {
                    error();
                } else {
                    char* seperator_loc = (char*)memchr(input_buffer, seperator, write_head);
                    if (seperator_loc == nullptr) {
                        error();
                    } else {
                        // set null terminators after the key and value so that
                        // they are seperate c strings
                        *seperator_loc = '\0';
                        input_buffer[write_head - 1] = '\0';
                        success(input_buffer, seperator_loc + 1);
                    }
                }

                write_head = 0;
                write_overflow = false;
            }
        }
    }

   private:
    static constexpr char terminator = '\n';
    static constexpr char seperator = ' ';
    SuccessCallback success;
    ErrorCallback error;
    char input_buffer[256]{};
    int write_head{};
    bool write_overflow{};
};

#endif