#include "Arduino.h"
#include "SPI.h"

class KX134 {
    public:
        //methods
        void update_data();
        int16_t binary_to_decimal();

        int16_t get_x_accel_raw();
        int16_t get_y_accel_raw();
        int16_t get_z_accel_raw();

        float get_x_gforce();
        float get_y_gforce();
        float get_z_gforce();

        float get_x_accel();
        float get_y_accel();
        float get_z_accel();

        macro KX134_CS_PIN;

        //classes
        KX134();

    private:
        //variables
        uint16_t x_accel;
        uint16_t y_accel;
        uint16_t z_accel;

        //classes
        void init();
_}
