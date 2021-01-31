//TODO: Figure out header guard?

#include "Arduino.h"
#include "SPI.h"

class KX134 {
    public:
        //methods
        void update_data();
        int16_t binary_to_decimal(int16_t);
        void init();

        int16_t get_x_accel_raw();
        int16_t get_y_accel_raw();
        int16_t get_z_accel_raw();

        float get_x_gforce();
        float get_y_gforce();
        float get_z_gforce();

        float get_x_accel();
        float get_y_accel();
        float get_z_accel();


        //classes
        KX134();

    private:
        //variables
        int16_t x_accel;
        int16_t y_accel;
        int16_t z_accel;

        //classes
};
