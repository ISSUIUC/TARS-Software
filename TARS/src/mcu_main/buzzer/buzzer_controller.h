#pragma once

#include <ChRt.h>

class BuzzerController {
public:
    explicit BuzzerController(int pin, int8_t const* sequences);

    void playSequence(size_t sequence);

    void tick();

private:
    int pin;
    int8_t const* sequences;
    size_t idx = 0;
    bool playing = false;
};


