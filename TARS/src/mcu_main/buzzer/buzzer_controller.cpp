#include "buzzer_controller.h"

#include "mcu_main/buzzer/notes.h"

uint16_t note_frequencies[] = {
    31,
    33,
    35,
    37,
    39,
    41,
    44,
    46,
    49,
    52,
    55,
    58,
    62,
    65,
    69,
    73,
    78,
    82,
    87,
    93,
    98,
    104,
    110,
    117,
    123,
    131,
    139,
    147,
    156,
    165,
    175,
    185,
    196,
    208,
    220,
    233,
    247,
    262,
    277,
    294,
    311,
    330,
    349,
    370,
    392,
    415,
    440,
    466,
    494,
    523,
    554,
    587,
    622,
    659,
    698,
    740,
    784,
    831,
    880,
    932,
    988,
    1047,
    1109,
    1175,
    1245,
    1319,
    1397,
    1480,
    1568,
    1661,
    1760,
    1865,
    1976,
    2093,
    2217,
    2349,
    2489,
    2637,
    2794,
    2960,
    3136,
    3322,
    3520,
    3729,
    3951,
    4186,
    4435,
    4699,
    4978,
    0
};


BuzzerController::BuzzerController(int pin_, int8_t const* sequences_) : pin(pin_), sequences(sequences_) { }

void BuzzerController::playSequence(size_t sequence) {
    idx = sequence;
    playing = true;
}

void BuzzerController::tick() {
    if (!playing) return;

    uint16_t note = note_frequencies[sequences[idx]];
    if (sequences[idx] == STOP) {
        playing = false;
        return;
    }
    int8_t duration_code = sequences[idx+1];

    int note_duration;
    if (duration_code > 0) {
        note_duration = (60000 * 2 / 140) / duration_code;
    } else if (duration_code < 0) {
        note_duration = (60000 * 2 / 140) / -duration_code * 1.5;
    }
    tone(pin, note, note_duration*0.9);
    chThdSleep(note_duration);
    noTone(pin);
}


