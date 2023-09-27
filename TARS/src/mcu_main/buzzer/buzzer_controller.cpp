#include "buzzer_controller.h"

#include "mcu_main/buzzer/notes.h"

static constexpr uint16_t note_frequencies[] = {
    31,   33,   35,   37,   39,   41,   44,   46,   49,   52,   55,   58,   62,   65,   69,   73,   78,   82,
    87,   93,   98,   104,  110,  117,  123,  131,  139,  147,  156,  165,  175,  185,  196,  208,  220,  233,
    247,  262,  277,  294,  311,  330,  349,  370,  392,  415,  440,  466,  494,  523,  554,  587,  622,  659,
    698,  740,  784,  831,  880,  932,  988,  1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865,
    1976, 2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951, 4186, 4435, 4699, 4978, 0};

BuzzerController::BuzzerController(int pin_, int8_t const* sequences_) : pin(pin_), sequences(sequences_) {}

void BuzzerController::playSequence(size_t sequence) {
    idx = sequence;
    playing = true;
}

void BuzzerController::init_sponge() {
    float T = 400.0;
    // Storing notes for startup
    const int spongebob[] = {NOTE_C5, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_G4, NOTE_C5};

    // Storing delays as fraction of T where T represents a quarter note
    const float spongebob_delays[] = {T / 3, T / 3, T / 3, 0.75 * T, 0.25 * T, 0.75 * T, 0.25 * T, T};

    for (int i = 0; i < 8; i++) {
        tone(15, note_frequencies[spongebob[i]]);
        chThdSleepMilliseconds(spongebob_delays[i]);
        noTone(15);
    }
}

void BuzzerController::init_mario() {
    float T = 500.0;
    const int mario[] = {NOTE_C5, NOTE_C5, REST, NOTE_C5, REST, NOTE_GS4, NOTE_C5, NOTE_DS5, NOTE_DS4};

    const float mario_delays[] = {T / 4, T / 4, T / 4, T / 4, T / 4, T / 4, T / 2, T, T};
    for (int i = 0; i < 8; i++) {
        tone(15, note_frequencies[mario[i]]);
        chThdSleepMilliseconds(mario_delays[i]);
        noTone(15);
    }
}

void BuzzerController::tick() {
    if (!playing) return;

    if (sequences[idx] == STOP) {
        playing = false;
        return;
    }
    uint16_t note = note_frequencies[sequences[idx]];
    int8_t duration_code = sequences[idx + 1];

    float note_duration = 0.0;
    if (duration_code > 0) {
        note_duration = (60000.0f * 2 / 140) / (float)duration_code;
    } else if (duration_code < 0) {
        note_duration = (60000.0f * 2 / 140) / -(float)duration_code * 1.5f;
    }
    //    Serial.println(idx);
    tone(pin, note, note_duration * 0.9);
    chThdSleepMilliseconds((int)note_duration * 100);
    noTone(pin);
    idx += 2;
}
