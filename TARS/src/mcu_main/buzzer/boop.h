#pragma once

#include <mcu_main/buzzer/notes.h>

const int8_t melody[] = {
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

