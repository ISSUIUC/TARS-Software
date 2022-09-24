#define BUZZER_PORT 11

#include "BuzzerControl.hpp"

unsigned long millis();
void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);
void noTone(uint8_t _pin);

BuzzerController::BuzzerController(pointers* rocket_state) :
    curr_state(INITIAL), time_since_state_start(0), rocket_state(rocket_state), last_rocket_fsm_state(STATE_INIT) {
}

void BuzzerController::setBuzzerState(BuzzerState new_state) {
    auto new_pitch = BuzzerStates[new_state][0];
    noTone(BUZZER_PORT);
    if (new_pitch) {
        tone(BUZZER_PORT, new_pitch);
    }
    time_since_state_start = millis();
    curr_state = new_state;
}

void BuzzerController::tickBuzzer() {
    unsigned int* state_info = BuzzerStates[curr_state];

    chMtxLock(&rocket_state->dataloggerTHDVarsPointer.dataMutex_rocket_state);
    auto rocket_fsm_state = rocket_state->sensorDataPointer->rocketState_data.rocketState;
    chMtxUnlock(&rocket_state->dataloggerTHDVarsPointer.dataMutex_rocket_state);

    if (rocket_fsm_state != last_rocket_fsm_state) {
        switch (rocket_fsm_state) {
            case STATE_IDLE:
                last_rocket_fsm_state = rocket_fsm_state;
                setBuzzerState(BuzzerState::BUZZ_IDLE_STATE_0);
                break;
            case STATE_BOOST:
                last_rocket_fsm_state = rocket_fsm_state;
                break;
            case STATE_COAST:
                last_rocket_fsm_state = rocket_fsm_state;
                break;
            case STATE_APOGEE:
                last_rocket_fsm_state = rocket_fsm_state;
                break;
            case STATE_DROGUE:
                last_rocket_fsm_state = rocket_fsm_state;
                break;
            case STATE_MAIN:
                last_rocket_fsm_state = rocket_fsm_state;
                break;
            case STATE_LANDED:
                last_rocket_fsm_state = rocket_fsm_state;
                break;
            case STATE_ABORT:
                last_rocket_fsm_state = rocket_fsm_state;
                break;
            default:
                break;
        }
    }

    unsigned long curr_time = millis();
    unsigned long elapsed = curr_time - time_since_state_start;

    if (elapsed >= state_info[1]) {
        auto new_state = static_cast<BuzzerState>(state_info[2]);
        setBuzzerState(new_state);
    }
}
