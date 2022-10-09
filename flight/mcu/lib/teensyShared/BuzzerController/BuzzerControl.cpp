#include "BuzzerControl.hpp"
#include "pins.h"


BuzzerController::BuzzerController(pointers* rocket_state) :
        control_state(WAITING_FOR_BATTERY),
        curr_state(INITIAL),
        state_start_time(0),
        rocket_state(rocket_state),
        last_rocket_fsm_state(STATE_INIT) {
}

void BuzzerController::setBuzzerState(BuzzerState new_state) {
    auto new_pitch = BuzzerStates[new_state][0];
    noTone(BUZZER_PORT);
    if (new_pitch) {
        tone(BUZZER_PORT, new_pitch);
    }
    state_start_time = millis();
    curr_state = new_state;
}


BuzzerState BuzzerController::getNewStateFromBattery(float battery_voltage) {
    switch ((int) (battery_voltage / 3.3 * 10.0)) {
        case 0:
            return BuzzerState::BATTERY_ZERO_STATE_0;
        case 1:
            return BuzzerState::BATTERY_ONE_STATE_0;
        case 2:
            return BuzzerState::BATTERY_TWO_STATE_0;
        case 3:
            return BuzzerState::BATTERY_THREE_STATE_0;
        case 4:
            return BuzzerState::BATTERY_FOUR_STATE_0;
        case 5:
            return BuzzerState::BATTERY_FIVE_STATE_0;
        case 6:
            return BuzzerState::BATTERY_SIX_STATE_0;
        case 7:
            return BuzzerState::BATTERY_SEVEN_STATE_0;
        case 8:
            return BuzzerState::BATTERY_EIGHT_STATE_0;
        case 9:
            return BuzzerState::BATTERY_NINE_STATE_0;
        default:
            return BuzzerState::INITIAL;
    }
}


BuzzerState BuzzerController::getNewStateFromRocket(FSM_State rocket_state) {
    // Notably, not all stages have a beep sequence, so the default case happens pretty often
    switch (rocket_state) {
        case STATE_IDLE:
            return BuzzerState::BUZZ_IDLE_STATE_0;
        case STATE_BOOST:
            return BuzzerState::BUZZ_BOOST_STATE_0;
        case STATE_COAST:
            return BuzzerState::BUZZ_COAST_STATE_0;
        case STATE_APOGEE:
            return BuzzerState::BUZZ_APOGEE_STATE_0;
        case STATE_DROGUE:
            return BuzzerState::BUZZ_DROGUE_STATE_0;
        case STATE_MAIN:
            return BuzzerState::BUZZ_MAIN_STATE_0;
        case STATE_LANDED:
            return BuzzerState::BUZZ_LANDED_STATE_0;
        case STATE_ABORT:
            return BuzzerState::BUZZ_ABORT_STATE_0;
        default:
            return BuzzerState::INITIAL;
    }
}

void BuzzerController::tickBuzzer() {
    // The main purpose of the control state is to allow the battery voltage to be beeped out before the FSM state gets beeped out
    switch (control_state) {
        case WAITING_FOR_BATTERY:
            // This state waits for some battery voltage to be read in
            if (rocket_state->sensorDataPointer->has_voltage_data) {
                setBuzzerState(getNewStateFromBattery(rocket_state->sensorDataPointer->voltage_data.v_battery));
                control_state = SENDING_BATTERY;
            }
            break;
        case SENDING_BATTERY:
            // This state waits for the battery voltage to finished being read out
            if (curr_state == INITIAL) {
                control_state = SENDING_FSM_STATE;
            }
            break;
        case SENDING_FSM_STATE:
            // This is the core state, waits for the FSM state to change and sets the buzzer state accordingly
            chMtxLock(&rocket_state->dataloggerTHDVarsPointer.dataMutex_rocket_state);
            FSM_State rocket_fsm_state = rocket_state->sensorDataPointer->rocketState_data.rocketState;
            chMtxUnlock(&rocket_state->dataloggerTHDVarsPointer.dataMutex_rocket_state);

            if (rocket_fsm_state != last_rocket_fsm_state) {
                BuzzerState new_state = getNewStateFromRocket(rocket_fsm_state);
                if (new_state != BuzzerState::INITIAL) {
                    last_rocket_fsm_state = rocket_fsm_state;
                    setBuzzerState(new_state);
                }
            }
            break;
    }

    // This is the part of the code which advances the buzzer FSM when needed
    unsigned int* state_info = BuzzerStates[curr_state];

    unsigned long curr_time = millis();
    unsigned long elapsed = curr_time - state_start_time;

    if (elapsed >= state_info[1]) {
        auto new_state = static_cast<BuzzerState>(state_info[2]);
        setBuzzerState(new_state);
    }
}
