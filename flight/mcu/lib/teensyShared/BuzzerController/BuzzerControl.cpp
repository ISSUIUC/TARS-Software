#include "BuzzerControl.hpp"
#include "pins.h"


BuzzerController::BuzzerController(pointers* rocket_state) :
    control_state(WAITING_FOR_BATTERY),
    curr_state(INITIAL),
    time_since_state_start(0),
    rocket_state(rocket_state),
    last_rocket_fsm_state(STATE_INIT) {
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
    switch (control_state) {
        case WAITING_FOR_BATTERY:
            if (rocket_state->sensorDataPointer->has_voltage_data) {
                control_state = SENDING_BATTERY;
                BuzzerState state = BuzzerState::INITIAL;
                switch ((int) (rocket_state->sensorDataPointer->voltage_data.v_battery / 3.3 * 10.0)) {
                    case 0:
                        state = BuzzerState::BATTERY_ZERO_STATE_0;
                        break;
                    case 1:
                        state = BuzzerState::BATTERY_ONE_STATE_0;
                        break;
                    case 2:
                        state = BuzzerState::BATTERY_TWO_STATE_0;
                        break;
                    case 3:
                        state = BuzzerState::BATTERY_THREE_STATE_0;
                        break;
                    case 4:
                        state = BuzzerState::BATTERY_FOUR_STATE_0;
                        break;
                    case 5:
                        state = BuzzerState::BATTERY_FIVE_STATE_0;
                        break;
                    case 6:
                        state = BuzzerState::BATTERY_SIX_STATE_0;
                        break;
                    case 7:
                        state = BuzzerState::BATTERY_SEVEN_STATE_0;
                        break;
                    case 8:
                        state = BuzzerState::BATTERY_EIGHT_STATE_0;
                        break;
                    case 9:
                        state = BuzzerState::BATTERY_NINE_STATE_0;
                        break;
                }
                setBuzzerState(state);
            }
            break;
        case SENDING_BATTERY:
            if (curr_state == INITIAL) {
                control_state = SENDING_FSM_STATE;
            }
            break;
        case SENDING_FSM_STATE:
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
                        setBuzzerState(BuzzerState::BUZZ_BOOST_STATE_0);
                        break;
                    case STATE_COAST:
                        last_rocket_fsm_state = rocket_fsm_state;
                        setBuzzerState(BuzzerState::BUZZ_COAST_STATE_0);
                        break;
                    case STATE_APOGEE:
                        last_rocket_fsm_state = rocket_fsm_state;
                        setBuzzerState(BuzzerState::BUZZ_APOGEE_STATE_0);
                        break;
                    case STATE_DROGUE:
                        last_rocket_fsm_state = rocket_fsm_state;
                        setBuzzerState(BuzzerState::BUZZ_DROGUE_STATE_0);
                        break;
                    case STATE_MAIN:
                        last_rocket_fsm_state = rocket_fsm_state;
                        setBuzzerState(BuzzerState::BUZZ_MAIN_STATE_0);
                        break;
                    case STATE_LANDED:
                        last_rocket_fsm_state = rocket_fsm_state;
                        setBuzzerState(BuzzerState::BUZZ_LANDED_STATE_0);
                        break;
                    case STATE_ABORT:
                        last_rocket_fsm_state = rocket_fsm_state;
                        setBuzzerState(BuzzerState::BUZZ_ABORT_STATE_0);
                        break;
                    default:
                        break;
                }
            }

            unsigned int* state_info = BuzzerStates[curr_state];

            unsigned long curr_time = millis();
            unsigned long elapsed = curr_time - time_since_state_start;

            if (elapsed >= state_info[1]) {
                auto new_state = static_cast<BuzzerState>(state_info[2]);
                setBuzzerState(new_state);
            }
            break;
    }
}
