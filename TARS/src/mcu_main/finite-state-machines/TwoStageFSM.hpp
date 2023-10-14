#pragma once

#include "mcu_main/Abort.h"
#include "mcu_main/dataLog.h"
#include "mcu_main/finite-state-machines/RocketFSMBase.h"
#include "mcu_main/finite-state-machines/thresholds.h"
#include "mcu_main/sensors/sensors.h"

template <size_t view>
class TwoStageFSM : public RocketFSMBase {
   public:
    TwoStageFSM() = default;

   private:
    systime_t launch_time_ = 0;
    sysinterval_t burn_timer_ = 0;
    systime_t burnout_time_ = 0;
    systime_t burnout_time_ = 0;
    systime_t coast_time_ = 0;
    sysinterval_t coast_timer_ = 0;

    systime_t second_boost_time_ = 0;

    systime_t apogee_time_ = 0;
    sysinterval_t apogee_timer_ = 0;

    systime_t drogue_time_ = 0;
    sysinterval_t drogue_timer_ = 0;

    systime_t main_time_ = 0;
    sysinterval_t main_timer_ = 0;

    systime_t landing_time_ = 0;
    sysinterval_t landing_timer = 0;


    double getAltitudeAverage(size_t start, size_t len) {
        return TwoStageFSM::getAverage(
            dataLogger.barometerFifo, +[](BarometerData& b) { return (double)b.altitude; }, start, len);
    }

    double getSecondDerivativeAltitudeAverage(size_t start, size_t len) {
        return TwoStageFSM::getSecondDerivativeAverage(
            dataLogger.barometerFifo, +[](BarometerData& b) { return (double)b.altitude; },
            +[](BarometerData& b) { return b.timeStamp_barometer; }, start, len);
    }

    double getAccelerationAverage(size_t start, size_t len) {
        return TwoStageFSM::getAverage(
            dataLogger.highGFifo, +[](HighGData& g) { return (double)g.hg_az; }, start, len);
    }

   public:
    /**
     * @brief TwoStageFSM tick function
     *
     * Uses a combination of linear acceleration and timers to govern FSM state
     * changes for each timestep of the rocket's flight.
     */
    void tickFSM() override {
        // Lock mutexes for data used in switch
        chMtxLock(&highG.mutex);

        // Serial.println(state_map[(int)rocket_state_]);

        // Links to abort for other states
        if (isAborted()) {
            rocket_state_ = FSM_State::STATE_ABORT;
        }

        switch (rocket_state_) {
            case FSM_State::STATE_ABORT:
                // If true, always stay in abort
                break;

            case FSM_State::STATE_INIT:
                // Go to state idle regardless of gps lock
                rocket_state_ = FSM_State::STATE_IDLE;
                break;

            case FSM_State::STATE_IDLE:
                // If high acceleration is observed in z direction...
                if (highG.getAccel().az > launch_linear_acceleration_thresh) {
                    launch_time_ = chVTGetSystemTime();
                    rocket_state_ = FSM_State::STATE_FIRST_BOOST;
                }

                break;

            // case FSM_State::STATE_LAUNCH_DETECT:
            //     // If the acceleration was too brief, go back to IDLE
            //     if (highG.getAccel().az < launch_linear_acceleration_thresh) {
            //         rocket_state_ = FSM_State::STATE_IDLE;
            //         break;
            //     }

            //     // Measure the length of the burn time (for hysteresis)
            //     burn_timer_ = chVTGetSystemTime() - launch_time_;

            //     // If the acceleration lasts long enough, boost is detected
            //     if (TIME_I2MS(burn_timer_) > launch_time_thresh) {
            //         rocket_state_ = FSM_State::STATE_BOOST;
            //     }

            //     break;

            case FSM_State::STATE_FIRST_BOOST:// figure out how to add this state
                //If the acceleration was too brief, go back to IDLE
                if (highG.getAccel().az < launch_linear_acceleration_thresh && chVTGetSystemTime() - launch_time_ < 1000) {
                    rocket_state_ = FSM_State::STATE_IDLE;
                    break;
                }

                burn_timer_ = chVTGetSystemTime() - launch_time_;
                // If low acceleration in the Z direction...
                if (highG.getAccel().az < coast_thresh) {
                    // Serial.println("Acceleration below thresh");
                    burnout_time_ = chVTGetSystemTime();
                    rocket_state_ = FSM_State::STATE_BURNOUT_DETECT;
                    break;
                }
                // Keeping rocket in FSM_State::FIRST_BOOST if time below a certain
                // threshold
                if (TIME_I2MS(burn_timer_) < burn_time_thresh_ms) {
                    rocket_state_ = FSM_State::FIRST_BOOST;
                } else {  // Forcing rocket to go to FSM_State::STATE_COAST if threshold crossed
                    rocket_state_ = FSM_State::STATE_BURNOUT_DETECT;
                    // Setting burnout time because we don't otherwise
                    burnout_time_ = chVTGetSystemTime();
                }

                break;

            case FSM_State::STATE_BURNOUT_DETECT:
                // If the 0 acceleration was too brief, go back to BOOST
                if (highG.getAccel().az > coast_thresh && chVTGetSystemTime() - burnout_time_ < 1000) {
                    rocket_state_ = FSM_State::STATE_BOOST;
                    break;
                }


                // Measure the length of the coast time (for hysteresis)
                coast_timer_ = chVTGetSystemTime() - burnout_time_;

                //seperate first part safely
                if (TIME_I2MS(coast_timer_) > seperate_time_thresh) {  // 1000
                    // seperate the rocket
                }

                // hardcoing waiting 2 seconds before transitioning
                if (TIME_I2MS(coast_timer_) > burnout_time_thresh) { // 2500 * do we need to change this?
                    rocket_state_ = FSM_State::STATE_SUSTAINER_IGNITION;
                }

                break;

            case FSM_State::STATE_SUSTAINER_IGNITION:
                // fire ignition
                // ARE WE RESPONSIBLE OR THE PYRO TEAM?
                // ?????!?!?!?!?!?!?!?!?!?

                // If high acceleration is observed in z direction...
                if (highG.getAccel().az > first_seperation_linear_acceleration_thresh) {
                    second_boost_time_ = chVTGetSystemTime();
                    rocket_state_ = FSM_State::STATE_SECOND_BOOST;
                }

                break;

            case FSM_State::STATE_SECOND_BOOST:// figure out how to add this state
                //If the acceleration was too brief, go back to IDLE
                if (highG.getAccel().az < first_seperation_linear_acceleration_thresh && chVTGetSystemTime() - second_boost_time_ < 1000) {
                    rocket_state_ = FSM_State::STATE_SUSTAINER_IGNITION;
                    break;
                }

                if (highG.getAccel().az < coast_thresh) {
                    // Serial.println("Acceleration below thresh");
                    coast_time_ = chVTGetSystemTime();
                    rocket_state_ = FSM_State::STATE_COAST;
                    break;
                }

                break;

            case FSM_State::STATE_COAST:
                if (highG.getAccel().az > coast_thresh && chVTGetSystemTime() - coast_time_ < 1000) {
                    // Serial.println("Acceleration below thresh");
                    rocket_state_ = FSM_State::STATE_SECOND_BOOST;
                    break;
                }


                coast_timer_ = chVTGetSystemTime() - coast_time_;

                if (fabs(getAltitudeAverage(0, view / 2) - getAltitudeAverage(view / 2, view / 2)) <
                    apogee_altimeter_threshold) {
                    rocket_state_ = FSM_State::STATE_APOGEE;
                    apogee_time_ = chVTGetSystemTime();
                    break;
                }

                if (TIME_I2MS(coast_timer_) < coast_to_apogee_time_thresh) {
                    rocket_state_ = FSM_State::STATE_COAST;
                } else {
                    rocket_state_ = FSM_State::STATE_APOGEE;
                    apogee_time_ = chVTGetSystemTime();
                }

                break;


                // LEFT OFF HERE

            case FSM_State::STATE_APOGEE_DETECT:
                // If the 0 velocity was too brief, go back to coast
                if (fabs(getAltitudeAverage(0, view / 2) - getAltitudeAverage(view / 2, view / 2)) >
                    apogee_altimeter_threshold) {
                    rocket_state_ = FSM_State::STATE_COAST_GNC;
                    break;
                }

                // Measure the length of the apogee time (for hysteresis)
                apogee_timer_ = chVTGetSystemTime() - apogee_time_;

                // If the low velocity lasts long enough, apogee is detected
                if (TIME_I2MS(apogee_timer_) > apogee_time_thresh) {
                    rocket_state_ = FSM_State::STATE_APOGEE;
                }

                break;

            case FSM_State::STATE_APOGEE:
                if (fabs(getAccelerationAverage(0, view / 2) - getAccelerationAverage(view / 2, view / 2)) >
                    drogue_acceleration_change_threshold_imu) {
                    rocket_state_ = FSM_State::STATE_DROGUE_DETECT;
                    break;
                }
                // potentially add back state to put us back into coast

                if (TIME_I2MS(apogee_timer_) < drogue_deploy_time_since_apogee_threshold) {
                    rocket_state_ = FSM_State::STATE_APOGEE;
                } else {
                    rocket_state_ = FSM_State::STATE_DROGUE;
                    drogue_time_ = chVTGetSystemTime();
                }
                break;

            case FSM_State::STATE_DROGUE_DETECT:
                if (fabs(getSecondDerivativeAltitudeAverage(0, view / 2) -
                         getSecondDerivativeAltitudeAverage(view / 2, view / 2)) >
                    drogue_acceleration_change_threshold_altimeter) {
                    rocket_state_ = FSM_State::STATE_DROGUE;
                    drogue_time_ = chVTGetSystemTime();
                    break;
                } else {
                    rocket_state_ = FSM_State::STATE_APOGEE;
                }
                break;

            case FSM_State::STATE_DROGUE:
                drogue_timer_ = chVTGetSystemTime() - drogue_time_;
                if (TIME_I2MS(drogue_timer_) > refresh_timer) {
                    if (fabs(getAccelerationAverage(0, view / 2) - getAccelerationAverage(view / 2, view / 2)) >
                        main_acceleration_change_threshold_imu) {
                        rocket_state_ = FSM_State::STATE_MAIN_DETECT;
                        break;
                    }
                }

                if (TIME_I2MS(drogue_timer_) < main_deploy_time_since_drogue_threshold) {
                    rocket_state_ = FSM_State::STATE_DROGUE;

                } else {
                    rocket_state_ = FSM_State::STATE_MAIN;
                    main_time_ = chVTGetSystemTime();
                }
                break;

            case FSM_State::STATE_MAIN_DETECT:
                if (fabs(getSecondDerivativeAltitudeAverage(0, view / 2) -
                         getSecondDerivativeAltitudeAverage(view / 2, view / 2)) >
                    main_acceleration_change_threshold_altimeter) {
                    rocket_state_ = FSM_State::STATE_MAIN;
                    main_time_ = chVTGetSystemTime();
                    break;
                } else {
                    rocket_state_ = FSM_State::STATE_DROGUE;
                }
                break;

            case FSM_State::STATE_MAIN:
                main_timer_ = chVTGetSystemTime() - main_time_;

                if (fabs(getAltitudeAverage(0, view / 2) - getAltitudeAverage(view / 2, view / 2)) <
                    landing_altimeter_threshold) {
                    rocket_state_ = FSM_State::STATE_LANDED_DETECT;
                    landing_time_ = chVTGetSystemTime();
                    break;
                }

                if (TIME_I2MS(main_timer_) < main_deploy_time_since_drogue_threshold) {
                    rocket_state_ = FSM_State::STATE_MAIN;
                } else {
                    rocket_state_ = FSM_State::STATE_LANDED;
                    landing_time_ = chVTGetSystemTime();
                }
                break;

            case FSM_State::STATE_LANDED_DETECT:
                // If the 0 velocity was too brief, go back to main
                if (fabs(getAltitudeAverage(0, view / 2) - getAltitudeAverage(view / 2, view / 2)) >
                    landing_altimeter_threshold) {
                    rocket_state_ = FSM_State::STATE_MAIN;
                    break;
                }

                // Measure the length of the landed time (for hysteresis)
                landing_timer = chVTGetSystemTime() - landing_time_;

                // If the low velocity lasts long enough, landing is detected
                if (TIME_I2MS(landing_timer) > landing_time_thresh) {
                    rocket_state_ = FSM_State::STATE_LANDED;
                }

                break;

            case FSM_State::STATE_LANDED:
            default:
                break;
        }

        // Unlock mutexes used during the switch statement
        chMtxUnlock(&highG.mutex);
    }
};
