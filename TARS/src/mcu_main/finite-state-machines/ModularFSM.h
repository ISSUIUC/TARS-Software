/**
 * @file        ModularFSM.cpp
 *
 * @authors     Aidan Costello
 * 		        David Howard
 *              Siddhant Panse
 * 		        Caleb Peach
 *              Vinay Siva
 *  
 * @brief      The implementation of the finite state machine class that governs state transitions.
 *
 * The ModuarFSM class encapsulates the finite state machine that dictates which
 * state the rocket is in throughout the mission. The class implements the logic
 * necessary to reliably transition between states along with hysteresis to
 * avoid premature state transitions by utilizing a modular system. This allows
 * for new states to be easily added or removed by simply adding a state and
 * event check. This FSM utilizes an "unknown" state in order to handle unpredicted
 * actions of the rocket.
 * 
 * This is a highly critical software module and should be tested throughly in
 * simulation and on hardware targets.:
 *
 */



#pragma once

#include "mcu_main/finite-state-machines/RocketFSMBase.h"

class ModularFSM : public RocketFSMBase {
    public:
        ModularFSM() = default;

        void tickFSM() override;

    private:
        //coast time should always be set before we look at it
        systime_t coast_time_ = 0;

        //we want a default very high number in case we never reach our apogee
        float apogee_altitude_ = 15000.0;

        float launch_site_altitude_ = 0;

        //helps determine which checks to run
        FSM_State last_state_ = FSM_State::STATE_IDLE;

        //used to average data
        double getAltitudeAverage(size_t start, size_t len);
        double getAccelerationAverage(size_t start, size_t len);


        /**
         *  @brief Event check that checks if the rocket has moved out of the idle state
         * 
         *  By checking the linear accelertion in the upwards direction, we can determine 
         *  whether the rocket has accelerated past a determined theshold. 
         *  If so, we move to the boost state.
         */
        bool idleEventCheck();
        
        /**
         *  @brief State check that checks if the rocket is in the idle state
         * 
         *  By checking the altitude, acceleration, angle, and velocity of the
         *  rocket, we can determine if the rocket is in the idle state. If it 
         *  is not in the idle state, the rocket moves to the unknown state.
         */
        bool idleStateCheck();

        /**
         *  @brief Event check that checks if the rocket has moved out of the boost state
         * 
         *  By checking the linear accelertion in the upwards direction, we can determine 
         *  whether the rocket's acceleration has dropped under a determined threshold.
         *  If so, we move to the pre-GNC state.
         */
        bool boostEventCheck();

        /**
         *  @brief State check that checks if the rocket is in the boost state
         * 
         *  By checking the altitude, acceleration average, angle, and velocity of the
         *  rocket, we can determine if the rocket is in the boost state. If it 
         *  is not in the boost state, the rocket moves to the unknown state.
         */
        bool boostStateCheck();


        /**
         *  @brief Event check that checks if the rocket has moved out of the pre-GNC state
         * 
         *  If a certain amount of time has passed, we can safely move out of pre-GNC and to
         *  coast GNC state.
         */
        bool coastPreGNCEventCheck();


        /**
         *  @brief State check that checks if the rocket is in the pre-GNC state
         * 
         *  By checking the altitude, acceleration average, angle, and velocity of the
         *  rocket, we can determine if the rocket is in the pre-GNC state. If it 
         *  is not in the pre-GNC state, the rocket moves to the unknown state.
         */
        bool coastPreGNCStateCheck();


        /**
         *  @brief Event check that checks if the rocket has moved out of the coast-GNC state
         * 
         *  If the velocity has become negative (within some error), we move to the apogee state.
         */
        bool coastGNCEventCheck();

        /**
         *  @brief State check that checks if the rocket is in the coast-GNC state
         * 
         *  By checking the altitude, acceleration average, angle, and velocity of the
         *  rocket, we can determine if the rocket is in the coast-GNC state. If it 
         *  is not in the coast-GNC state, the rocket moves to the unknown state.
         */
        bool coastGNCStateCheck();

        /**
         *  @brief Event check that checks if the rocket has moved out of the apogee state
         * 
         *  If the velocity is greater than a predetermined value, set the rocket state to seperation.
         */
        bool apogeeEventCheck();

        /**
         *  @brief State check that checks if the rocket is in the apogee state
         * 
         *  By checking the altitude, acceleration average, and velocity of the
         *  rocket, we can determine if the rocket is in the apogee state. If it 
         *  is not in the apogee state, the rocket moves to the unknown state.
         */
        bool apogeeStateCheck();

        /**
         *  @brief Event check that checks if the rocket has moved out of the sepeartion state
         * 
         *  If the acceleration is lower than a predetermined value, set the rocket state to drogue.
         */
        bool separationEventCheck();

        /**
         *  @brief State check that checks if the rocket is in the seperation state
         * 
         *  By checking the altitude, acceleration average, and velocity of the
         *  rocket, we can determine if the rocket is in the seperation state. If it 
         *  is not in the seperation state, the rocket moves to the unknown state.
         */
        bool separationStateCheck();

        /**
         *  @brief Event check that checks if the rocket has moved out of the drogue state
         * 
         *  If the acceleration is lower than a predetermined value, set the rocket state to main.
         */
        bool drogueEventCheck();

        /**
         *  @brief State check that checks if the rocket is in the drogue state
         * 
         *  By checking the altitude, acceleration average, angle, and velocity of the
         *  rocket, we can determine if the rocket is in the drogue state. If it 
         *  is not in the drogue state, the rocket moves to the unknown state.
         */
        bool drogueStateCheck();

        /**
         *  @brief Event check that checks if the rocket has moved out of the main state
         * 
         *  If the velocity is zero (plus or minus some error), set the rocket state to landed
         */
        bool mainEventCheck();

        /**
         *  @brief State check that checks if the rocket is in the main state
         * 
         *  By checking the altitude, acceleration average, angle, and velocity of the
         *  rocket, we can determine if the rocket is in the main state. If it 
         *  is not in the main state, the rocket moves to the unknown state.
         */
        bool mainStateCheck();

        /**
         *  @brief State check that checks if the rocket is in the landed state
         * 
         *  By checking the altitude, acceleration average, and velocity of the
         *  rocket, we can determine if the rocket is in the landed state. If it 
         *  is not in the landed state, the rocket moves to the unknown state.
         */
        bool landedStateCheck();

};