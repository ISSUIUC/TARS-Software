#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>

#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include "KX134-1211.h" //High-G IMU Library
#include "ZOEM8Q0.hpp" //GPS Library
#include "hybridShared.h"
#include "acShared.h"
#include "dataLog.h"
#include "dataLog.cpp"
#include "thresholds.h"
#include "pins.h"

fsm_struct rocketTimers;

static THD_FUNCTION(rocket_FSM, arg){
  struct pointers *pointer_struct = (struct pointers *)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### Rocket FSM thread entrance");
    #endif

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      // Lock mutexes for data used in switch
      chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG);
      chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS);
      chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_GPS); //TODO, used for state init?
      switch (pointer_struct->sensorDataPointer->rocketState_data.rocketState) {
            case STATE_INIT:
                if(pointer_struct->GPSPointer->get_position_lock()) { //if GPS lock detected, go to state Idle
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_IDLE;
                }
            break;
 
            case STATE_IDLE:
 
                // If high acceleration is observed in z direction...
                if(pointer_struct->sensorDataPointer->state_data.state_az > launch_az_thresh) {
                    rocketTimers.launch_time = chVTGetSystemTime();
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_LAUNCH_DETECT;
                }
 
            break;
 
            case STATE_LAUNCH_DETECT:
 
                //If the acceleration was too brief, go back to IDLE
                if (pointer_struct->sensorDataPointer->state_data.state_az < launch_az_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_IDLE;
                    break;
                }
 
                // measure the length of the burn time (for hysteresis)
                rocketTimers.burn_timer =
                    chVTGetSystemTime() - rocketTimers.launch_time;
 
                // If the acceleration lasts long enough, boost is detected
                if (rocketTimers.burn_timer > launch_time_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_BOOST;
                    digitalWrite(LED_RED, HIGH);
                }
 
            break;
 
            case STATE_BOOST:
 
            // If low acceleration in the Z direction...
            if (pointer_struct->sensorDataPointer->state_data.state_az < coast_thresh) {
                rocketTimers.burnout_time = chVTGetSystemTime();
                pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_BURNOUT_DETECT;
            }
 
            break;
 
            case STATE_BURNOUT_DETECT:
 
                //If the 0 acceleration was too brief, go back to BOOST
                if (pointer_struct->sensorDataPointer->state_data.state_az > coast_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_BOOST;
                    break;
                }
 
                // measure the length of the coast time (for hysteresis)
                rocketTimers.coast_timer =
                    chVTGetSystemTime() - rocketTimers.burnout_time;
 
                // If the low acceleration lasts long enough, coast is detected
                if (rocketTimers.coast_timer > coast_time_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_COAST;
                }
 
            break;
 
            case STATE_COAST:
                //if velocity <= 0
                if (pointer_struct->sensorDataPointer->state_data.state_vz <= apogee_thresh) {
                    rocketTimers.apogee_time = chVTGetSystemTime();
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_APOGEE_DETECT;
                }
 
            break;
 
            case STATE_APOGEE_DETECT:
                //If the negative velocity was too brief, go back to coast
                if (pointer_struct->sensorDataPointer->state_data.state_vz > apogee_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_COAST;
                    break;
                }
 
                // measure the length of the apogee time
                rocketTimers.apogee_timer =
                    chVTGetSystemTime() - rocketTimers.apogee_time;
 
                // If the < 0 velocity lasts long enough, apogee is detected
                if (rocketTimers.coast_timer > apogee_time_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_APOGEE;
                }
            break;
 
            case STATE_APOGEE: //*assumes velocity is <= 0
                //If rocket has velocity in main, drogue, or landed range
                if (pointer_struct->sensorDataPointer->state_data.state_vz < drogue_thresh_max
                        && pointer_struct->sensorDataPointer->state_data.state_vz > drogue_thresh_min) {
                    rocketTimers.drogue_time = chVTGetSystemTime();
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_DROGUE_DETECT;
                } else if (pointer_struct->sensorDataPointer->state_data.state_vz < main_thresh_max
                        && pointer_struct->sensorDataPointer->state_data.state_vz > main_thresh_min) {
                    rocketTimers.main_time = chVTGetSystemTime();
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_MAIN_DETECT;
                } else if (pointer_struct->sensorDataPointer->state_data.state_vz > landed_thresh) {
                    rocketTimers.landing_time = chVTGetSystemTime();
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_LANDED_DETECT;
                }
            break;
 
            case STATE_DROGUE_DETECT:
                //If the detected velocity was too brief, go back to apogee
                if (pointer_struct->sensorDataPointer->state_data.state_vz > drogue_thresh_max
                        || pointer_struct->sensorDataPointer->state_data.state_vz < drogue_thresh_min) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_APOGEE;
                    break;
                }
 
                // measure the length of drogue time
                rocketTimers.apogee_timer =
                    chVTGetSystemTime() - rocketTimers.drogue_time;
 
                // If the velocity lasts long enough, drogue is detected
                if (rocketTimers.drogue_timer > drogue_time_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_DROGUE;
                }
            break;
 
            case STATE_DROGUE:
                //If the rocket has lower velocity
                if (pointer_struct->sensorDataPointer->state_data.state_vz > main_thresh_min) {
                    rocketTimers.main_time = chVTGetSystemTime();
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_MAIN_DETECT;
                }
            break;
 
            case STATE_MAIN_DETECT:
                //If the detected velocity was too brief, go back to apogee
                if (pointer_struct->sensorDataPointer->state_data.state_vz < main_thresh_min
                        || pointer_struct->sensorDataPointer->state_data.state_vz > main_thresh_max) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_APOGEE;
                    break;
                }
 
                // measure the length of the main time
                rocketTimers.main_timer =
                    chVTGetSystemTime() - rocketTimers.main_time;
 
                // If the low velocity lasts long enough, main is detected
                if (rocketTimers.main_timer > main_time_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_MAIN;
                }
            break;
 
            case STATE_MAIN:
                //If the rocket has 0 velocity
                if (pointer_struct->sensorDataPointer->state_data.state_vz > landed_thresh) {
                    rocketTimers.landing_time = chVTGetSystemTime();
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_LANDED_DETECT;
                }
            break;
 
            case STATE_LANDED_DETECT:
                //If the 0 velocity was too brief, go back to apogee
                if (pointer_struct->sensorDataPointer->state_data.state_vz < landed_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_APOGEE;
                    break;
                }
 
                // measure the length of the landing
                rocketTimers.landing_timer =
                    chVTGetSystemTime() - rocketTimers.landing_time;
 
                // If the 0 velocity lasts long enough, landed is detected
                if (rocketTimers.landing_timer > landing_time_thresh) {
                    pointer_struct->sensorDataPointer->rocketState_data.rocketState = STATE_LANDED;
                }
            break;
 
            case STATE_LANDED:
                // final state
            break;

        }
        // Update timestamp for when rocket state was polled
        pointer_struct->sensorDataPointer->rocketState_data.timeStamp_RS = chVTGetSystemTime();

        // Unlock mutexes used during the switch statement
        chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS);
        chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG);
        chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_GPS); //?
        

        // check that data can be writen to the rocket state buffer
        if (chSemWaitTimeout(&pointer_struct->dataloggerTHDVarsPointer.fifoSpace_RS, TIME_IMMEDIATE) != MSG_OK) {
            pointer_struct->dataloggerTHDVarsPointer.bufferErrors_RS++;
            digitalWrite(LED_BUILTIN, HIGH);
            continue;
        }
        // Write rocket state data to the buffer
        chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS);
        pointer_struct->dataloggerTHDVarsPointer.fifoArray[pointer_struct->dataloggerTHDVarsPointer.fifoHead_GPS].rocketState_data = pointer_struct->sensorDataPointer->rocketState_data;
        pointer_struct->dataloggerTHDVarsPointer.bufferErrors_RS = 0;
        pointer_struct->dataloggerTHDVarsPointer.fifoHead_RS = pointer_struct->dataloggerTHDVarsPointer.fifoHead_RS < (FIFO_SIZE - 1) ? pointer_struct->dataloggerTHDVarsPointer.fifoHead_RS + 1 : 0;
        chSemSignal(&pointer_struct->dataloggerTHDVarsPointer.fifoData_RS);
        //!Unlocking &dataMutex for rocket state
        chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS);

        

        chThdSleepMilliseconds(6); // FSM runs at 100 Hz
  }
}