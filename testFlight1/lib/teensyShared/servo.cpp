
#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PWMServo.h>
//TODO remove
#include "main.cpp"

#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include "KX134-1211.h" //High-G IMU Library
#include "ZOEM8Q0.hpp" //GPS Library
#include "hybridShared.h"
#include "acShared.h"
#include "dataLog.h"
#include "dataLog.cpp"
#include "thresholds.h"
#include "pins.h"

PWMServo servo_cw; //Servo that controlls roll in the clockwise direction
PWMServo servo_ccw; //Servo that controlls roll in the counter clockwise direction

float flap_drag;
float native_drag;

void round_off_angle(int &value) {
  if (value > 180) {
    value = 180;
  }
  if (value < 0) {
    value = 0;
  }
}

static THD_FUNCTION(servo_THD, arg){
  (void)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### Servo thread entrance");
    #endif
    
    int ccw_angle = 90;
    int cw_angle = 90;
    bool active_control = false;

    switch(rocketState) {
      case STATE_INIT :
        active_control = true;
        break;
      case STATE_IDLE:
        active_control = true;
        break;
      case STATE_LAUNCH_DETECT :
        active_control = true;
        break;
      case STATE_BOOST :
        active_control = false;
        break;
      case STATE_COAST :
        active_control = true;
        break;
      case STATE_APOGEE_DETECT :
        active_control = false;
        break;
      default :
        active_control = false;
      break;
    }
    // turns active control off if not in takeoff/coast sequence
    if (active_control) {
      cw_angle = lowgSensorData.gz;
      ccw_angle = lowgSensorData.gz;

    } else {
      //Turns active control off if not in coast state.
      cw_angle = 0;
      ccw_angle = 0;
    }
    round_off_angle(cw_angle);
    round_off_angle(ccw_angle);
    servo_cw.write(cw_angle);
    servo_ccw.write(ccw_angle); 
    
    #ifdef SERVO_DEBUG
      Serial.print("\nclockwise: ");
      Serial.print(cw_angle);
      Serial.print(" counterclockwise: ");
      Serial.print(ccw_angle);
    #endif

    chMtxUnlock(&lowg_datalogger_THD_vars.dataMutex);
    chThdSleepMilliseconds(6); // FSM runs at 100 Hz
  }

}