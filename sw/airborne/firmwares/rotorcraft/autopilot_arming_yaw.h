/*
 * Copyright (C) 2012 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/autopilot_arming_yaw.h
 *
 * Arm the motors by with max yaw stick.
 *
 */

#ifndef AUTOPILOT_ARMING_YAW_H
#define AUTOPILOT_ARMING_YAW_H

#include "autopilot_rc_helpers.h"

/** Delay until motors are armed/disarmed.
 * In number of rc frames recieved.
 * So 40 is usually ~1s.
 */
#ifndef MOTOR_ARMING_DELAY
#define MOTOR_ARMING_DELAY  40
#endif

/// Motors ON check state machine states
enum arming_state {
  STATUS_INITIALISE_RC,
  STATUS_MOTORS_AUTOMATICALLY_OFF,
  STATUS_MOTORS_AUTOMATICALLY_OFF_SAFETY_WAIT,
  STATUS_MOTORS_OFF,
  STATUS_M_OFF_STICK_PUSHED,
  STATUS_START_MOTORS,
  STATUS_MOTORS_ON,
  STATUS_M_ON_STICK_PUSHED,
  STATUS_STOP_MOTORS
};

uint32_t autopilot_motors_on_counter;
enum arming_state autopilot_check_motor_status;


static inline void autopilot_arming_init(void)
{
//  printf("\nautopilot_arming_init yaw\n");
  autopilot_motors_on_counter = 0;
  autopilot_check_motor_status = STATUS_INITIALISE_RC;
}


/** Update the status of the check_motors state machine.
 */
static inline void autopilot_arming_set(bool_t motors_on)
{
  if (motors_on) {
    autopilot_check_motor_status = STATUS_MOTORS_ON;
  } else {
    autopilot_check_motor_status = STATUS_MOTORS_AUTOMATICALLY_OFF;
  }
}

/**
 * State machine to check if motors should be turned ON or OFF.
 * The motors start/stop when pushing the yaw stick without throttle until #MOTOR_ARMING_DELAY is reached.
 * An intermediate state prevents oscillating between ON and OFF while keeping the stick pushed.
 * The stick must return to a neutral position before starting/stoping again.
 */
static inline void autopilot_arming_check_motors_on(void)
{
  /* only allow switching motor if not in KILL mode */
  if (autopilot_mode != AP_MODE_KILL) {
//    printf("autopilot_mode != AP_MODE_KILL\n");

    switch (autopilot_check_motor_status) {
      case STATUS_INITIALISE_RC: // Wait until RC is initialised (it being centered is a good pointer to this)
//        printf("STATUS_INITIALISE_RC\n");
            if (THROTTLE_STICK_DOWN() && YAW_STICK_CENTERED() && PITCH_STICK_CENTERED() && ROLL_STICK_CENTERED()) {
//              printf("IN IF nothing\n");
          autopilot_check_motor_status = STATUS_MOTORS_OFF;
        }
        break;
      case STATUS_MOTORS_AUTOMATICALLY_OFF: // Motors were disarmed externally
//        printf("STATUS_MOTORS_AUTOMATICALLY_OFF\n");
        //(possibly due to crash)
        //wait extra delay before enabling the normal arming state machine
        autopilot_motors_on = FALSE;
        autopilot_motors_on_counter = 0;
        if (THROTTLE_STICK_DOWN() && YAW_STICK_CENTERED()) { // stick released
          printf("IN IF RELEASED\n");
          autopilot_check_motor_status = STATUS_MOTORS_AUTOMATICALLY_OFF_SAFETY_WAIT;
        }
        break;
      case STATUS_MOTORS_AUTOMATICALLY_OFF_SAFETY_WAIT:
//        printf("STATUS_MOTORS_AUTOMATICALLY_OFF_SAFETY_WAIT\n");
          autopilot_motors_on_counter++;
          if (autopilot_motors_on_counter >= MOTOR_ARMING_DELAY) {
            autopilot_check_motor_status = STATUS_MOTORS_OFF;
          }
        break;
      case STATUS_MOTORS_OFF:
//        printf("STATUS_MOTORS_OFF\n");
        autopilot_motors_on = FALSE;
        autopilot_motors_on_counter = 0;
        if (THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED()) { // stick pushed
          printf("IN IF pushed\n");
          autopilot_check_motor_status = STATUS_M_OFF_STICK_PUSHED;
        }
        break;
      case STATUS_M_OFF_STICK_PUSHED:
//        printf("STATUS_M_OFF_STICK_PUSHED\n");
        autopilot_motors_on = FALSE;
        autopilot_motors_on_counter++;
        if (autopilot_motors_on_counter >= MOTOR_ARMING_DELAY) {
          autopilot_check_motor_status = STATUS_START_MOTORS;
        } else if (!(THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED())) { // stick released too soon
          printf("IN IF toosoon\n");
          autopilot_check_motor_status = STATUS_MOTORS_OFF;
        }
        break;
      case STATUS_START_MOTORS:
//        printf("STATUS_START_MOTORS\n");
        autopilot_motors_on = TRUE;
        autopilot_motors_on_counter = MOTOR_ARMING_DELAY;
        if (!(THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED())) { // wait until stick released
          printf("IN IF wait till released \n");
          autopilot_check_motor_status = STATUS_MOTORS_ON;
        }
        break;
      case STATUS_MOTORS_ON:
//        printf("STATUS_MOTORS_ON\n");
        autopilot_motors_on = TRUE;
        autopilot_motors_on_counter = MOTOR_ARMING_DELAY;
        if (THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED()) { // stick pushed
          printf("IN IF pushed 2\n");
          autopilot_check_motor_status = STATUS_M_ON_STICK_PUSHED;
        }
        break;
      case STATUS_M_ON_STICK_PUSHED:
//        printf("STATUS_M_ON_STICK_PUSHED\n");
        autopilot_motors_on = TRUE;
        autopilot_motors_on_counter--;
        if (autopilot_motors_on_counter == 0) {
          autopilot_check_motor_status = STATUS_STOP_MOTORS;
        } else if (!(THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED())) { // stick released too soon
          printf("IN IF released too soon 2\n");
          autopilot_check_motor_status = STATUS_MOTORS_ON;
        }
        break;
      case STATUS_STOP_MOTORS:
//        printf("STATUS_STOP_MOTORS\n");
        autopilot_motors_on = FALSE;
        autopilot_motors_on_counter = 0;
        if (!(THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED())) { // wait until stick released
          printf("IN IF wait \n");
          autopilot_check_motor_status = STATUS_MOTORS_OFF;
        }
        break;
      default:
        break;
    }
  }
}

#endif /* AUTOPILOT_ARMING_YAW_H */
