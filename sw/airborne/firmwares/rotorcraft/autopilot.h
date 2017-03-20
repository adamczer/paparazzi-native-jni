/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * @file firmwares/rotorcraft/autopilot.h
 *
 * Autopilot modes.
 *
 */

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "std.h"
#include "generated/airframe.h"
#include "state.h"

#define AP_MODE_KILL              0
#define AP_MODE_FAILSAFE          1
#define AP_MODE_HOME              2
#define AP_MODE_RATE_DIRECT       3
#define AP_MODE_ATTITUDE_DIRECT   4
#define AP_MODE_RATE_RC_CLIMB     5
#define AP_MODE_ATTITUDE_RC_CLIMB 6
#define AP_MODE_ATTITUDE_CLIMB    7
#define AP_MODE_RATE_Z_HOLD       8
#define AP_MODE_ATTITUDE_Z_HOLD   9
#define AP_MODE_HOVER_DIRECT      10
#define AP_MODE_HOVER_CLIMB       11
#define AP_MODE_HOVER_Z_HOLD      12
#define AP_MODE_NAV               13
#define AP_MODE_RC_DIRECT         14  // Safety Pilot Direct Commands for helicopter direct control
#define AP_MODE_CARE_FREE_DIRECT  15
#define AP_MODE_FORWARD           16
#define AP_MODE_MODULE            17
#define AP_MODE_FLIP              18
#define AP_MODE_GUIDED            19

extern uint8_t autopilot_mode;
extern uint8_t autopilot_mode_auto2;
extern bool_t autopilot_motors_on;
extern bool_t autopilot_in_flight;
extern bool_t kill_throttle;
extern bool_t autopilot_rc;

extern bool_t autopilot_power_switch;

extern void autopilot_init(void);
extern void autopilot_periodic(void);
extern void autopilot_on_rc_frame(void);
extern void autopilot_on_rc_frame_juav(void);
extern void autopilot_set_mode(uint8_t new_autopilot_mode);
extern void autopilot_set_motors_on(bool_t motors_on);
extern void autopilot_check_in_flight(bool_t motors_on);

extern bool_t autopilot_ground_detected;
extern bool_t autopilot_detect_ground_once;

extern uint16_t autopilot_flight_time;

/** Default RC mode.
 */
#ifndef MODE_MANUAL
#define MODE_MANUAL AP_MODE_ATTITUDE_DIRECT
#endif
#ifndef MODE_AUTO1
#define MODE_AUTO1 AP_MODE_HOVER_Z_HOLD
#endif
#ifndef MODE_AUTO2
#define MODE_AUTO2 AP_MODE_NAV
#endif


#define autopilot_KillThrottle(_kill) { \
    if (_kill)                          \
      autopilot_set_motors_on(FALSE);   \
    else                                \
      autopilot_set_motors_on(TRUE);    \
  }

#ifdef POWER_SWITCH_GPIO
#include "mcu_periph/gpio.h"
#define autopilot_SetPowerSwitch(_v) {                          \
    autopilot_power_switch = _v;                                \
    if (_v) { gpio_set(POWER_SWITCH_GPIO); }  \
    else { gpio_clear(POWER_SWITCH_GPIO); }   \
  }
#else
#define autopilot_SetPowerSwitch(_v) {  \
    autopilot_power_switch = _v;        \
  }
#endif

/** Set Rotorcraft commands.
 *  Limit thrust and/or yaw depending of the in_flight
 *  and motors_on flag status
 */
#ifdef ROTORCRAFT_IS_HELI
#define SetRotorcraftCommands(_cmd, _in_flight,  _motor_on) { }
#else

#ifndef ROTORCRAFT_COMMANDS_YAW_ALWAYS_ENABLED
#define SetRotorcraftCommands(_cmd, _in_flight,  _motor_on) { \
    if (!(_in_flight)) { _cmd[COMMAND_YAW] = 0; }               \
    if (!(_motor_on)) { _cmd[COMMAND_THRUST] = 0; }             \
    commands[COMMAND_ROLL] = _cmd[COMMAND_ROLL];                \
    commands[COMMAND_PITCH] = _cmd[COMMAND_PITCH];              \
    commands[COMMAND_YAW] = _cmd[COMMAND_YAW];                  \
    commands[COMMAND_THRUST] = _cmd[COMMAND_THRUST];            \
}
//printf("_in_flight = %d\n", _in_flight);\
//printf("_motor_on = %d\n", _motor_on);\
//printf("COMMAND_ROLL = %d\n", commands[COMMAND_ROLL]);\
//printf("COMMAND_PITCH = %d\n", commands[COMMAND_PITCH]);\
//printf("COMMAND_YAW = %d\n", commands[COMMAND_YAW]);\
//printf("COMMAND_THRUST = %d\n\n\n", commands[COMMAND_THRUST]);\
//  }
#else
#define SetRotorcraftCommands(_cmd, _in_flight,  _motor_on) { }
#endif
#endif

/** Z-acceleration threshold to detect ground in m/s^2 */
#ifndef THRESHOLD_GROUND_DETECT
#define THRESHOLD_GROUND_DETECT 25.0
#endif
/** Ground detection based on vertical acceleration.
 */
static inline void DetectGroundEvent(void)
{
  if (autopilot_mode == AP_MODE_FAILSAFE || autopilot_detect_ground_once) {
    struct NedCoor_f *accel = stateGetAccelNed_f();
    if (accel->z < -THRESHOLD_GROUND_DETECT ||
        accel->z > THRESHOLD_GROUND_DETECT) {
      autopilot_ground_detected = TRUE;
      autopilot_detect_ground_once = FALSE;
    }
  }
}

#include "subsystems/settings.h"

/* try to make sure that we don't write to flash while flying */
static inline void autopilot_StoreSettings(float store)
{
  if (kill_throttle && store) {
    settings_store_flag = store;
    settings_store();
  }
}

static inline void autopilot_ClearSettings(float clear)
{
  if (kill_throttle && clear) {
    settings_clear_flag = clear;
    settings_clear();
  }
}

#if DOWNLINK
#include "subsystems/datalink/transport.h"
extern void send_autopilot_version(struct transport_tx *trans, struct link_device *dev);
#endif

/** Set position and heading setpoints in GUIDED mode.
 * @param x North position (local NED frame) in meters.
 * @param y East position (local NED frame) in meters.
 * @param z Down position (local NED frame) in meters.
 * @param heading Setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern bool_t autopilot_guided_goto_ned(float x, float y, float z, float heading);

/** Set position and heading setpoints wrt. current position in GUIDED mode.
 * @param dx Offset relative to current north position (local NED frame) in meters.
 * @param dy Offset relative to current east position (local NED frame) in meters.
 * @param dz Offset relative to current down position (local NED frame) in meters.
 * @param dyaw Offset relative to current heading setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern bool_t autopilot_guided_goto_ned_relative(float dx, float dy, float dz, float dyaw);

/** Set position and heading setpoints wrt. current position AND heading in GUIDED mode.
 * @param dx relative position (body frame, forward) in meters.
 * @param dy relative position (body frame, right) in meters.
 * @param dz relative position (body frame, down) in meters.
 * @param dyaw Offset relative to current heading setpoint in radians.
 * @return TRUE if setpoint was set (currently in AP_MODE_GUIDED)
 */
extern bool_t autopilot_guided_goto_body_relative(float dx, float dy, float dz, float dyaw);

void autopilot_periodic_prior_juav();
bool is_autopilot_mode_ap_mode_kill_juav();//false stablilization needed -> run java code replacing guidance_h_run(autopilot_in_flight);
void autopilot_periodic_post_juav();
bool get_autopilot_in_flight_juav(); // needed for guidance code

// test all native
void guidance_h_run_native_test_juav(bool in_flight);

void juav_register_periodic_telemetry_send_autopilot_version(void);
void juav_register_periodic_telemetry_send_alive(void);
void juav_register_periodic_telemetry_send_status(void);
void juav_register_periodic_telemetry_send_attitude(void);
void juav_register_periodic_telemetry_send_energy(void);
void juav_register_periodic_telemetry_send_fp(void);
void juav_register_periodic_telemetry_send_rotorcraft_cmd(void);
void juav_register_periodic_telemetry_send_dl_value(void);
void juav_register_periodic_telemetry_send_actuators(void);
void juav_register_periodic_telemetry_send_rc(void);
void juav_register_periodic_telemetry_send_rotorcraft_rc(void);

int juav_get_radio_control_value(int index);
short juav_get_radio_control_status();

struct EnuCoor_i juav_get_navigation_carrot();
float juav_get_dist2_to_home();
bool juav_get_too_far_from_home();
short juav_get_horizontal_mode();
int juav_get_nav_roll();
int juav_get_nav_pitch();
int juav_get_nav_heading();
void juav_set_nav_heading(int new_heading);
int juav_get_vertical_mode();
int juav_get_nav_climb();
int juav_get_nav_flight_altitude();
int juav_get_nav_throttle();

void juav_set_guidance_h_mode(short newMode);
void juav_set_guidance_v_mode(short newMode);
void juav_set_autopilot_mode(short newMode);
void juav_set_stabilization_command(int cmdIndex, int command);
bool juav_get_autopilot_motors_on();
void juav_set_autopilot_motors_on(bool newValue);

void juav_set_autopilot_check_motor_status(int newValue);
int juav_get_autopilot_check_motor_status();
void juav_set_autopilot_motors_on_counter(int newValue);
int juav_get_autopilot_motors_on_counter();


int juav_get_stabilization_cmd(int cmdIndex);
void juav_set_stabilization_cmd(int cmdIndex, int newValue);

int juav_get_guidance_h_sp_pos_x();
int juav_get_guidance_h_sp_pos_y();
void juav_set_guidance_h_sp_pos_x(int x);
void juav_set_guidance_h_sp_pos_y(int y);

int juav_get_guidance_h_sp_speed_x();
int juav_get_guidance_h_sp_speed_y();

int juav_get_guidance_h_ref_pos_x();
int juav_get_guidance_h_ref_pos_y();
void juav_set_guidance_h_ref_pos_x(int x);
void juav_set_guidance_h_ref_pos_y(int y);

int juav_get_guidance_h_ref_speed_x();
int juav_get_guidance_h_ref_speed_y();
void juav_set_guidance_h_ref_speed_x(int x);
void juav_set_guidance_h_ref_speed_y(int y);

int juav_get_guidance_h_ref_accel_x();
int juav_get_guidance_h_ref_accel_y();
void juav_set_guidance_h_ref_accel_x(int x);
void juav_set_guidance_h_ref_accel_y(int y);

int juav_get_guidance_h_sp_heading();
void juav_set_guidance_h_sp_heading(int newHeading);

int juav_get_guidance_h_cmd_earth_x();
int juav_get_guidance_h_cmd_earth_y();
void juav_set_guidance_h_cmd_earth_x(int x);
void juav_set_guidance_h_cmd_earth_y(int y);


int juav_get_stab_att_sp_quat_qi();
int juav_get_stab_att_sp_quat_qx();
int juav_get_stab_att_sp_quat_qy();
int juav_get_stab_att_sp_quat_qz();
void juav_set_stab_att_sp_quat_qi(int qi);
void juav_set_stab_att_sp_quat_qx(int qx);
void juav_set_stab_att_sp_quat_qy(int qy);
void juav_set_stab_att_sp_quat_qz(int qz);

int juav_get_stab_att_sp_euler_phi();
int juav_get_stab_att_sp_euler_psi();
int juav_get_stab_att_sp_euler_theta();
void juav_set_stab_att_sp_euler_phi(int phi);
void juav_set_stab_att_sp_euler_psi(int psi);
void juav_set_stab_att_sp_euler_theta(int theta);

void guidance_v_run_native_test_juav(bool in_flight);
void juav_guidance_h_mode_changed(short newMode);
void juav_guidance_v_mode_changed(short newMode);
void juav_guidance_h_read_rc(bool in_flight);

void juav_autopilot_set_mode_native(short new_mode);

//For Fiji

int  juav_fiji_stateGetPositionNedIX();
int  juav_fiji_stateGetPositionNedIY();
int  juav_fiji_stateGetPositionNedIZ();
int  juav_fiji_stateGetSpeedNedIX();
int  juav_fiji_stateGetSpeedNedIY();
int  juav_fiji_stateGetSpeedNedIZ();
int  juav_fiji_stateGetNedToBodyRMatI_10();
int  juav_fiji_stateGetNedToBodyRMatI_11();
int  juav_fiji_stateGetNedToBodyRMatI_12();
int  juav_fiji_stateGetNedToBodyRMatI_13();
int  juav_fiji_stateGetNedToBodyRMatI_14();
int  juav_fiji_stateGetNedToBodyRMatI_15();
int  juav_fiji_stateGetNedToBodyRMatI_16();
int  juav_fiji_stateGetNedToBodyRMatI_17();
int  juav_fiji_stateGetNedToBodyRMatI_18();
int  juav_fiji_stateGetNedToBodyEulersIPsiInt();
int  juav_fiji_stateGetNedToBodyEulersITheataInt();
int  juav_fiji_stateGetNedToBodyEulersIPhiInt();
float  juav_fiji_stateGetNedToBodyEulersIPsiFloat();
float  juav_fiji_stateGetNedToBodyEulersITheataFloat();
float  juav_fiji_stateGetNedToBodyEulersIPhiFloat();
int  juav_fiji_stateGetAccelNedIX();
int  juav_fiji_stateGetAccelNedIY();
int  juav_fiji_stateGetAccelNedIZ();
float juav_fiji_stateGetSpeedNedFX();
float juav_fiji_stateGetSpeedNedFY();
float juav_fiji_stateGetSpeedNedFZ();
float juav_fiji_stateGetAccelNedFX();
float juav_fiji_stateGetAccelNedFY();
float juav_fiji_stateGetAccelNedFZ();
int juav_fiji_getNavigationCarrotX();
int juav_fiji_getNavigationCarrotY();
int juav_fiji_getNavigationCarrotZ();
void juav_fiji_setAutopilotGroundDetected(bool newValue);
void juav_fiji_setAutopilotDetectGroundOnce(bool newValue);

long getNextAutopilotIterationIndex();
#endif /* AUTOPILOT_H */

