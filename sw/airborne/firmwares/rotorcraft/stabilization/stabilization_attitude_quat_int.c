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

/** @file stabilization_attitude_quat_int.c
 * Rotorcraft quaternion attitude stabilization
 */

#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "std.h"
#include "paparazzi.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"
#include <time.h>

struct Int32AttitudeGains stabilization_gains = {
  {STABILIZATION_ATTITUDE_PHI_PGAIN, STABILIZATION_ATTITUDE_THETA_PGAIN, STABILIZATION_ATTITUDE_PSI_PGAIN },
  {STABILIZATION_ATTITUDE_PHI_DGAIN, STABILIZATION_ATTITUDE_THETA_DGAIN, STABILIZATION_ATTITUDE_PSI_DGAIN },
  {STABILIZATION_ATTITUDE_PHI_DDGAIN, STABILIZATION_ATTITUDE_THETA_DDGAIN, STABILIZATION_ATTITUDE_PSI_DDGAIN },
  {STABILIZATION_ATTITUDE_PHI_IGAIN, STABILIZATION_ATTITUDE_THETA_IGAIN, STABILIZATION_ATTITUDE_PSI_IGAIN }
};

/* warn if some gains are still negative */
#if (STABILIZATION_ATTITUDE_PHI_PGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_THETA_PGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_PSI_PGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_PHI_DGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_THETA_DGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_PSI_DGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_PHI_IGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_THETA_IGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_PSI_IGAIN  < 0)
#error "ALL control gains have to be positive!!!"
#endif

struct Int32Quat stabilization_att_sum_err_quat;

int32_t stabilization_att_fb_cmd[COMMANDS_NB];
int32_t stabilization_att_ff_cmd[COMMANDS_NB];

struct Int32Quat   stab_att_sp_quat;
struct Int32Eulers stab_att_sp_euler;

struct AttRefQuatInt att_ref_quat_i;

#define IERROR_SCALE 128
#define GAIN_PRESCALER_FF 48
#define GAIN_PRESCALER_P 12
#define GAIN_PRESCALER_D 3
#define GAIN_PRESCALER_I 3
static bool juavBenchmarkLogging = false;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_att(struct transport_tx *trans, struct link_device *dev)   //FIXME really use this message here ?
{
  struct Int32Rates *body_rate = stateGetBodyRates_i();
  struct Int32Eulers *att = stateGetNedToBodyEulers_i();
  pprz_msg_send_STAB_ATTITUDE_INT(trans, dev, AC_ID,
                                  &(body_rate->p), &(body_rate->q), &(body_rate->r),
                                  &(att->phi), &(att->theta), &(att->psi),
                                  &stab_att_sp_euler.phi,
                                  &stab_att_sp_euler.theta,
                                  &stab_att_sp_euler.psi,
                                  &stabilization_att_sum_err_quat.qx,
                                  &stabilization_att_sum_err_quat.qy,
                                  &stabilization_att_sum_err_quat.qz,
                                  &stabilization_att_fb_cmd[COMMAND_ROLL],
                                  &stabilization_att_fb_cmd[COMMAND_PITCH],
                                  &stabilization_att_fb_cmd[COMMAND_YAW],
                                  &stabilization_att_ff_cmd[COMMAND_ROLL],
                                  &stabilization_att_ff_cmd[COMMAND_PITCH],
                                  &stabilization_att_ff_cmd[COMMAND_YAW],
                                  &stabilization_cmd[COMMAND_ROLL],
                                  &stabilization_cmd[COMMAND_PITCH],
                                  &stabilization_cmd[COMMAND_YAW]);
}

static void send_att_ref(struct transport_tx *trans, struct link_device *dev)
{
  // ref eulers in message are with REF_ANGLE_FRAC, convert
  struct Int32Eulers ref_euler;
  INT32_EULERS_LSHIFT(ref_euler, att_ref_quat_i.euler, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
  pprz_msg_send_STAB_ATTITUDE_REF_INT(trans, dev, AC_ID,
                                      &stab_att_sp_euler.phi,
                                      &stab_att_sp_euler.theta,
                                      &stab_att_sp_euler.psi,
                                      &ref_euler.phi,
                                      &ref_euler.theta,
                                      &ref_euler.psi,
                                      &att_ref_quat_i.rate.p,
                                      &att_ref_quat_i.rate.q,
                                      &att_ref_quat_i.rate.r,
                                      &att_ref_quat_i.accel.p,
                                      &att_ref_quat_i.accel.q,
                                      &att_ref_quat_i.accel.r);
}

static void send_ahrs_ref_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  pprz_msg_send_AHRS_REF_QUAT(trans, dev, AC_ID,
                              &att_ref_quat_i.quat.qi,
                              &att_ref_quat_i.quat.qx,
                              &att_ref_quat_i.quat.qy,
                              &att_ref_quat_i.quat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz));
}
#endif
//TODO ALL THIS
void stabilization_attitude_init(void)
{

  attitude_ref_quat_int_init(&att_ref_quat_i);

  int32_quat_identity(&stabilization_att_sum_err_quat);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_INT, send_att);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_REF_INT, send_att_ref);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_REF_QUAT, send_ahrs_ref_quat);
#endif
}

void juav_register_periodic_telemetry_send_att() {
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_INT, send_att);
}
void juav_register_periodic_telemetry_send_att_ref() {
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_REF_INT, send_att_ref);
}
void juav_register_periodic_telemetry_send_ahrs_ref_quat() {
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_REF_QUAT, send_ahrs_ref_quat);
}


void stabilization_attitude_enter(void) //TODO PORT
{
  printf("stabilization_attitude_enter\n");

  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  attitude_ref_quat_int_enter(&att_ref_quat_i, stab_att_sp_euler.psi);

  int32_quat_identity(&stabilization_att_sum_err_quat);

//  printf("stab_att_sp_euler psi,psi,theta= %d,%d,%d",
//                     stab_att_sp_euler.psi,
//                     stab_att_sp_euler.phi,
//                     stab_att_sp_euler.theta);

}

void stabilization_attitude_set_failsafe_setpoint(void)
{
//  printf("stabilization_attitude_set_failsafe_setpoint yyyyyyyy\n");
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy)//TODO PORT
{
//  printf("stabilization_attitude_set_rpy_setpoint_i\n");
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler = *rpy;

  int32_quat_of_eulers(&stab_att_sp_quat, &stab_att_sp_euler);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
//  printf("stabilization_attitude_set_earth_cmd_i\n");
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler.psi = heading;

  // compute sp_euler phi/theta for debugging/telemetry
  /* Rotate horizontal commands to body frame by psi */
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);
  stab_att_sp_euler.phi = (-s_psi * cmd->x + c_psi * cmd->y) >> INT32_TRIG_FRAC;
  stab_att_sp_euler.theta = -(c_psi * cmd->x + s_psi * cmd->y) >> INT32_TRIG_FRAC;
//  printf("CCC stab_att_sp_euler psi,phi,theta = %d,%d,%d\n",stab_att_sp_euler.psi,stab_att_sp_euler.phi,stab_att_sp_euler.theta);

  quat_from_earth_cmd_i(&stab_att_sp_quat, cmd, heading);
//  printf("CCC stab_att_sp_quat qi,qx,qy,qz = %d,%d,%d,%d\n",stab_att_sp_quat.qi,stab_att_sp_quat.qx,stab_att_sp_quat.qy,stab_att_sp_quat.qz);
//  printf("CCC stab_att_sp_quat qi,qx,qy,qz = %d,%d,%d,%d\n",quat.qi,quat.qx,quat.qy,quat.qz);
}

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

static void attitude_run_ff(int32_t ff_commands[], struct Int32AttitudeGains *gains, struct Int32Rates *ref_accel)
{
//  printf("gains->dd.x = %d\n",gains->dd.x);
//  printf("gains->dd.y = %d\n",gains->dd.y);
//  printf("gains->dd.z = %d\n",gains->dd.z);
//    printf("ref_accel->p = %d\n",ref_accel->p);
//  printf("ref_accel->q = %d\n",ref_accel->q);
//  printf("ref_accel->r = %d\n",ref_accel->r);
  /* Compute feedforward based on reference acceleration */

  ff_commands[COMMAND_ROLL]  = GAIN_PRESCALER_FF * gains->dd.x * RATE_FLOAT_OF_BFP(ref_accel->p) / (1 << 7);
  ff_commands[COMMAND_PITCH] = GAIN_PRESCALER_FF * gains->dd.y * RATE_FLOAT_OF_BFP(ref_accel->q) / (1 << 7);
  ff_commands[COMMAND_YAW]   = GAIN_PRESCALER_FF * gains->dd.z * RATE_FLOAT_OF_BFP(ref_accel->r) / (1 << 7);
}

static void attitude_run_fb(int32_t fb_commands[], struct Int32AttitudeGains *gains, struct Int32Quat *att_err,
                            struct Int32Rates *rate_err, struct Int32Quat *sum_err)
{
//  printf("attitude_run_fb\n");
  /*  PID feedback */
  fb_commands[COMMAND_ROLL] =
    GAIN_PRESCALER_P * gains->p.x  * QUAT1_FLOAT_OF_BFP(att_err->qx) +
    GAIN_PRESCALER_D * gains->d.x  * RATE_FLOAT_OF_BFP(rate_err->p) +
    GAIN_PRESCALER_I * gains->i.x  * QUAT1_FLOAT_OF_BFP(sum_err->qx);

  fb_commands[COMMAND_PITCH] =
    GAIN_PRESCALER_P * gains->p.y  * QUAT1_FLOAT_OF_BFP(att_err->qy) +
    GAIN_PRESCALER_D * gains->d.y  * RATE_FLOAT_OF_BFP(rate_err->q) +
    GAIN_PRESCALER_I * gains->i.y  * QUAT1_FLOAT_OF_BFP(sum_err->qy);

  fb_commands[COMMAND_YAW] =
    GAIN_PRESCALER_P * gains->p.z  * QUAT1_FLOAT_OF_BFP(att_err->qz) +
    GAIN_PRESCALER_D * gains->d.z  * RATE_FLOAT_OF_BFP(rate_err->r) +
    GAIN_PRESCALER_I * gains->i.z  * QUAT1_FLOAT_OF_BFP(sum_err->qz);

//  printf("fb_commands = yaw,pitch,roll = %d,%d,%d\n",
//         fb_commands[COMMAND_YAW],
//         fb_commands[COMMAND_PITCH],
//         fb_commands[COMMAND_ROLL]);

}
static int iterCount = 0;
void stabilization_attitude_run(bool_t enable_integrator)
{
//  printf("stabilization_attitude_run\n");

  /*
   * Update reference
   * Warning: dt is currently not used in the quat_int ref impl
   * PERIODIC_FREQUENCY is assumed to be 512Hz
   */
  static const float dt = (1./PERIODIC_FREQUENCY);
//  printf("att_ref_quat_i->saturation->max_accel.p,q,r = %d,%d,%d\n",att_ref_quat_i.saturation.max_accel.p,att_ref_quat_i.saturation.max_accel.q,att_ref_quat_i.saturation.max_accel.r);
//  printf("stab_att_sp_quat qi,qx,qy,qz = %d,%d,%d,%d\n",stab_att_sp_quat.qi,stab_att_sp_quat.qx,stab_att_sp_quat.qy,stab_att_sp_quat.qz);
  attitude_ref_quat_int_update(&att_ref_quat_i, &stab_att_sp_quat, dt);
  struct timespec t0;
  if(juavBenchmarkLogging)
    clock_gettime(CLOCK_REALTIME, &t0);

  /*
   * Compute errors for feedback
   */

  /* attitude error                          */
  struct Int32Quat att_err;
  struct Int32Quat *att_quat = stateGetNedToBodyQuat_i();
//  printf("att_quat qi,qx,qy,qz = %d,%d,%d,%d\n",att_quat->qi,att_quat->qx,att_quat->qy,att_quat->qz);
  INT32_QUAT_INV_COMP(att_err, *att_quat, att_ref_quat_i.quat);
  /* wrap it in the shortest direction       */
  int32_quat_wrap_shortest(&att_err);
  int32_quat_normalize(&att_err);

  /*  rate error                */
  const struct Int32Rates rate_ref_scaled = {
    OFFSET_AND_ROUND(att_ref_quat_i.rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(att_ref_quat_i.rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(att_ref_quat_i.rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC))
  };
  struct Int32Rates rate_err;
  struct Int32Rates *body_rate = stateGetBodyRates_i();
  RATES_DIFF(rate_err, rate_ref_scaled, (*body_rate));

#define INTEGRATOR_BOUND 100000
  /* integrated error */
  if (enable_integrator) {
    stabilization_att_sum_err_quat.qx += att_err.qx / IERROR_SCALE;
    stabilization_att_sum_err_quat.qy += att_err.qy / IERROR_SCALE;
    stabilization_att_sum_err_quat.qz += att_err.qz / IERROR_SCALE;
    Bound(stabilization_att_sum_err_quat.qx, -INTEGRATOR_BOUND, INTEGRATOR_BOUND);
    Bound(stabilization_att_sum_err_quat.qy, -INTEGRATOR_BOUND, INTEGRATOR_BOUND);
    Bound(stabilization_att_sum_err_quat.qz, -INTEGRATOR_BOUND, INTEGRATOR_BOUND);
  } else {
    /* reset accumulator */
    int32_quat_identity(&stabilization_att_sum_err_quat);
  }

  /* compute the feed forward command */
  attitude_run_ff(stabilization_att_ff_cmd, &stabilization_gains, &att_ref_quat_i.accel);
//  printf("stabilization_att_ff_cmd roll,pitch,yaw = %d,%d,%d\n",stabilization_att_ff_cmd[COMMAND_ROLL],stabilization_att_ff_cmd[COMMAND_PITCH],stabilization_att_ff_cmd[COMMAND_YAW]);
//  printf("att_ref_quat_i.accel p,q,r = %d,%d,%d\n",att_ref_quat_i.accel.p,att_ref_quat_i.accel.q,att_ref_quat_i.accel.r);

  /* compute the feed back command */
  attitude_run_fb(stabilization_att_fb_cmd, &stabilization_gains, &att_err, &rate_err, &stabilization_att_sum_err_quat);

  /* sum feedforward and feedback */
  stabilization_cmd[COMMAND_ROLL] = stabilization_att_fb_cmd[COMMAND_ROLL] + stabilization_att_ff_cmd[COMMAND_ROLL];
  stabilization_cmd[COMMAND_PITCH] = stabilization_att_fb_cmd[COMMAND_PITCH] + stabilization_att_ff_cmd[COMMAND_PITCH];
  stabilization_cmd[COMMAND_YAW] = stabilization_att_fb_cmd[COMMAND_YAW] + stabilization_att_ff_cmd[COMMAND_YAW];

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);

//  printf("stabilization_cmd yaw,pitch,roll = %d,%d,%d\n",stabilization_cmd[COMMAND_YAW],stabilization_cmd[COMMAND_PITCH],stabilization_cmd[COMMAND_ROLL]);


  if(juavBenchmarkLogging) {
    iterCount++;
    struct timespec t1;
    clock_gettime(CLOCK_REALTIME, &t1); // Works on Linux
    long elapsed = (t1.tv_sec-t0.tv_sec)*1000000 + t1.tv_nsec-t0.tv_nsec;
    printf("%d", iterCount);
    printf(" %d\n", elapsed);
  }
}

//TODO PORT THIS
void stabilization_attitude_read_rc(bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn)
{
  printf("stabilization_attitude_read_rc\n");
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
    printf("USE_EARTH_BOUND_RC_SETPOINT\n");
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
//    printf("USE_EARTH_BOUND_RC_SETPOINT ELSE\n");
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif
//  printf("stab_att_sp_quat qi,qx,qy,qz = %d,%d,%d,%d\n",stab_att_sp_quat.qi,stab_att_sp_quat.qx,stab_att_sp_quat.qy,stab_att_sp_quat.qz);
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}

void attitude_ref_quat_int_update_juav(float dt) {
  attitude_ref_quat_int_update(&att_ref_quat_i, &stab_att_sp_quat, dt);
}

int get_stabilization_att_sum_err_quat_i_juav() {
  return stabilization_att_sum_err_quat.qi;
}
int get_stabilization_att_sum_err_quat_x_juav() {
  return stabilization_att_sum_err_quat.qx;
}
int get_stabilization_att_sum_err_quat_y_juav() {
  return stabilization_att_sum_err_quat.qy;
}
int get_stabilization_att_sum_err_quat_z_juav() {
  return stabilization_att_sum_err_quat.qz;
}
void set_stabilization_att_sum_err_quat_i_juav(int qi) {
  stabilization_att_sum_err_quat.qi = qi;
}
void set_stabilization_att_sum_err_quat_x_juav(int qx) {
  stabilization_att_sum_err_quat.qx = qx;
}
void set_stabilization_att_sum_err_quat_y_juav(int qy) {
  stabilization_att_sum_err_quat.qy = qy;
}
void set_stabilization_att_sum_err_quat_z_juav(int qz) {
  stabilization_att_sum_err_quat.qz = qz;
}

int get_att_ref_quat_i_quat_qi_juav() {
  return att_ref_quat_i.quat.qi;
}
int get_att_ref_quat_i_quat_qx_juav() {
  return att_ref_quat_i.quat.qx;
}
int get_att_ref_quat_i_quat_qy_juav() {
  return att_ref_quat_i.quat.qy;
}
int get_att_ref_quat_i_quat_qz_juav() {
  return att_ref_quat_i.quat.qz;
}

int get_att_ref_quat_i_rate_p_juav() {
  return att_ref_quat_i.rate.p;
}
int get_att_ref_quat_i_rate_q_juav() {
  return att_ref_quat_i.rate.q;
}
int get_att_ref_quat_i_rate_r_juav() {
  return att_ref_quat_i.rate.r;
}
int get_att_ref_quat_i_accel_p_juav() {
  return att_ref_quat_i.accel.p;
}
int get_att_ref_quat_i_accel_q_juav() {
  return att_ref_quat_i.accel.q;
}
int get_att_ref_quat_i_accel_r_juav() {
  return att_ref_quat_i.accel.r;
}

void set_att_ref_quat_i_quat_qi_juav(int i) {
  att_ref_quat_i.quat.qi = i;
}
  void set_att_ref_quat_i_quat_qx_juav(int i) {
  att_ref_quat_i.quat.qx = i;
}
void set_att_ref_quat_i_quat_qy_juav(int i) {
  att_ref_quat_i.quat.qy = i;
}
void set_att_ref_quat_i_quat_qz_juav(int i) {
  att_ref_quat_i.quat.qz = i;
}

void set_att_ref_quat_i_rate_p_juav(int i) {
  att_ref_quat_i.rate.p = i;
}
void set_att_ref_quat_i_rate_q_juav(int i) {
  att_ref_quat_i.rate.q = i;
}
void set_att_ref_quat_i_rate_r_juav(int i) {
  att_ref_quat_i.rate.r = i;
}
void set_att_ref_quat_i_accel_p_juav(int i) {
  att_ref_quat_i.accel.p = i;
}
void set_att_ref_quat_i_accel_q_juav(int i) {
  att_ref_quat_i.accel.q = i;
}
void set_att_ref_quat_i_accel_r_juav(int i) {
  att_ref_quat_i.accel.r = i;
}

int get_stabilization_gains_p_x_juav() {
  return stabilization_gains.p.x;
}
int get_stabilization_gains_p_y_juav() {
  return stabilization_gains.p.y;
}
int get_stabilization_gains_p_z_juav() {
  return stabilization_gains.p.z;
}
int get_stabilization_gains_d_x_juav() {
  return stabilization_gains.d.x;
}
int get_stabilization_gains_d_y_juav() {
  return stabilization_gains.d.y;
}
int get_stabilization_gains_d_z_juav() {
  return stabilization_gains.d.z;
}
int get_stabilization_gains_dd_x_juav() {
  return stabilization_gains.dd.x;
}
int get_stabilization_gains_dd_y_juav() {
  return stabilization_gains.dd.y;
}
int get_stabilization_gains_dd_z_juav() {
  return stabilization_gains.dd.z;
}
int get_stabilization_gains_i_x_juav() {
  return stabilization_gains.i.x;
}
int get_stabilization_gains_i_y_juav() {
  return stabilization_gains.i.y;
}
int get_stabilization_gains_i_z_juav() {
  return stabilization_gains.i.z;
}

void set_stabilization_cmd(int yaw,int pitch, int roll) {
  stabilization_cmd[COMMAND_ROLL] = roll;
  stabilization_cmd[COMMAND_PITCH] = pitch;
  stabilization_cmd[COMMAND_YAW] = yaw;
}

int get_stateGetNedToBodyQuat_i_Qi_juav() {
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  return quat->qi;
}
int get_stateGetNedToBodyQuat_i_Qx_juav() {
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  return quat->qx;
}
int get_stateGetNedToBodyQuat_i_Qy_juav() {
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  return quat->qy;
}
int get_stateGetNedToBodyQuat_i_Qz_juav() {
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  return quat->qz;
}
int get_stateGetBodyRates_i_p_juav() {
  struct Int32Rates *body_rate = stateGetBodyRates_i();
  return body_rate->p;
}
int get_stateGetBodyRates_i_q_juav() {
  struct Int32Rates *body_rate = stateGetBodyRates_i();
  return body_rate->q;
}
int get_stateGetBodyRates_i_r_juav() {
  struct Int32Rates *body_rate = stateGetBodyRates_i();
  return body_rate->r;
}
void juav_stabilization_attitude_run_native(bool enable_integrator) {
  stabilization_attitude_run(enable_integrator);
}

void juav_stabilization_attitude_set_rpy_setpoint_i_native(int psi, int phi, int theta) {
  struct Int32Eulers rpy;
  rpy.psi = psi;
  rpy.phi = phi;
  rpy.theta = theta;
  stabilization_attitude_set_rpy_setpoint_i(&rpy);
}

int get_att_ref_quat_i_euler_psi_juav() {
  return att_ref_quat_i.euler.psi;
}
int get_att_ref_quat_i_euler_phi_juav() {
  return att_ref_quat_i.euler.phi;
}
int get_att_ref_quat_i_euler_theta_juav() {
  return att_ref_quat_i.euler.theta;
}
///
int get_att_ref_quat_i_model_two_zeta_omega_p_juav() {
  return att_ref_quat_i.model.two_zeta_omega.p;
}
int get_att_ref_quat_i_model_two_zeta_omega_q_juav() {
  return att_ref_quat_i.model.two_zeta_omega.q;
}
int get_att_ref_quat_i_model_two_zeta_omega_r_juav() {
  return att_ref_quat_i.model.two_zeta_omega.r;
}

int get_att_ref_quat_i_model_two_omega2_p_juav() {
  return att_ref_quat_i.model.two_omega2.p;
}
int get_att_ref_quat_i_model_two_omega2_q_juav() {
  return att_ref_quat_i.model.two_omega2.q;
}
int get_att_ref_quat_i_model_two_omega2_r_juav() {
  return att_ref_quat_i.model.two_omega2.r;
}

float get_att_ref_quat_i_model_zeta_p_juav() {
  return att_ref_quat_i.model.zeta.p;
}
float get_att_ref_quat_i_model_zeta_q_juav() {
  return att_ref_quat_i.model.zeta.q;
}
float get_att_ref_quat_i_model_zeta_r_juav() {
  return att_ref_quat_i.model.zeta.r;
}
float get_att_ref_quat_i_model_omega_p_juav() {
  return att_ref_quat_i.model.zeta.p;
}
float get_att_ref_quat_i_model_omega_q_juav() {
  return att_ref_quat_i.model.zeta.q;
}
float get_att_ref_quat_i_model_omega_r_juav() {
  return att_ref_quat_i.model.zeta.r;
}


int get_att_ref_quat_i_saturation_max_accel_p_juav() {
  return att_ref_quat_i.saturation.max_accel.p;
}
int get_att_ref_quat_i_saturation_max_accel_q_juav() {
  return att_ref_quat_i.saturation.max_accel.q;
}
int get_att_ref_quat_i_saturation_max_accel_r_juav() {
  return att_ref_quat_i.saturation.max_accel.r;
}
int get_att_ref_quat_i_saturation_max_rate_p_juav() {
  return att_ref_quat_i.saturation.max_rate.p;
}
int get_att_ref_quat_i_saturation_max_rate_q_juav() {
  return att_ref_quat_i.saturation.max_rate.q;
}
int get_att_ref_quat_i_saturation_max_rate_r_juav() {
  return att_ref_quat_i.saturation.max_rate.r;
}