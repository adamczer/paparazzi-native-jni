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

#ifndef STABILIZATION_ATTITUDE_QUAT_INT_H
#define STABILIZATION_ATTITUDE_QUAT_INT_H

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"

#include "math/pprz_algebra_int.h"

extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC

extern struct AttRefQuatInt att_ref_quat_i;

/* settings handlers for ref model params */
#define stabilization_attitude_quat_int_SetOmegaP(_val) {   \
    attitude_ref_quat_int_set_omega_p(&att_ref_quat_i, _val);   \
  }
#define stabilization_attitude_quat_int_SetOmegaQ(_val) {   \
    attitude_ref_quat_int_set_omega_q(&att_ref_quat_i, _val);   \
  }
#define stabilization_attitude_quat_int_SetOmegaR(_val) {   \
    attitude_ref_quat_int_set_omega_r(&att_ref_quat_i, _val);   \
  }

#define stabilization_attitude_quat_int_SetZetaP(_val) {    \
    attitude_ref_quat_int_set_zeta_p(&att_ref_quat_i, _val);    \
  }
#define stabilization_attitude_quat_int_SetZetaQ(_val) {    \
    attitude_ref_quat_int_set_zeta_q(&att_ref_quat_i, _val);    \
  }
#define stabilization_attitude_quat_int_SetZetaR(_val) {    \
    attitude_ref_quat_int_set_zeta_r(&att_ref_quat_i, _val);    \
  }

void attitude_ref_quat_int_update_juav(float dt);

int get_stabilization_att_sum_err_quat_i_juav();
int get_stabilization_att_sum_err_quat_x_juav();
int get_stabilization_att_sum_err_quat_y_juav();
int get_stabilization_att_sum_err_quat_z_juav();

void set_stabilization_att_sum_err_quat_i_juav(int qi);
void set_stabilization_att_sum_err_quat_x_juav(int qx);
void set_stabilization_att_sum_err_quat_y_juav(int qy);
void set_stabilization_att_sum_err_quat_z_juav(int qz);

// input params used in computation
int get_att_ref_quat_i_quat_qi_juav();
int get_att_ref_quat_i_quat_qx_juav();
int get_att_ref_quat_i_quat_qy_juav();
int get_att_ref_quat_i_quat_qz_juav();
int get_att_ref_quat_i_rate_p_juav();
int get_att_ref_quat_i_rate_q_juav();
int get_att_ref_quat_i_rate_r_juav();
int get_att_ref_quat_i_accel_p_juav();
int get_att_ref_quat_i_accel_q_juav();
int get_att_ref_quat_i_accel_r_juav();


int get_stabilization_gains_p_x_juav();
int get_stabilization_gains_p_y_juav();
int get_stabilization_gains_p_z_juav();
int get_stabilization_gains_d_x_juav();
int get_stabilization_gains_d_y_juav();
int get_stabilization_gains_d_z_juav();
int get_stabilization_gains_dd_x_juav();
int get_stabilization_gains_dd_y_juav();
int get_stabilization_gains_dd_z_juav();
int get_stabilization_gains_i_x_juav();
int get_stabilization_gains_i_y_juav();
int get_stabilization_gains_i_z_juav();


// set the results back in c
void set_att_ref_quat_i_quat_qi_juav(int i);
void set_att_ref_quat_i_quat_qx_juav(int i);
void set_att_ref_quat_i_quat_qy_juav(int i);
void set_att_ref_quat_i_quat_qz_juav(int i);
void set_att_ref_quat_i_rate_p_juav(int i);
void set_att_ref_quat_i_rate_q_juav(int i);
void set_att_ref_quat_i_rate_r_juav(int i);
void set_att_ref_quat_i_accel_p_juav(int i);
void set_att_ref_quat_i_accel_q_juav(int i);
void set_att_ref_quat_i_accel_r_juav(int i);


int get_stateGetNedToBodyQuat_i_Qi_juav();
int get_stateGetNedToBodyQuat_i_Qx_juav();
int get_stateGetNedToBodyQuat_i_Qy_juav();
int get_stateGetNedToBodyQuat_i_Qz_juav();


int get_stateGetBodyRates_i_p_juav();
int get_stateGetBodyRates_i_q_juav();
int get_stateGetBodyRates_i_r_juav();

void set_stabilization_cmd(int yaw,int pitch, int roll);

#endif /* STABILIZATION_ATTITUDE_QUAT_INT_H */
