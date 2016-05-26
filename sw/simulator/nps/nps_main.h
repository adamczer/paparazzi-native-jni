#ifndef NPS_MAIN_H
#define NPS_MAIN_H

extern void nps_set_time_factor(float time_factor);
void nps_main_init_juav();
int fake_nps_main_periodic();
double get_nps_sim_time_juav();
void nps_main_periodic_juav_native();
void set_nps_sim_time_juav(double d);
void nps_main_display_juav();
double get_nps_host_time_elapsed_juav();
double get_nps_main_real_initial_time_juav();
double get_nps_host_time_now_juav();
double get_nps_display_time_juav();
void set_nps_display_time_juav(double d);
void nps_main_run_sim_step_juav();
//setting vars to defaults in struct.
void nps_main_parse_options_juav();

// Fdm body inertual rot vel
double get_fdm_body_inetrial_rot_vel_p_juav();
double get_fdm_body_inetrial_rot_vel_q_juav();
double get_fdm_body_inetrial_rot_vel_r_juav();
// FdmBodyAccel
double get_fdm_body_accel_x_juav();
double get_fdm_body_accel_y_juav();
double get_fdm_body_accel_z_juav();
//FdmBodyToImu
double get_fdm_body_to_imu_juav(int row, int col);
//FDM Ltp To Body Quat;
double get_fdm_ltp_to_body_quat_qi_juav();
double get_fdm_ltp_to_body_quat_qx_juav();
double get_fdm_ltp_to_body_quat_qy_juav();
double get_fdm_ltp_to_body_quat_qz_juav();
//FDM Ltp H
double get_fdm_ltp_h_x_juav();
double get_fdm_ltp_h_y_juav();
double get_fdm_ltp_h_z_juav();
//FDM hmsl
double get_fdm_hmsl_juav();
//Fdm Ecef Ecef Vel
double get_fdm_ecef_ecef_vel_x_juav();
double get_fdm_ecef_ecef_vel_y_juav();
double get_fdm_ecef_ecef_vel_z_juav();
//Fdm Ecef Pos
double get_fdm_ecef_pos_x_juav();
double get_fdm_ecef_pos_y_juav();
double get_fdm_ecef_pos_z_juav();
//Fdm agl
double get_fdm_agl_juav();


#endif /* NPS_MAIN_H */
