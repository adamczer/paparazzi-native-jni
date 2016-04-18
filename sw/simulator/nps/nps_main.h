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

#endif /* NPS_MAIN_H */
