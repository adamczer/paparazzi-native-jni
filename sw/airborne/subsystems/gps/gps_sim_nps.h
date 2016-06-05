#ifndef GPS_SIM_NPS_H
#define GPS_SIM_NPS_H

#include "std.h"

#define GPS_NB_CHANNELS 16

extern bool_t gps_has_fix;

extern void gps_feed_value();

extern void gps_impl_init();

void gps_feed_value_week_juav(int week);
void gps_feed_value_tow_juav(double time);
void gps_feed_value_ecef_pos_juav(double ecef_pos_x, double ecef_pos_y, double ecef_pos_z);
void gps_feed_value_ecef_vel_juav(double ecef_vel_x, double ecef_vel_y, double ecef_vel_z);
void gps_feed_value_lla_pos_juav(double lla_pos_lat, double lla_pos_lon, double lla_pos_alt);
void gps_feed_value_hmsl_juav(double hmsl);
void gps_feed_value_ned_speed(double ned_vel_x, double ned_vel_y, double ned_vel_z);
void gps_feed_value_finalize_juav();
void  gps_feed_value_juav(int week, double time, double ecef_pos_x, double ecef_pos_y, double ecef_pos_z,
                          double ecef_vel_x, double ecef_vel_y, double ecef_vel_z,
                          double lla_pos_lat, double lla_pos_lon, double lla_pos_alt,
                          double hmsl
);

#define GpsEvent() {}

#endif /* GPS_SIM_NPS_H */
