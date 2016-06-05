/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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

#include "subsystems/gps/gps_sim_nps.h"
#include "subsystems/gps.h"
#include "subsystems/abi.h"
#include "nps_sensors.h"
#include "nps_fdm.h"

#if GPS_USE_LATLONG
/* currently needed to get nav_utm_zone0 */
#include "subsystems/navigation/common_nav.h"
#include "math/pprz_geodetic_float.h"
#endif

bool_t gps_has_fix;

void  gps_feed_value()
{
  // FIXME, set proper time instead of hardcoded to May 2014
  gps.week = 1794;
  gps.tow = fdm.time * 1000;

  gps.ecef_pos.x = sensors.gps.ecef_pos.x * 100.;
  gps.ecef_pos.y = sensors.gps.ecef_pos.y * 100.;
  gps.ecef_pos.z = sensors.gps.ecef_pos.z * 100.;

//  printf("sensors.gps.ecef_pos.x = %f\n",sensors.gps.ecef_pos.x);
//  printf("sensors.gps.ecef_pos.y = %f\n",sensors.gps.ecef_pos.y);
//  printf("sensors.gps.ecef_pos.z = %f\n",sensors.gps.ecef_pos.z);
  gps.ecef_vel.x = sensors.gps.ecef_vel.x * 100.;
  gps.ecef_vel.y = sensors.gps.ecef_vel.y * 100.;
  gps.ecef_vel.z = sensors.gps.ecef_vel.z * 100.;

//  printf("sensors.gps.ecef_vel.x = %f\n",sensors.gps.ecef_vel.x);
//  printf("sensors.gps.ecef_vel.y = %f\n",sensors.gps.ecef_vel.y);
//  printf("sensors.gps.ecef_vel.z = %f\n",sensors.gps.ecef_vel.z);
  //ecef pos seems to be based on geocentric model, hence we get a very high alt when converted to lla
  gps.lla_pos.lat = DegOfRad(sensors.gps.lla_pos.lat) * 1e7;

//  printf("sensors.gps.lla_pos.lat = %f\n",sensors.gps.lla_pos.lat);
//  printf("DegOfRad(sensors.gps.lla_pos.lat) * 1e7 = %f\n",DegOfRad(sensors.gps.lla_pos.lat) * 1e7);
  gps.lla_pos.lon = DegOfRad(sensors.gps.lla_pos.lon) * 1e7;
//  printf("sensors.gps.lla_pos.lon = %f\n",sensors.gps.lla_pos.lon);
//  printf("DegOfRad(sensors.gps.lla_pos.lon) * 1e7 = %f\n",DegOfRad(sensors.gps.lla_pos.lon) * 1e7);
  gps.lla_pos.alt = sensors.gps.lla_pos.alt * 1000.;

//  printf("sensors.gps.lla_pos.alt * 1000. = %f\n",sensors.gps.lla_pos.alt * 1000.);

  gps.hmsl        = sensors.gps.hmsl * 1000.;
//  printf("sensors.gps.hmsl * 1000. = %f\n",sensors.gps.lla_pos.alt * 1000.);


  /* calc NED speed from ECEF */
  struct LtpDef_d ref_ltp;
  ltp_def_from_ecef_d(&ref_ltp, &sensors.gps.ecef_pos);
  struct NedCoor_d ned_vel_d;
  ned_of_ecef_vect_d(&ned_vel_d, &ref_ltp, &sensors.gps.ecef_vel);
//  printf("ned_vel_d.x = %f\n",ned_vel_d.x);
//  printf("ned_vel_d.y = %f\n",ned_vel_d.y);
//  printf("ned_vel_d.z = %f\n",ned_vel_d.z);
  gps.ned_vel.x = ned_vel_d.x * 100;
  gps.ned_vel.y = ned_vel_d.y * 100;
  gps.ned_vel.z = ned_vel_d.z * 100;

  /* horizontal and 3d ground speed in cm/s */
  gps.gspeed = sqrt(ned_vel_d.x * ned_vel_d.x + ned_vel_d.y * ned_vel_d.y) * 100;
//  printf("gps.gspeed = %f\n",sqrt(ned_vel_d.x * ned_vel_d.x + ned_vel_d.y * ned_vel_d.y) * 100);

  gps.speed_3d = sqrt(ned_vel_d.x * ned_vel_d.x + ned_vel_d.y * ned_vel_d.y + ned_vel_d.z * ned_vel_d.z) * 100;
//  printf("gps.speed_3d = %f\n",sqrt(ned_vel_d.x * ned_vel_d.x + ned_vel_d.y * ned_vel_d.y + ned_vel_d.z * ned_vel_d.z) * 100);


  /* ground course in radians * 1e7 */
  gps.course = atan2(ned_vel_d.y, ned_vel_d.x) * 1e7;

//#if GPS_USE_LATLONG
//  /* Computes from (lat, long) in the referenced UTM zone */
//  struct LlaCoor_f lla_f;
//  LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
//  struct UtmCoor_f utm_f;
//  utm_f.zone = nav_utm_zone0;
//  /* convert to utm */
//  utm_of_lla_f(&utm_f, &lla_f);
//  /* copy results of utm conversion */
//  gps.utm_pos.east = utm_f.east * 100;
//  gps.utm_pos.north = utm_f.north * 100;
//  gps.utm_pos.alt = gps.lla_pos.alt;
//  gps.utm_pos.zone = nav_utm_zone0;
//#endif

  if (gps_has_fix) {
    gps.fix = GPS_FIX_3D;
  } else {
    gps.fix = GPS_FIX_NONE;
  }

  // publish gps data
  uint32_t now_ts = get_sys_time_usec();
  gps.last_msg_ticks = sys_time.nb_sec_rem;
  gps.last_msg_time = sys_time.nb_sec;
  if (gps.fix == GPS_FIX_3D) {
    gps.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps.last_3dfix_time = sys_time.nb_sec;
  }

  printf("gps {\n last_msg_ticks = %d,\n last_msg_time = %f,\n ecef_pos = x=%f, y=%f,z=%f,\n ecef_vel = x=%f,y=%f,z=%f,\n lla_pos = x=%f,y=%f,z=%f,\n utm_pos = north=%f,east=%f,alt=%f,zone=%d\n}\n"
          ,gps.last_msg_ticks
          ,gps.last_msg_time
          ,gps.ecef_pos.x,gps.ecef_pos.y,gps.ecef_pos.z
          ,gps.ecef_vel.x,gps.ecef_vel.y,gps.ecef_vel.z
          ,gps.lla_pos.lat,gps.lla_pos.lon,gps.lla_pos.alt
          ,gps.utm_pos.north,gps.utm_pos.east,gps.utm_pos.alt,gps.utm_pos.zone);
  AbiSendMsgGPS(GPS_SIM_ID, now_ts, &gps);
}

void gps_impl_init()
{
  gps_has_fix = TRUE;
}

void gps_feed_value_week_juav(int week) {
  gps.week = week;
}
void gps_feed_value_tow_juav(double time) {
  gps.tow = time;
}
void gps_feed_value_ecef_pos_juav(double ecef_pos_x, double ecef_pos_y, double ecef_pos_z) {
  gps.ecef_pos.x = ecef_pos_x;
  gps.ecef_pos.y = ecef_pos_y;
  gps.ecef_pos.z = ecef_pos_z;
}
void gps_feed_value_ecef_vel_juav(double ecef_vel_x, double ecef_vel_y, double ecef_vel_z) {
  gps.ecef_vel.x = ecef_vel_x;
  gps.ecef_vel.y = ecef_vel_y;
  gps.ecef_vel.z = ecef_vel_z;
}
void gps_feed_value_lla_pos_juav(double lla_pos_lat, double lla_pos_lon, double lla_pos_alt) {
  gps.lla_pos.lat = lla_pos_lat;
  gps.lla_pos.lon = lla_pos_lon;
  gps.lla_pos.alt = lla_pos_alt;
}
void gps_feed_value_hmsl_juav(double hmsl) {
  gps.hmsl = hmsl;
}
void gps_feed_value_ned_speed(double ned_vel_x, double ned_vel_y, double ned_vel_z) {
  gps.ned_vel.x = ned_vel_x * 100;
  gps.ned_vel.y = ned_vel_y * 100;
  gps.ned_vel.z = ned_vel_z * 100;

  /* horizontal and 3d ground speed in cm/s */
  gps.gspeed = sqrt(ned_vel_x * ned_vel_x + ned_vel_y * ned_vel_y) * 100;
  gps.speed_3d = sqrt(ned_vel_x * ned_vel_x + ned_vel_y * ned_vel_y + ned_vel_z * ned_vel_z) * 100;

  /* ground course in radians * 1e7 */
  gps.course = atan2(ned_vel_y, ned_vel_x) * 1e7;
}

void gps_feed_value_finalize_juav() {

//#if GPS_USE_LATLONG
//  /* Computes from (lat, long) in the referenced UTM zone */
//  struct LlaCoor_f lla_f;
//  LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
//  struct UtmCoor_f utm_f;
//  utm_f.zone = nav_utm_zone0;
//  /* convert to utm */
//  utm_of_lla_f(&utm_f, &lla_f);
//  /* copy results of utm conversion */
//  gps.utm_pos.east = utm_f.east * 100;
//  gps.utm_pos.north = utm_f.north * 100;
//  gps.utm_pos.alt = gps.lla_pos.alt;
//  gps.utm_pos.zone = nav_utm_zone0;
//#endif

  if (gps_has_fix) {
    gps.fix = GPS_FIX_3D;
  } else {
    gps.fix = GPS_FIX_NONE;
  }

  // publish gps data
  uint32_t now_ts = get_sys_time_usec();
  gps.last_msg_ticks = sys_time.nb_sec_rem;
  gps.last_msg_time = sys_time.nb_sec;
  if (gps.fix == GPS_FIX_3D) {
    gps.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps.last_3dfix_time = sys_time.nb_sec;
  }
  printf("gps {\n last_msg_ticks = %d,\n last_msg_time = %f,\n ecef_pos = x=%f, y=%f,z=%f,\n ecef_vel = x=%f,y=%f,z=%f,\n lla_pos = x=%f,y=%f,z=%f,\n utm_pos = north=%f,east=%f,alt=%f,zone=%d\n}\n"
          ,gps.last_msg_ticks
          ,gps.last_msg_time
          ,gps.ecef_pos.x,gps.ecef_pos.y,gps.ecef_pos.z
          ,gps.ecef_vel.x,gps.ecef_vel.y,gps.ecef_vel.z
          ,gps.lla_pos.lat,gps.lla_pos.lon,gps.lla_pos.alt
          ,gps.utm_pos.north,gps.utm_pos.east,gps.utm_pos.alt,gps.utm_pos.zone);
  AbiSendMsgGPS(GPS_SIM_ID, now_ts, &gps);
}

void  gps_feed_value_juav(int week, double time, double ecef_pos_x, double ecef_pos_y, double ecef_pos_z,
                          double ecef_vel_x, double ecef_vel_y, double ecef_vel_z,
                          double lla_pos_lat, double lla_pos_lon, double lla_pos_alt,
                          double hmsl
)
{
  // FIXME, set proper time instead of hardcoded to May 2014
  gps.week = week;
  gps.tow = time *1000;

  gps.ecef_pos.x = ecef_pos_x * 100.;
  gps.ecef_pos.y = ecef_pos_y * 100.;
  gps.ecef_pos.z = ecef_pos_z * 100.;

  gps.ecef_vel.x = ecef_vel_x * 100.;
  gps.ecef_vel.y = ecef_vel_y * 100.;
  gps.ecef_vel.z = ecef_vel_z * 100.;

  gps.lla_pos.lat = DegOfRad(lla_pos_lat) * 1e7;

  gps.lla_pos.lon = DegOfRad(lla_pos_lon) * 1e7;
  gps.lla_pos.alt = lla_pos_alt * 1000.;

  gps.hmsl        = hmsl * 1000.;

  /* calc NED speed from ECEF */
  struct EcefCoor_d ecef_pos;
    ecef_pos.x=ecef_pos_x;
    ecef_pos.y=ecef_pos_y;
    ecef_pos.z=ecef_pos_z;
  struct LtpDef_d ref_ltp;
  ltp_def_from_ecef_d(&ref_ltp, &ecef_pos);
  struct NedCoor_d ned_vel_d;
  struct DoubleVect3 ecef_vel;
  ecef_vel.x=ecef_vel_x;
  ecef_vel.y=ecef_vel_y;
  ecef_vel.z=ecef_vel_z;
  ned_of_ecef_vect_d(&ned_vel_d, &ref_ltp, &ecef_vel);
  gps.ned_vel.x = ned_vel_d.x * 100;
  gps.ned_vel.y = ned_vel_d.y * 100;
  gps.ned_vel.z = ned_vel_d.z * 100;

  /* horizontal and 3d ground speed in cm/s */
  gps.gspeed = sqrt(ned_vel_d.x * ned_vel_d.x + ned_vel_d.y * ned_vel_d.y) * 100;

  gps.speed_3d = sqrt(ned_vel_d.x * ned_vel_d.x + ned_vel_d.y * ned_vel_d.y + ned_vel_d.z * ned_vel_d.z) * 100;


  /* ground course in radians * 1e7 */
  gps.course = atan2(ned_vel_d.y, ned_vel_d.x) * 1e7;

#if GPS_USE_LATLONG
  /* Computes from (lat, long) in the referenced UTM zone */
  struct LlaCoor_f lla_f;
  LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
  struct UtmCoor_f utm_f;
  utm_f.zone = nav_utm_zone0;
  /* convert to utm */
  utm_of_lla_f(&utm_f, &lla_f);
  /* copy results of utm conversion */
  gps.utm_pos.east = utm_f.east * 100;
  gps.utm_pos.north = utm_f.north * 100;
  gps.utm_pos.alt = gps.lla_pos.alt;
  gps.utm_pos.zone = nav_utm_zone0;
#endif

  if (gps_has_fix) {
    gps.fix = GPS_FIX_3D;
  } else {
    gps.fix = GPS_FIX_NONE;
  }

  // publish gps data
  uint32_t now_ts = get_sys_time_usec();
  gps.last_msg_ticks = sys_time.nb_sec_rem;
  gps.last_msg_time = sys_time.nb_sec;
  if (gps.fix == GPS_FIX_3D) {
    gps.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps.last_3dfix_time = sys_time.nb_sec;
  }

  printf("gps {\n last_msg_ticks = %d,\n last_msg_time = %f,\n ecef_pos = x=%f, y=%f,z=%f,\n ecef_vel = x=%f,y=%f,z=%f,\n lla_pos = x=%f,y=%f,z=%f,\n utm_pos = north=%f,east=%f,alt=%f,zone=%d\n}\n"
          ,gps.last_msg_ticks
          ,gps.last_msg_time
          ,gps.ecef_pos.x,gps.ecef_pos.y,gps.ecef_pos.z
          ,gps.ecef_vel.x,gps.ecef_vel.y,gps.ecef_vel.z
          ,gps.lla_pos.lat,gps.lla_pos.lon,gps.lla_pos.alt
          ,gps.utm_pos.north,gps.utm_pos.east,gps.utm_pos.alt,gps.utm_pos.zone);
  AbiSendMsgGPS(GPS_SIM_ID, now_ts, &gps);
}

