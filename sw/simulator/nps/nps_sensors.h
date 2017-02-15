#ifndef NPS_SENSORS_H
#define NPS_SENSORS_H

#include "math/pprz_algebra.h"
#include "nps_sensor_gyro.h"
#include "nps_sensor_accel.h"
#include "nps_sensor_mag.h"
#include "nps_sensor_baro.h"
#include "nps_sensor_gps.h"
#include "nps_sensor_sonar.h"

struct NpsSensors {
  struct DoubleRMat body_to_imu_rmat;
  struct NpsSensorGyro  gyro;
  struct NpsSensorAccel accel;
  struct NpsSensorMag   mag;
  struct NpsSensorBaro  baro;
  struct NpsSensorGps   gps;
  struct NpsSensorSonar sonar;
};

extern struct NpsSensors sensors;

extern void nps_sensors_init(double time);
extern void nps_sensors_run_step(double time);

extern bool_t nps_sensors_gyro_available();
extern bool_t nps_sensors_mag_available();
extern bool_t nps_sensors_baro_available();
extern bool_t nps_sensors_gps_available();
extern bool_t nps_sensors_sonar_available();


void UpdateSensorLatencySpeedJuav(double time, double cur_speed_reading_x, double cur_speed_reading_y, double cur_speed_reading_z);
void UpdateSensorLatencyPosJuav(double time, double pos_reading_x, double pos_reading_y, double pos_reading_z);
void UpdateSensorLatencyLlaJuav(double time, double lat, double lon, double alt);
void UpdateSensorLatency_Single_Hmsl(double time, double cur_hmsl_reading);

void nps_sensor_gyro_init_juav(double time);
void nps_sensor_accel_init_juav(double time);
void nps_sensor_mag_init_juav(double time);
void nps_sensor_baro_init_juav(double time);
void nps_sensor_gps_init_juav(double time);

void nps_sensor_gyro_run_step_juav(double time);
void nps_sensor_accel_run_step_juav(double time);
void nps_sensor_mag_run_step_juav(double time);
void nps_sensor_baro_run_step_juav(double time);
void nps_sensor_gps_run_step_juav(double time);
void nps_sensor_sonar_run_step_juav(double time);

#endif /* NPS_SENSORS_H */
