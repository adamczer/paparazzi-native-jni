#include "nps_sensors.h"

#include "generated/airframe.h"
#include NPS_SENSORS_PARAMS

struct NpsSensors sensors;

void nps_sensors_init(double time)
{

  struct DoubleEulers body_to_imu_eulers =
  { NPS_BODY_TO_IMU_PHI, NPS_BODY_TO_IMU_THETA, NPS_BODY_TO_IMU_PSI };
  DOUBLE_RMAT_OF_EULERS(sensors.body_to_imu_rmat, body_to_imu_eulers);

  nps_sensor_gyro_init(&sensors.gyro, time);
  nps_sensor_accel_init(&sensors.accel, time);
  nps_sensor_mag_init(&sensors.mag, time);
  nps_sensor_baro_init(&sensors.baro, time);
  nps_sensor_gps_init(&sensors.gps, time);

}


void nps_sensors_run_step(double time)
{
  nps_sensor_gyro_run_step(&sensors.gyro, time, &sensors.body_to_imu_rmat);
  nps_sensor_accel_run_step(&sensors.accel, time, &sensors.body_to_imu_rmat);
  nps_sensor_mag_run_step(&sensors.mag, time, &sensors.body_to_imu_rmat);
  nps_sensor_baro_run_step(&sensors.baro, time);
  nps_sensor_gps_run_step(&sensors.gps, time);
  nps_sensor_sonar_run_step(&sensors.sonar, time);
}


bool_t nps_sensors_gyro_available(void)
{
  if (sensors.gyro.data_available) {
    sensors.gyro.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

bool_t nps_sensors_mag_available(void)
{
  if (sensors.mag.data_available) {
    sensors.mag.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

bool_t nps_sensors_baro_available(void)
{
  if (sensors.baro.data_available) {
    sensors.baro.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

bool_t nps_sensors_gps_available(void)
{
  if (sensors.gps.data_available) {
    sensors.gps.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

bool_t nps_sensors_sonar_available(void)
{
  if (sensors.sonar.data_available) {
    sensors.sonar.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

void UpdateSensorLatencySpeedJuav(double time, double cur_speed_reading_x, double cur_speed_reading_y, double cur_speed_reading_z) {
  struct DoubleVect3 cur_speed_reading;
  cur_speed_reading.x=cur_speed_reading_x;
  cur_speed_reading.y=cur_speed_reading_y;
  cur_speed_reading.z=cur_speed_reading_z;
  struct NpsSensorGps *gps = &sensors.gps;
  UpdateSensorLatency(time, &cur_speed_reading, &gps->speed_history, gps->speed_latency, &gps->ecef_vel);
}


void UpdateSensorLatencyPosJuav(double time, double pos_reading_x, double pos_reading_y, double pos_reading_z) {
  struct DoubleVect3 pos_reading;
  pos_reading.x=pos_reading_x;
  pos_reading.y=pos_reading_y;
  pos_reading.z=pos_reading_z;
  struct NpsSensorGps *gps = &sensors.gps;
  UpdateSensorLatency(time, &pos_reading, &gps->pos_history, gps->pos_latency, &gps->ecef_pos);
}

void UpdateSensorLatencyLlaJuav(double time, double lat, double lon, double alt) {
  struct LlaCoor_d cur_lla_reading;
  cur_lla_reading.lat=lat;
  cur_lla_reading.lon=lon;
  cur_lla_reading.alt=alt;
  struct NpsSensorGps *gps = &sensors.gps;
  UpdateSensorLatency(time, &cur_lla_reading, &gps->lla_history, gps->pos_latency, &gps->lla_pos);
}

void UpdateSensorLatency_Single_Hmsl(double time, double cur_hmsl_reading) {
  struct NpsSensorGps *gps = &sensors.gps;
  UpdateSensorLatency_Single(time, &cur_hmsl_reading, &gps->hmsl_history, gps->pos_latency, &gps->hmsl);
}
