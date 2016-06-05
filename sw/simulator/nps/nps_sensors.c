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

//  printf("body_to_imu_rmat = {%f,%f,%f,%f,%f,%f,%f,%f,%f}\n",
//         sensors.body_to_imu_rmat.m[0],
//         sensors.body_to_imu_rmat.m[1],
//         sensors.body_to_imu_rmat.m[2],
//         sensors.body_to_imu_rmat.m[3],
//         sensors.body_to_imu_rmat.m[4],
//         sensors.body_to_imu_rmat.m[5],
//         sensors.body_to_imu_rmat.m[6],
//         sensors.body_to_imu_rmat.m[7],
//         sensors.body_to_imu_rmat.m[8]);

//  printf("sensors.gyro = {\n"
//                 "gyro.value = {x=%f,y=%f,z=%f}\n"
//                 "gyro.min = %f\n"
//                 "gyro.max = %f\n"
//                 "gyro.bias_initial = {x=%f,y=%f,z=%f}\n"
//                 "gyro.bias_random_walk_std_dev = {x=%f,y=%f,z=%f}\n"
//                 "gyro.bias_random_walk_value = {x=%f,y=%f,z=%f}\n"
//                 "gyro.neutral = {x=%f,y=%f,z=%f}\n"
//                 "gyro.noise_std_dev = {x=%f,y=%f,z=%f}\n"
//                 "gyro.data_available = %d\n"
//                 "}",
//         sensors.gyro.value.x,sensors.gyro.value.y,sensors.gyro.value.z,
//         sensors.gyro.min,
//         sensors.gyro.max,
//         sensors.gyro.bias_initial.x,sensors.gyro.bias_initial.y,sensors.gyro.bias_initial.z,
//         sensors.gyro.bias_random_walk_std_dev.x,sensors.gyro.bias_random_walk_std_dev.y,sensors.gyro.bias_random_walk_std_dev.z,
//         sensors.gyro.bias_random_walk_value.x,sensors.gyro.bias_random_walk_value.y,sensors.gyro.bias_random_walk_value.z,
//         sensors.gyro.neutral.x,sensors.gyro.neutral.y,sensors.gyro.neutral.z,
//         sensors.gyro.noise_std_dev.x,sensors.gyro.noise_std_dev.y,sensors.gyro.noise_std_dev.z,
//         sensors.accel.data_available
//  );

//  printf("sensors.accel = {x=%f,y=%f,z=%f \n}",
//         sensors.accel.value.x,sensors.accel.value.y,sensors.accel.value.z
//  );
//
//  printf("sensors.gyro = {x=%f,y=%f,z=%f} \n ",
//         sensors.gyro.value.x,sensors.gyro.value.y,sensors.gyro.value.z
//  );
//
//  printf("sensors.mag = {x=%f,y=%f,z=%f} \n",
//         sensors.mag.value.x,sensors.mag.value.y,sensors.mag.value.z
//  );
//
//  printf("sensors.gps.ecef_pos = {x=%f,y=%f,z=%f} \n",
//         sensors.gps.ecef_pos.x,sensors.gps.ecef_pos.y,sensors.gps.ecef_pos.z
//  );

//  printf("sensors.accel = {\n"
//                 "accel.value = {x=%f,y=%f,z=%f}\n"
//                 "accel.min = %f\n"
//                 "accel.max = %f\n"
//                 "accel.bias = {x=%f,y=%f,z=%f}\n"
//                 "accel.neutral = {x=%f,y=%f,z=%f}\n"
//                 "accel.data_available = %d\n"
//                 "}",
//         sensors.accel.value.x,sensors.accel.value.y,sensors.accel.value.z,
//         sensors.accel.min,
//         sensors.accel.max,
//         sensors.accel.bias.x,sensors.accel.bias.y,sensors.accel.bias.z,
//         sensors.accel.neutral.x,sensors.accel.neutral.y,sensors.accel.neutral.z,
//         sensors.accel.data_available
//  );
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

void nps_sensor_gyro_init_juav(double time) {
nps_sensor_gyro_init(&sensors.gyro, time);
}
void nps_sensor_accel_init_juav(double time){
nps_sensor_accel_init(&sensors.accel, time);
}
void nps_sensor_mag_init_juav(double time){
nps_sensor_mag_init(&sensors.mag, time);
}
void nps_sensor_baro_init_juav(double time){
nps_sensor_baro_init(&sensors.baro, time);
}
void nps_sensor_gps_init_juav(double time){
nps_sensor_gps_init(&sensors.gps, time);
}

void nps_sensor_gyro_run_step_juav(double time) {
  nps_sensor_gyro_run_step(&sensors.gyro, time, &sensors.body_to_imu_rmat);
}
void nps_sensor_accel_run_step_juav(double time) {
  nps_sensor_accel_run_step(&sensors.accel, time, &sensors.body_to_imu_rmat);
}
void nps_sensor_mag_run_step_juav(double time) {
  nps_sensor_mag_run_step(&sensors.mag, time, &sensors.body_to_imu_rmat);
}
void nps_sensor_baro_run_step_juav(double time) {
  nps_sensor_baro_run_step(&sensors.baro, time);
}
void nps_sensor_gps_run_step_juav(double time) {
  nps_sensor_gps_run_step(&sensors.gps, time);
//  printf("gps {\necef_pos {x=%f,y=%f,z=%f}\necef_vel {x=%f,y=%f,z=%f}\nlla_pos {lat=%f,lon=%f,alt=%f}\nspeed_latency=%f",
//  sensors.gps.ecef_pos.x,sensors.gps.ecef_pos.y,sensors.gps.ecef_pos.z,
//         sensors.gps.ecef_vel.x,sensors.gps.ecef_vel.y,sensors.gps.ecef_vel.z,
//         sensors.gps.hmsl,
//         sensors.gps.lla_pos.lat,sensors.gps.lla_pos.lon,sensors.gps.lla_pos.alt,
//         sensors.gps.speed_latency
//
//  );
}
void nps_sensor_sonar_run_step_juav(double time) {
  nps_sensor_sonar_run_step(&sensors.sonar, time);
}
