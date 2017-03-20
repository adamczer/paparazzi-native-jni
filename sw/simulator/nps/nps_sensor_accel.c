#include "nps_sensor_accel.h"

#include "generated/airframe.h" /* to get NPS_SENSORS_PARAMS */

#include "nps_fdm.h"
#include "nps_random.h"
#include NPS_SENSORS_PARAMS
#include "math/pprz_algebra_int.h"

void nps_sensor_accel_init(struct NpsSensorAccel *accel, double time)
{
//  printf("NPS_ACCEL_MIN = %d\n",NPS_ACCEL_MIN);
//  printf("NPS_ACCEL_MAX = %d\n",NPS_ACCEL_MAX);
//  printf("IMU_ACCEL_X_SIGN = %f\n",IMU_ACCEL_X_SIGN);
//  printf("IMU_ACCEL_Y_SIGN = %f\n",IMU_ACCEL_Y_SIGN);
//  printf("IMU_ACCEL_Z_SIGN = %f\n",IMU_ACCEL_Z_SIGN);
//  printf("IMU_ACCEL_X_NEUTRAL = %f\n",IMU_ACCEL_X_NEUTRAL);
//  printf("IMU_ACCEL_Y_NEUTRAL = %f\n",IMU_ACCEL_Y_NEUTRAL);
//  printf("IMU_ACCEL_Z_NEUTRAL = %f\n",IMU_ACCEL_Z_NEUTRAL);
//  printf("NPS_ACCEL_NOISE_STD_DEV_X = %f\n",NPS_ACCEL_NOISE_STD_DEV_X);
//  printf("NPS_ACCEL_NOISE_STD_DEV_Y = %f\n",NPS_ACCEL_NOISE_STD_DEV_Y);
//  printf("NPS_ACCEL_NOISE_STD_DEV_Z = %f\n",NPS_ACCEL_NOISE_STD_DEV_Z);
//    printf("NPS_ACCEL_BIAS_X = %f\n",NPS_ACCEL_BIAS_X);
//    printf("NPS_ACCEL_BIAS_Y = %f\n",NPS_ACCEL_BIAS_Y);
//    printf("NPS_ACCEL_BIAS_Z = %f\n",NPS_ACCEL_BIAS_Z);
//    printf("IMU_ACCEL_X_SENS = %f\n",IMU_ACCEL_X_SENS);
//    printf("IMU_ACCEL_Y_SENS = %f\n",IMU_ACCEL_Y_SENS);
//    printf("IMU_ACCEL_Z_SENS = %f\n",IMU_ACCEL_Z_SENS);
//    printf("NPS_ACCEL_DT = %f\n",NPS_ACCEL_DT);

  FLOAT_VECT3_ZERO(accel->value);
  accel->min = NPS_ACCEL_MIN;
  accel->max = NPS_ACCEL_MAX;
  FLOAT_MAT33_DIAG(accel->sensitivity,
                   NPS_ACCEL_SENSITIVITY_XX, NPS_ACCEL_SENSITIVITY_YY, NPS_ACCEL_SENSITIVITY_ZZ);
  VECT3_ASSIGN(accel->neutral,
               NPS_ACCEL_NEUTRAL_X, NPS_ACCEL_NEUTRAL_Y, NPS_ACCEL_NEUTRAL_Z);
  VECT3_ASSIGN(accel->noise_std_dev,
               NPS_ACCEL_NOISE_STD_DEV_X, NPS_ACCEL_NOISE_STD_DEV_Y, NPS_ACCEL_NOISE_STD_DEV_Z);
  VECT3_ASSIGN(accel->bias,
               NPS_ACCEL_BIAS_X, NPS_ACCEL_BIAS_Y, NPS_ACCEL_BIAS_Z);
  accel->next_update = time;
  accel->data_available = FALSE;
}

void nps_sensor_accel_run_step(struct NpsSensorAccel *accel, double time, struct DoubleRMat *body_to_imu)
{
  if (time < accel->next_update) {
    return;
  }
  /* transform to imu frame */
  struct DoubleVect3 accelero_imu;
//  printf("body_accel xyz= %f,%f,%f\n",fdm.body_accel.x,fdm.body_accel.y,fdm.body_accel.z);
  MAT33_VECT3_MUL(accelero_imu, *body_to_imu, fdm.body_accel);
//  printf("body_to_imu = \n%f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",body_to_imu[0],body_to_imu[1],body_to_imu[2],body_to_imu[3],body_to_imu[4],body_to_imu[5],body_to_imu[6],body_to_imu[7],body_to_imu[8]);

  /* compute accelero readings */
  MAT33_VECT3_MUL(accel->value, accel->sensitivity, accelero_imu);
  VECT3_ADD(accel->value, accel->neutral);

  /* Compute sensor error */
  struct DoubleVect3 accelero_error;
  /* constant bias */
  VECT3_COPY(accelero_error, accel->bias);
  /* white noise   */
  double_vect3_add_gaussian_noise(&accelero_error, &accel->noise_std_dev);
  /* scale */
  struct DoubleVect3 gain = {accel->sensitivity.m[0], accel->sensitivity.m[4], accel->sensitivity.m[8]};
  VECT3_EW_MUL(accelero_error, accelero_error, gain);
  /* add error */
  VECT3_ADD(accel->value, accelero_error);

  /* round signal to account for adc discretisation */
  DOUBLE_VECT3_ROUND(accel->value);
  /* saturate                                       */
//  printf("NPS_ACCEL_MIN = %f\n", accel->min);
//  printf("NPS_ACCEL_MAX = %f\n", accel->max);
  //FIXME: some magic is happening here ;(
  VECT3_BOUND_CUBE(accel->value, accel->min, accel->max);
//  printf("accel->value.x = %f\n",accel->value.x);
//  printf("accel->value.y = %f\n",accel->value.y);
//  printf("accel->value.z = %f\n",accel->value.z);

  accel->next_update += NPS_ACCEL_DT;
  accel->data_available = TRUE;
}

