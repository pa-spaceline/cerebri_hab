/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include "attitude_estimator.h"
#include "casadi.h"
#include "hab.h"

/* Magnetic declination at the launch site, radians, east-positive - same
 * placeholder pattern as cerebri/app/hab's decl_WL. Update for wherever the
 * balloon actually launches from before flight. */
static const double RDD2_MAG_DECL_RAD = 0.0;

static const double RDD2_ATTITUDE_ACCEL_GAIN = 40.0 * 1e-3;
static const double RDD2_ATTITUDE_MAG_GAIN = 40.0 * 1e-3;

/* Standard ZYX (yaw-pitch-roll) quaternion-to-Euler extraction, q = [w, x,
 * y, z] Hamilton convention - matches cyecca's SO3Quat parameterization and
 * the SO3EulerB321 (yaw-Z, pitch-Y, roll-X) sequence used throughout hab.py,
 * so it stays consistent with attitude_init/yaw_init's own Euler-to-quat
 * construction. */
static void quat_to_euler(const double q[4], float *roll, float *pitch,
                          float *yaw) {
  double w = q[0], x = q[1], y = q[2], z = q[3];
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  double sinp = 2.0 * (w * y - z * x);
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);

  if (sinp > 1.0) {
    sinp = 1.0;
  } else if (sinp < -1.0) {
    sinp = -1.0;
  }

  *roll = (float)atan2(sinr_cosp, cosr_cosp);
  *pitch = (float)asin(sinp);
  *yaw = (float)atan2(siny_cosp, cosy_cosp);
}

void rdd2_attitude_estimator_init(struct rdd2_attitude_estimator *estimator) {
  if (estimator == NULL) {
    return;
  }

  estimator->q[0] = 1.0;
  estimator->q[1] = 0.0;
  estimator->q[2] = 0.0;
  estimator->q[3] = 0.0;
  for (int i = 0; i < 6; i++) {
    estimator->p_att[i] = 0.0;
  }
}

void rdd2_attitude_estimator_reset(struct rdd2_attitude_estimator *estimator,
                                   const rdd2_vec3f_t *accel,
                                   const rdd2_vec3f_t *mag, bool mag_valid) {
  if (estimator == NULL || accel == NULL) {
    return;
  }

  if (!mag_valid || mag == NULL) {
    /* No magnetometer data yet - leave attitude as-is (whatever init() or
     * the last successful reset produced) rather than reconstructing a
     * partial, yaw-less attitude by hand. predict() below still integrates
     * gyro every cycle regardless, so this only skips the correction step
     * for this one call. */
    return;
  }

  {
    CASADI_FUNC_ARGS(attitude_init)

    double mag_b[3] = {(double)mag->x, (double)mag->y, (double)mag->z};
    double accel_b[3] = {(double)accel->x, (double)accel->y,
                         (double)accel->z};

    args[0] = mag_b;
    args[1] = accel_b;
    args[2] = &RDD2_MAG_DECL_RAD;

    res[0] = estimator->q;

    CASADI_FUNC_CALL(attitude_init)
  }

  for (int i = 0; i < 6; i++) {
    estimator->p_att[i] = 0.0;
  }
}

void rdd2_attitude_estimator_predict(struct rdd2_attitude_estimator *estimator,
                                     const rdd2_vec3f_t *gyro,
                                     const rdd2_vec3f_t *accel,
                                     const rdd2_vec3f_t *mag, bool mag_valid,
                                     float dt) {
  if (estimator == NULL || gyro == NULL || accel == NULL || dt <= 0.0f) {
    return;
  }

  double dt_d = (double)dt;
  double omega_b[3] = {(double)gyro->x, (double)gyro->y, (double)gyro->z};

  {
    CASADI_FUNC_ARGS(attitude_propagate)

    args[0] = estimator->q;
    args[1] = omega_b;
    args[2] = &dt_d;

    res[0] = estimator->q;

    CASADI_FUNC_CALL(attitude_propagate)
  }

  {
    CASADI_FUNC_ARGS(attitude_estimator)

    double accel_b[3] = {(double)accel->x, (double)accel->y,
                         (double)accel->z};
    /* Safe non-zero placeholder when mag isn't valid yet - mag_gain=0 below
     * makes its actual content irrelevant, this just avoids a 0-vector
     * divide in the model's mag_earth normalization. */
    double mag_b[3] = {1.0, 0.0, 0.0};
    double mag_gain = 0.0;

    if (mag_valid && mag != NULL) {
      mag_b[0] = (double)mag->x;
      mag_b[1] = (double)mag->y;
      mag_b[2] = (double)mag->z;
      mag_gain = RDD2_ATTITUDE_MAG_GAIN;
    }

    args[0] = estimator->q;
    args[1] = mag_b;
    args[2] = &RDD2_MAG_DECL_RAD;
    args[3] = omega_b;
    args[4] = accel_b;
    args[5] = &RDD2_ATTITUDE_ACCEL_GAIN;
    args[6] = &mag_gain;
    args[7] = &dt_d;
    args[8] = estimator->p_att;

    res[0] = estimator->q;
    res[1] = estimator->p_att;

    CASADI_FUNC_CALL(attitude_estimator)
  }
}

void rdd2_attitude_estimator_get_attitude(
    const struct rdd2_attitude_estimator *estimator,
    rdd2_attitude_euler_t *attitude) {
  if (estimator == NULL || attitude == NULL) {
    return;
  }

  quat_to_euler(estimator->q, &attitude->roll, &attitude->pitch,
               &attitude->yaw);
}
