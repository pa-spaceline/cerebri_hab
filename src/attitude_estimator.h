#ifndef RDD2_ATTITUDE_ESTIMATOR_H_
#define RDD2_ATTITUDE_ESTIMATOR_H_

#include <stdbool.h>

#include "synapse_messages.h"

struct rdd2_attitude_estimator {
  double q[4];      /* [w, x, y, z] */
  double p_att[6];  /* attitude covariance - pass-through, not actively used
                      * by cyecca's attitude_estimator (kept only for
                      * interface compatibility, same as cerebri/app/hab). */
};

void rdd2_attitude_estimator_init(struct rdd2_attitude_estimator *estimator);

/* Full attitude re-init from accel (roll/pitch) + mag (yaw) - used while
 * disarmed to continuously track current orientation before arming.
 * If mag_valid is false, yaw is left at its previous value (no magnetometer
 * data available yet) and only roll/pitch are refreshed from accel. */
void rdd2_attitude_estimator_reset(struct rdd2_attitude_estimator *estimator,
                                   const rdd2_vec3f_t *accel,
                                   const rdd2_vec3f_t *mag, bool mag_valid);

/* Gyro-propagate then correct with accel (roll/pitch) + mag (yaw). If
 * mag_valid is false, magnetometer correction is skipped for this step
 * (gain forced to zero) rather than feeding in stale/zero data. */
void rdd2_attitude_estimator_predict(struct rdd2_attitude_estimator *estimator,
                                     const rdd2_vec3f_t *gyro,
                                     const rdd2_vec3f_t *accel,
                                     const rdd2_vec3f_t *mag, bool mag_valid,
                                     float dt);

void rdd2_attitude_estimator_get_attitude(
    const struct rdd2_attitude_estimator *estimator,
    rdd2_attitude_euler_t *attitude);

#endif
