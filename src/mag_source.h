#ifndef RDD2_MAG_SOURCE_H_
#define RDD2_MAG_SOURCE_H_

#include "rdd2_control_types.h"

#include <stdbool.h>

/* Copies the newest magnetometer sample, in the sensor's axes and gauss.
 * Returns false when there is no mag0, no sample yet, or the newest one is
 * stale, so the estimator can drop its mag correction for that update. */
bool rdd2_mag_latest(rdd2_vec3f_t *mag);

#endif
