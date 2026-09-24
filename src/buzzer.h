#ifndef RDD2_BUZZER_H_
#define RDD2_BUZZER_H_

#include <stddef.h>

#include "buzzer_tones.h"

void rdd2_buzzer_init(void);

/* Starts playing a tone sequence immediately, preempting whatever is
 * currently playing (status tones are rare, don't-care-about-queueing
 * events - the latest one wins). Non-blocking: schedules the first note
 * and returns; rdd2_buzzer_update() steps through the rest. */
void rdd2_buzzer_play(const struct tones_t *tones, size_t count);

/* Advances the current tone sequence if its current note's duration has
 * elapsed. Cheap (a single timestamp comparison) when nothing needs to
 * change - safe to call from a tight real-time loop every iteration,
 * unlike cerebri's original driver which blocked with k_msleep() per note
 * from its own dedicated thread. */
void rdd2_buzzer_update(void);

#endif
