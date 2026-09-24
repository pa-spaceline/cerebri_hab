/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Note/tone definitions, ported from cerebri/drivers/actuate/sound's
 * actuator_sound.h. Only the tones actually used by cerebri_hab's simpler
 * status model (no fuel/safety-switch/reject concepts here) are kept -
 * startup, armed/disarmed, and the two flight modes this app has.
 */
#ifndef RDD2_BUZZER_TONES_H_
#define RDD2_BUZZER_TONES_H_

#define thrirtysecond 19
#define sixteenth     38
#define eigth         75
#define quarter       150
#define half          300
#define whole         600

#define B4  494
#define C5  523
#define D5  587
#define Eb5 622
#define Gb5 740
#define A5  880
#define A4  440
#define B5  988

#define REST 1

struct tones_t {
  int note;
  int duration;
};

static const struct tones_t rdd2_buzzer_startup_tone[] = {
    {.note = B4, .duration = eigth},    {.note = REST, .duration = thrirtysecond},
    {.note = B4, .duration = eigth},    {.note = REST, .duration = thrirtysecond},
    {.note = B4, .duration = eigth},    {.note = REST, .duration = thrirtysecond},
    {.note = D5, .duration = quarter},  {.note = REST, .duration = thrirtysecond},
    {.note = B4, .duration = quarter},  {.note = REST, .duration = thrirtysecond},
    {.note = D5, .duration = eigth},    {.note = REST, .duration = thrirtysecond},
    {.note = D5, .duration = eigth},    {.note = REST, .duration = thrirtysecond},
    {.note = D5, .duration = eigth},    {.note = REST, .duration = thrirtysecond},
    {.note = Gb5, .duration = quarter}, {.note = REST, .duration = thrirtysecond},
    {.note = D5, .duration = quarter},  {.note = REST, .duration = thrirtysecond},
    {.note = Gb5, .duration = eigth},   {.note = REST, .duration = thrirtysecond},
    {.note = Gb5, .duration = eigth},   {.note = REST, .duration = thrirtysecond},
    {.note = Gb5, .duration = eigth},   {.note = REST, .duration = thrirtysecond},
    {.note = A5, .duration = quarter},  {.note = REST, .duration = thrirtysecond},
    {.note = A4, .duration = quarter},  {.note = REST, .duration = thrirtysecond},
    {.note = D5, .duration = eigth},    {.note = REST, .duration = thrirtysecond},
    {.note = D5, .duration = eigth},    {.note = REST, .duration = thrirtysecond},
    {.note = D5, .duration = eigth},    {.note = REST, .duration = thrirtysecond},
    {.note = Gb5, .duration = quarter},
};

static const struct tones_t rdd2_buzzer_armed_tone[] = {
    {.note = C5, .duration = eigth},
    {.note = REST, .duration = thrirtysecond},
    {.note = Eb5, .duration = eigth},
    {.note = REST, .duration = thrirtysecond},
    {.note = A5, .duration = quarter},
};

static const struct tones_t rdd2_buzzer_disarmed_tone[] = {
    {.note = A5, .duration = eigth},
    {.note = REST, .duration = thrirtysecond},
    {.note = Eb5, .duration = eigth},
    {.note = REST, .duration = thrirtysecond},
    {.note = C5, .duration = quarter},
};

/* Acro mode - Morse code "1" (short-long-long-long) */
static const struct tones_t rdd2_buzzer_acro_mode_tone[] = {
    {.note = B5, .duration = eigth},   {.note = REST, .duration = thrirtysecond},
    {.note = D5, .duration = quarter}, {.note = REST, .duration = thrirtysecond},
    {.note = D5, .duration = quarter}, {.note = REST, .duration = thrirtysecond},
    {.note = D5, .duration = quarter}, {.note = REST, .duration = thrirtysecond},
};

/* Auto-level mode - Morse code "2" (short-short-long-long) */
static const struct tones_t rdd2_buzzer_autolevel_mode_tone[] = {
    {.note = B5, .duration = eigth},   {.note = REST, .duration = thrirtysecond},
    {.note = B5, .duration = eigth},   {.note = REST, .duration = thrirtysecond},
    {.note = D5, .duration = quarter}, {.note = REST, .duration = thrirtysecond},
    {.note = D5, .duration = quarter}, {.note = REST, .duration = thrirtysecond},
};

#endif
