/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>

#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "buzzer.h"

LOG_MODULE_REGISTER(rdd2_buzzer, LOG_LEVEL_INF);

#define BUZZER_NODE DT_ALIAS(buzzer)

static const struct pwm_dt_spec g_buzzer = PWM_DT_SPEC_GET(BUZZER_NODE);

static const struct tones_t *g_tones;
static size_t g_tone_count;
static size_t g_index;
static int64_t g_note_end_ms;
static bool g_active;

static void apply_note(const struct tones_t *tone) {
  if (tone->note == REST) {
    (void)pwm_set_pulse_dt(&g_buzzer, 0);
  } else {
    (void)pwm_set_dt(&g_buzzer, PWM_HZ(tone->note), PWM_HZ(tone->note) / 2);
  }
  g_note_end_ms = k_uptime_get() + tone->duration;
}

void rdd2_buzzer_init(void) {
  g_tones = NULL;
  g_tone_count = 0;
  g_index = 0;
  g_active = false;

  if (!pwm_is_ready_dt(&g_buzzer)) {
    LOG_ERR("buzzer PWM device not ready");
    return;
  }
  (void)pwm_set_pulse_dt(&g_buzzer, 0);
}

void rdd2_buzzer_play(const struct tones_t *tones, size_t count) {
  if (tones == NULL || count == 0 || !pwm_is_ready_dt(&g_buzzer)) {
    return;
  }

  g_tones = tones;
  g_tone_count = count;
  g_index = 0;
  g_active = true;
  apply_note(&g_tones[0]);
}

void rdd2_buzzer_update(void) {
  if (!g_active) {
    return;
  }

  if (k_uptime_get() < g_note_end_ms) {
    return;
  }

  g_index++;
  if (g_index >= g_tone_count) {
    g_active = false;
    (void)pwm_set_pulse_dt(&g_buzzer, 0);
    return;
  }

  apply_note(&g_tones[g_index]);
}
