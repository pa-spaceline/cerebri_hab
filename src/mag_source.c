/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Magnetometer reader, off the control loop.
 *
 * A mag fetch is several blocking I2C transactions (the IST8310's status,
 * data and re-trigger), about 1.6 ms at 100 kHz. Done inside the 1600 Hz loop
 * it cost two control ticks per read. This thread does the fetch at its own
 * rate and below the loop's priority; the loop only copies the newest sample.
 */

#include "mag_source.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/spinlock.h>

#include <math.h>

#define MAG_NODE DT_ALIAS(mag0)

#if DT_NODE_HAS_STATUS_OKAY(MAG_NODE)

/* The IST8310 converts a single-shot sample in about 6 ms, so a 10 ms period
 * always finds one ready. */
#define RDD2_MAG_PERIOD_MS     10
/* Older than a few periods means the reader is failing, not just late. */
#define RDD2_MAG_STALE_MS      50
#define RDD2_MAG_THREAD_PRIO   6
#define RDD2_MAG_STACK_SIZE    1024

static const struct device *const g_mag_dev = DEVICE_DT_GET(MAG_NODE);

static struct k_spinlock g_mag_lock;
static rdd2_vec3f_t g_mag;
static int64_t g_mag_stamp_ms;
static bool g_mag_valid;
static uint32_t g_mag_reads;
static uint32_t g_mag_failed;

static bool mag_fetch(rdd2_vec3f_t *mag)
{
	struct sensor_value values[3];

	if (sensor_sample_fetch(g_mag_dev) < 0) {
		return false;
	}
	if (sensor_channel_get(g_mag_dev, SENSOR_CHAN_MAGN_XYZ, values) < 0) {
		return false;
	}

	mag->x = sensor_value_to_float(&values[0]);
	mag->y = sensor_value_to_float(&values[1]);
	mag->z = sensor_value_to_float(&values[2]);
	return true;
}

static void mag_thread(void *arg0, void *arg1, void *arg2)
{
	ARG_UNUSED(arg0);
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);

	if (!device_is_ready(g_mag_dev)) {
		return;
	}

	while (true) {
		rdd2_vec3f_t mag;

		if (mag_fetch(&mag)) {
			k_spinlock_key_t key = k_spin_lock(&g_mag_lock);

			g_mag = mag;
			g_mag_stamp_ms = k_uptime_get();
			g_mag_valid = true;
			g_mag_reads++;
			k_spin_unlock(&g_mag_lock, key);
		} else {
			g_mag_failed++;
		}
		k_sleep(K_MSEC(RDD2_MAG_PERIOD_MS));
	}
}

K_THREAD_DEFINE(g_rdd2_mag_tid, RDD2_MAG_STACK_SIZE, mag_thread, NULL, NULL, NULL,
		RDD2_MAG_THREAD_PRIO, 0, 0);

bool rdd2_mag_latest(rdd2_vec3f_t *mag)
{
	k_spinlock_key_t key = k_spin_lock(&g_mag_lock);
	bool ok = g_mag_valid && (k_uptime_get() - g_mag_stamp_ms) <= RDD2_MAG_STALE_MS;

	if (ok) {
		*mag = g_mag;
	}
	k_spin_unlock(&g_mag_lock, key);
	return ok;
}

/* Reports the reader's newest sample rather than fetching: the sensor is
 * single-shot, so a second reader would steal the conversion from the thread
 * and fail itself whenever it lands inside one. */
static int cmd_mag_status(const struct shell *sh, size_t argc, char **argv)
{
	rdd2_vec3f_t mag;
	int64_t stamp_ms;
	uint32_t reads;
	uint32_t failed;
	bool valid;
	k_spinlock_key_t key;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	key = k_spin_lock(&g_mag_lock);
	mag = g_mag;
	stamp_ms = g_mag_stamp_ms;
	valid = g_mag_valid;
	reads = g_mag_reads;
	failed = g_mag_failed;
	k_spin_unlock(&g_mag_lock, key);

	shell_print(sh, "device=%s reads=%u failed=%u", g_mag_dev->name, (unsigned int)reads,
		    (unsigned int)failed);
	if (!valid) {
		shell_warn(sh, "no sample yet");
		return 0;
	}

	shell_print(sh, "last sample %lld ms ago%s", k_uptime_get() - stamp_ms,
		    rdd2_mag_latest(&mag) ? "" : " (stale, estimator ignores it)");
	shell_print(sh, "xyz=(%.4f, %.4f, %.4f) G  |B|=%.4f G", (double)mag.x, (double)mag.y,
		    (double)mag.z,
		    (double)sqrtf(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z));
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_mag,
	SHELL_CMD(status, NULL, "newest sample from the mag reader thread", cmd_mag_status),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(mag, &sub_mag, "magnetometer reader diagnostics", NULL);

#else

bool rdd2_mag_latest(rdd2_vec3f_t *mag)
{
	ARG_UNUSED(mag);
	return false;
}

#endif
