/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 CogniPilot Foundation */

/*
 * Status LED red/blue channels (Teensy pins 0/1, GPIO_AD_B0_02/03).
 *
 * These two LEDs are wired through the RT1062's XBARA1 crossbar into
 * Quad Timer 4 rather than a plain FlexPWM/FlexIO pin (matches PX4's
 * PWM_LED_RED/PWM_LED_BLUE in boards/nxp/tropic-community's
 * tropic_led_pwm.cpp), so there is no devicetree binding that can express
 * it - this file does the one-time XBAR/GPR6/QTMR4 setup and exposes
 * simple brightness-set functions. LED_G does not have this problem (it's
 * a plain FlexPWM2 pin) and is wired directly in tropic_community.dts.
 *
 * Every register value below is traced to the i.MX RT1060 Processor
 * Reference Manual, Rev. 3, 07/2021, not to PX4's NuttX-specific macros
 * (which use different constant shapes and were not re-derived from):
 *   - XBARA1 input 36 = QTIMER4_TIMER0, input 37 = QTIMER4_TIMER1
 *     (Chapter 4 "Interrupts, DMA Events, and XBAR Assignments",
 *     Table 4-5 "XBAR1 Input Assignments")
 *   - XBARA1 output 16/17 = IOMUX_XBAR_INOUT16/17, i.e. this board's
 *     LED_RED/LED_BLUE pins (same table)
 *   - XBARA1_SELn register layout: output 2n in the low byte, output
 *     2n+1 in the high byte of SELn (Chapter 61 "Inter-Peripheral
 *     Crossbar Switch A (XBARA)", section 61.5, e.g. 61.5.9 SEL8 has
 *     SEL16/SEL17)
 *   - IOMUXC_GPR_GPR6 bits 28/29 select XBAR_INOUT16/17 direction
 *     (1 = output) (Chapter 11 "IOMUX Controller", section 11.3.7)
 *   - Quad Timer CTRL/SCTRL/CSCTRL field encodings (Chapter 54 "Quad
 *     Timer (TMR)", section 54.9.1) set up "Variable-Frequency PWM
 *     Mode" (section 54.4.5.14): CM=001 (count rising edges of
 *     primary source), PCS=1101 (IP bus clock / 32), DIR=1 (count
 *     down, arbitrary choice matching PX4), OUTMODE=100 (toggle OFLAG,
 *     alternating COMP1/COMP2), SCTRL OEN=1 (drive OFLAG on the pin)
 *     + FORCE=1 (force the pin low once at init, while the counter is
 *     still disabled per the manual's note on CTRL[FORCE]), CSCTRL
 *     CL1=01 (reload COMP1 from CMPLD1 on every COMP1 match, so
 *     writing CMPLD1 changes brightness on the next cycle)
 *   - TMR4 base 0x401E_8000, per-channel register offsets from
 *     section 54.9.1.1's memory map (channel 1 = base + 0x20)
 *
 * LED is active-low/open-drain (same wiring convention as PX4's
 * GPIO_nLED_RED/BLUE), so a longer OFLAG-low phase (bigger CMPLD1) is
 * brighter - CMPLD1 defines the "off" (COMP1) side of the alternating
 * compare pair per section 54.4.2.
 *
 * These functions aren't hooked up to anything yet - nothing currently
 * calls them with arm/safety/mode semantics. mr_vmu_tropic's own default
 * cerebri config doesn't drive its onboard RGB LED with any arm/safety/
 * mode logic either (CEREBRI_RDD2_LIGHTING and CEREBRI_ACTUATE_LED_ARRAY
 * are both disabled there too), so this matches it rather than falling
 * short of it.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <fsl_iomuxc.h>
#include <fsl_clock.h>
#include <soc.h>

#define XBARA1_SEL_BASE 0x403BC000u
#define IOMUXC_GPR_GPR6 (*(volatile uint32_t *)0x400AC018u)

#define XBAR_IN_QTIMER4_TIMER0      36
#define XBAR_IN_QTIMER4_TIMER1      37
#define XBAR_OUT_IOMUX_XBAR_INOUT16 16
#define XBAR_OUT_IOMUX_XBAR_INOUT17 17

static void xbar_connect(uint32_t output, uint32_t input)
{
	volatile uint16_t *sel =
		(volatile uint16_t *)(uintptr_t)(XBARA1_SEL_BASE + (output / 2) * 2);

	if (output % 2 == 0) {
		*sel = (uint16_t)((*sel & 0xFF00u) | (input & 0x7Fu));
	} else {
		*sel = (uint16_t)((*sel & 0x00FFu) | ((input & 0x7Fu) << 8));
	}
}

/* TMR4 channel 0 = red (XBAR_OUT16), channel 1 = blue (XBAR_OUT17) */
#define TMR4_CH0_COMP1  (*(volatile uint16_t *)0x401E8000u)
#define TMR4_CH0_COMP2  (*(volatile uint16_t *)0x401E8002u)
#define TMR4_CH0_CNTR   (*(volatile uint16_t *)0x401E800Au)
#define TMR4_CH0_CTRL   (*(volatile uint16_t *)0x401E800Cu)
#define TMR4_CH0_SCTRL  (*(volatile uint16_t *)0x401E800Eu)
#define TMR4_CH0_CMPLD1 (*(volatile uint16_t *)0x401E8010u)
#define TMR4_CH0_CMPLD2 (*(volatile uint16_t *)0x401E8012u)
#define TMR4_CH0_CSCTRL (*(volatile uint16_t *)0x401E8014u)

#define TMR4_CH1_COMP1  (*(volatile uint16_t *)0x401E8020u)
#define TMR4_CH1_COMP2  (*(volatile uint16_t *)0x401E8022u)
#define TMR4_CH1_CNTR   (*(volatile uint16_t *)0x401E802Au)
#define TMR4_CH1_CTRL   (*(volatile uint16_t *)0x401E802Cu)
#define TMR4_CH1_SCTRL  (*(volatile uint16_t *)0x401E802Eu)
#define TMR4_CH1_CMPLD1 (*(volatile uint16_t *)0x401E8030u)
#define TMR4_CH1_CMPLD2 (*(volatile uint16_t *)0x401E8032u)
#define TMR4_CH1_CSCTRL (*(volatile uint16_t *)0x401E8034u)

#define QTMR_LED_CTRL   ((1u << 13) | (0xDu << 9) | (1u << 4) | (0x4u << 0))
#define QTMR_LED_SCTRL  ((1u << 0) | (1u << 2))
#define QTMR_LED_CSCTRL (0x1u)

static void qtmr4_channel_init(volatile uint16_t *ctrl, volatile uint16_t *sctrl,
				volatile uint16_t *csctrl, volatile uint16_t *comp1,
				volatile uint16_t *comp2, volatile uint16_t *cmpld1,
				volatile uint16_t *cmpld2, volatile uint16_t *cntr)
{
	*ctrl = 0;
	*cntr = 0;
	*sctrl = QTMR_LED_SCTRL;
	*csctrl = QTMR_LED_CSCTRL;
	*comp1 = 1;
	*cmpld1 = 1;
	*comp2 = 1;
	*cmpld2 = 1;
	*ctrl = QTMR_LED_CTRL;
}

void board_early_init_hook(void)
{
	CLOCK_EnableClock(kCLOCK_Xbar1);
	CLOCK_EnableClock(kCLOCK_Timer4);

	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_02_XBAR1_INOUT16, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_XBAR1_INOUT17, 0);

	IOMUXC_GPR_GPR6 |= (1u << 28) | (1u << 29);

	xbar_connect(XBAR_OUT_IOMUX_XBAR_INOUT16, XBAR_IN_QTIMER4_TIMER0);
	xbar_connect(XBAR_OUT_IOMUX_XBAR_INOUT17, XBAR_IN_QTIMER4_TIMER1);

	qtmr4_channel_init(&TMR4_CH0_CTRL, &TMR4_CH0_SCTRL, &TMR4_CH0_CSCTRL, &TMR4_CH0_COMP1,
			   &TMR4_CH0_COMP2, &TMR4_CH0_CMPLD1, &TMR4_CH0_CMPLD2, &TMR4_CH0_CNTR);
	qtmr4_channel_init(&TMR4_CH1_CTRL, &TMR4_CH1_SCTRL, &TMR4_CH1_CSCTRL, &TMR4_CH1_COMP1,
			   &TMR4_CH1_COMP2, &TMR4_CH1_CMPLD1, &TMR4_CH1_CMPLD2, &TMR4_CH1_CNTR);
}

/* value: 0 (off) .. 255 (max brightness) */
void tropic_community_led_red_set(uint8_t value)
{
	TMR4_CH0_CMPLD1 = (uint16_t)((uint16_t)value * 256U);
}

void tropic_community_led_blue_set(uint8_t value)
{
	TMR4_CH1_CMPLD1 = (uint16_t)((uint16_t)value * 256U);
}
