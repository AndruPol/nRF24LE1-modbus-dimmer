/*
 * dimmer.c
 *
 *  Created on: 15.04.2017
 *      Author: andru
 */

#define IFPPIN		GPIO_PIN_ID_P0_6		// P0.6 - GPINT1
#define PWMPIN		GPIO_PIN_ID_P0_2		// P0.2 - PWM output connected to T0 & T1

#define PULSE190US		0xFFFE		// ~190uS high pulse

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "gpio.h"
#include "pwm.h"
#include "interrupt.h"

#include "dimmer.h"
#include "main.h"

#if EN_CH0
#define CH0OUTPIN	GPIO_PIN_ID_P1_5		// P1.5 - dimmer 0 output
#define T0PIN		GPIO_PIN_ID_P0_7		// P0.7 - T0
#include "timer0.h"
static volatile uint16_t ch0_delay;
static volatile uint8_t ch0_on = 0, ch0_pulse;
#endif

#if EN_CH1
#define CH1OUTPIN	GPIO_PIN_ID_P1_6		// P1.6 - dimmer 1 output
#define T1PIN		GPIO_PIN_ID_P1_0		// P1.0 - T1
#include "timer1.h"
static volatile uint16_t ch1_delay;
static volatile uint8_t ch1_on = 0, ch1_pulse;
#endif

#if EN_CH0
// timer0 overflow isr
interrupt_isr_t0() {
	timer0_stop();
	if (! ch0_on) return;
	if (! ch0_pulse) {
		gpio_pin_val_set(CH0OUTPIN);
		ch0_pulse = 1;
		timer0_set_t0_val(PULSE190US);
		timer0_run();
	} else {
		gpio_pin_val_clear(CH0OUTPIN);
		ch0_pulse = 0;
	}
}
#endif

#if EN_CH1
// timer1 overflow isr
interrupt_isr_t1() {
	timer1_stop();
	if (! ch1_on) return;
	if (! ch1_pulse) {
		gpio_pin_val_set(CH1OUTPIN);
		ch1_pulse = 1;
		timer1_set_t1_val(PULSE190US);
		timer1_run();
	} else {
		gpio_pin_val_clear(CH1OUTPIN);
		ch1_pulse = 0;
	}
}
#endif

// zero cross detector isr
interrupt_isr_ifp() {
#if EN_CH0
	gpio_pin_val_clear(CH0OUTPIN);
	timer0_stop();
	if (ch0_on) {
		timer0_set_t0_val(ch0_delay);
		timer0_run();
	}
#endif
#if EN_CH1
	gpio_pin_val_clear(CH1OUTPIN);
	timer1_stop();
	if (ch1_on) {
		timer1_set_t1_val(ch1_delay);
		timer1_run();
	}
#endif
}

// init dimmer
void  dimmer_init(void) {
	// GPINT1 pin configure
	gpio_pin_configure(IFPPIN,
			GPIO_PIN_CONFIG_OPTION_DIR_INPUT
			| GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS
	);

	interrupt_configure_ifp(
			INTERRUPT_IFP_INPUT_GPINT1,
			INTERRUPT_IFP_CONFIG_OPTION_ENABLE
			| INTERRUPT_IFP_CONFIG_OPTION_TYPE_FALLING_EDGE
	);

	interrupt_set_priority(
			INTERRUPT_PRIORITY_GROUP_IFP_RFRDY,
			INTERRUPT_PRIORITY_LEVEL_2
	);

	// 10457.5 Hz
	pwm_configure(
			PWM_CONFIG_OPTION_PRESCALER_VAL_5
			| PWM_CONFIG_OPTION_WIDTH_8_BITS
	);

	// ~50uS high pulse if 134
	pwm_start(
			PWM_CHANNEL_0,
			128
	);

#if EN_CH0
	// T0 pin
	gpio_pin_configure(T0PIN,
			GPIO_PIN_CONFIG_OPTION_DIR_INPUT
			| GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS
	);

	// dimmer output pin
	gpio_pin_configure(CH0OUTPIN,
			GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT
			| GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_CLEAR
			| GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH
	);

	timer0_configure(
			TIMER0_CONFIG_OPTION_MODE_1_16_BIT_CTR_TMR
			| TIMER0_CONFIG_OPTION_FUNCTION_COUNT_EVENTS_ON_T0
			| TIMER0_CONFIG_OPTION_GATE_ALWAYS_RUN_TIMER,
			0
	);

	ch0_pulse = 0;
	ch0_on = 0;

	interrupt_set_priority(
			INTERRUPT_PRIORITY_GROUP_TF0_RFIRQ,
			INTERRUPT_PRIORITY_LEVEL_1
	);

	interrupt_control_t0_enable();
#endif

#if EN_CH1
	// T1 pin
	gpio_pin_configure(T1PIN,
			GPIO_PIN_CONFIG_OPTION_DIR_INPUT
			| GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS
	);

	// dimmer output pin
	gpio_pin_configure(CH1OUTPIN,
			GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT
			| GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_CLEAR
			| GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH
	);

	timer1_configure(
			TIMER1_CONFIG_OPTION_MODE_1_16_BIT_CTR_TMR
			| TIMER1_CONFIG_OPTION_FUNCTION_COUNT_EVENTS_ON_T1
			| TIMER1_CONFIG_OPTION_GATE_ALWAYS_RUN_TIMER,
			0
	);

	ch1_pulse = 0;
	ch1_on = 0;

	interrupt_set_priority(
			INTERRUPT_PRIORITY_GROUP_TF1_WUOPIRQ,
			INTERRUPT_PRIORITY_LEVEL_1
	);

	interrupt_control_t1_enable();
#endif

	interrupt_control_ifp_enable();
}

// start dimmer with percent value in range 20-100%
uint8_t dimmer_run(channel_t channel, uint8_t percent) {
	if (percent < DIMMERMIN || percent > DIMMERMAX) return 0;

#if EN_CH0
	if (channel == CH0) {
		if (ch0_on) ch0_on = 0;
		ch0_delay = (uint16_t) 0xFF00 | (254 + (percent - 100));
		ch0_pulse = 0;
		ch0_on = 1;
		return 1;
	}
#endif
#if EN_CH1
	if (channel == CH1) {
		if (ch1_on) ch1_on = 0;
		ch1_delay = (uint16_t) 0xFF00 | (254 + (percent - 100));
		ch1_pulse = 0;
		ch1_on = 1;
		return 1;
	}
#endif

	return 0;
}

// stop dimmer
void dimmer_stop(channel_t channel) {
#if EN_CH0
	if (channel == CH0) {
		ch0_pulse = 0;
		ch0_on = 0;
	}
#endif
#if EN_CH1
	if (channel == CH1) {
		ch1_pulse = 0;
		ch1_on = 0;
	}
#endif
}

uint8_t dimmer_state(channel_t channel) {
#if EN_CH0
	if (channel == CH0) return ch0_on;
#endif
#if EN_CH1
	if (channel == CH1) return ch1_on;
#endif
	return 0;
}
