/*
 *  Created on: 5.05.2017
 *      Author: andru
 *
 *      nRF24LE1 modbus dimmer
 *
 *		based on great nRF24LE1 SDK https://github.com/DeanCording/nRF24LE1_SDK
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "gpio.h"
#include "delay.h"
#include "memory.h"
#include "interrupt.h"

#include "main.h"
#include "dimmer.h"
#include "crc8.h"
#include "mb.h"

#define FIRMWARE		10			// FW VER 0.10
#define MAGIC			0xAE		// magic word

#if EN_LED
#define LEDPIN		GPIO_PIN_ID_P1_4	// P1.4 - LED
#define LEDDELAY	250					// 250mS
#endif

/* === BOARD PINOUT ===
 * >>> LED
 * #define LEDPIN				GPIO_PIN_ID_P1_4		// P1.4 - LED
 *
 * >>> SW0, SW1
 * #define SW0PIN				GPIO_PIN_ID_P0_0		// P0.0 - switch 0 input
 * #define SW1PIN				GPIO_PIN_ID_P0_1		// P0.1 - switch 1 input
 *
 * >>> MODBUS
 * #define BOARD_SERIAL_TX		GPIO_PIN_ID_P0_3		// P0.3 - UART TX
 * #define BOARD_SERIAL_RX		GPIO_PIN_ID_P0_4		// P0.4 - UART RX
 * #define BOARD_MAX485_TX_EN	GPIO_PIN_ID_P1_2		// P1.2 - MAX485 TX_EN
 *
 * >>> DIMMER
 * #define IFPPIN				GPIO_PIN_ID_P0_6		// P0.6 - IFP GPINT1 zero cross detector
 * #define PWMPIN				GPIO_PIN_ID_P0_2		// P0.2 - PWM output connected to T0, T1
 *
 * #define CH0OUTPIN			GPIO_PIN_ID_P1_5		// P1.5 - dimmer 0 output
 * #define T0PIN				GPIO_PIN_ID_P0_7		// P0.7 - T0 pin
 *
 * #define CH1OUTPIN			GPIO_PIN_ID_P1_6		// P1.6 - dimmer 1 output
 * #define T1PIN				GPIO_PIN_ID_P1_0		// P1.0 - T1 pin
 *
*/

#if EN_SW0
#define SW0PIN	GPIO_PIN_ID_P0_0		// P0.0 - switch 0 input
#endif
#if EN_SW1
#define SW1PIN	GPIO_PIN_ID_P0_1		// P0.1 - switch 1 input
#endif

//#define ENVM_START_ADDRESS	MEMORY_FLASH_NV_EXT_END_START_ADDRESS
#define ENVM_START_ADDRESS	(0xFB00)

#if EN_WDG
#include "watchdog.h"
#define WDGTIMEOUT	30	// watchdog timeout, S
#endif

CONFIG_T config;
static uint8_t percent[DIMMERS];

// halt
void halt(void) {
	while (1) {
#if EN_LED
		gpio_pin_val_complement(LEDPIN);
		delay_ms(LEDDELAY);
#endif
	}
}

// set watchdog timer
void setwatchdog(void) {
#if EN_WDG
	watchdog_set_wdsv_count(watchdog_calc_timeout_from_sec(WDGTIMEOUT));
#endif
}

// T2 interrupt in porttimer.c
interrupt_isr_t2();

// uart interrupt in portserial.c
interrupt_isr_uart();

#if EN_CH0
// T0 interrupt in dimmer.c
interrupt_isr_t0();
#endif

#if EN_CH1
// T1 interrupt in dimmer.c
interrupt_isr_t1();
#endif

// IFP interrupt in dimmer.c
interrupt_isr_ifp();

// default config
static void defconfig(void) {
	config.magic = MAGIC;
	config.addr = 1;
	config.bitrate = MB_BR_9600;
	config.parity = MB_PARITY_NONE;
	config.chcfg[CH0].enable = 1;
	config.chcfg[CH0].percent = 100;
	config.chcfg[CH1].enable = 1;
	config.chcfg[CH1].percent = 100;
}

// read config from eNVM with start address
static uint8_t read_config(uint16_t addr) {
	uint8_t i, pcon_temp;
	uint8_t buf[sizeof(CONFIG_T) + 1];
	CONFIG_T *p;

	interrupt_save_global_flag(PSW_SB_F0);
	interrupt_control_global_disable();

	pcon_temp = PCON;
	memory_movx_accesses_data_memory();

	for (i = 0; i < sizeof(CONFIG_T) + 1; i++) {
		buf[i] = *((__xdata uint8_t *) (addr + i));
	}

	PCON = pcon_temp;
	interrupt_restore_global_flag(PSW_SB_F0);

	p = (CONFIG_T *) &buf;
	if ( p->magic == MAGIC && CRC8(buf, sizeof(CONFIG_T)) == buf[sizeof(CONFIG_T)] ) {
		memcpy((uint8_t *) &config, buf, sizeof(CONFIG_T));
		return 1;
	}
	return 0;
}

// write config to eNVM with start address
static uint8_t write_config(uint16_t addr) {
	uint8_t i, ret, temp;
	uint8_t buf[sizeof(CONFIG_T) + 1];

	memcpy(buf, (uint8_t *) &config, sizeof(CONFIG_T));
	buf[sizeof(CONFIG_T)] = CRC8(buf, sizeof(CONFIG_T));

	if (memory_flash_get_page_num_from_address(addr, &temp) == MEMORY_FLASH_OK) {
		if (memory_flash_erase_page(temp) != MEMORY_FLASH_OK) return 0;
	} else return 0;

	ret = 1;
	interrupt_save_global_flag(PSW_SB_F0);
	interrupt_control_global_disable();

	temp = PCON;
	memory_movx_accesses_data_memory();

	for (i = 0; i < sizeof(CONFIG_T) + 1; i++) {
		memory_flash_enable_write_access();
		*((__xdata uint8_t *) (addr + i)) = buf[i];
		memory_flash_wait_for_write_complete();
		memory_flash_disable_write_access();
		if ( buf[i] != *((__xdata uint8_t *) (addr + i)) ) {
			ret = 0;
			break;
		}
	}

	PCON = temp;
	interrupt_restore_global_flag(PSW_SB_F0);

	return ret;
}

static void writeOk(void) {
	setDiscreteBit(ADDR_STATUS, 0);
	writeInput(ADDR_STATUS, CMD_OK);
	setCoilBit(ADDR_STATUS, 1);
}

static void writeError(error_t err) {
	setDiscreteBit(ADDR_STATUS, 1);
	writeInput(ADDR_STATUS, err);
	setCoilBit(ADDR_STATUS, 1);
}

// save to config modbus addr (lo byte), bitrate & parity (hi byte), default holding[1] = 0x0201
// holding[0] need to be MAGIC = 0xAE
static void setModbusCmd(void) {
	uint8_t addr, bitrate, parity;

   	if (! ( ucSCoilBuf[0] & (1 << ADDR_CONFIG) ) ) return;

	setCoilBit(ADDR_CONFIG, 0);

#if EN_CH0
	if (dimmer_state(CH0)) {
		writeError(CMD_DIMMER);
		return;
	}
#endif
#if EN_CH1
	if (dimmer_state(CH1)) {
		writeError(CMD_DIMMER);
		return;
	}
#endif

	if (usSRegHoldBuf[ADDR_STATUS] != MAGIC) goto error;

	addr = (uint8_t) usSRegHoldBuf[ADDR_CONFIG] & 0x00FF;
	if (addr == 0 || addr > 247) goto error;

	bitrate = (uint8_t) (usSRegHoldBuf[ADDR_CONFIG] >> 8) & 0x0F;
	if (bitrate > MB_BR_38400) goto error;

	parity = (uint8_t) (usSRegHoldBuf[ADDR_CONFIG] >> 12);
	if (parity > MB_PARITY_EVEN) goto error;

	config.addr = addr;
	config.bitrate = bitrate;
	config.parity = parity;

#if EN_WDG
	watchdog_set_wdsv_count(watchdog_calc_timeout_from_sec(WDGTIMEOUT));
#endif

 	if (! write_config(ENVM_START_ADDRESS)) {
 		writeError(CFG_WRITE);
 		return;
 	}

#if 0
 	// cause watchdog reset
 	watchdog_set_wdsv_count(1);
 	delay_ms(10);
#endif

 	writeOk();
	return;

error:
	writeError(CMD_PARAM);
}

// set current dimmer state & percent
static void setDimmerStateCmd(channel_t channel, uint8_t regaddr) {
	if (! config.chcfg[channel].enable) return;
   	if ( ucSCoilBuf[0] & (1 << regaddr) ) {
   		if ( (usSRegHoldBuf[regaddr] >= DIMMERMIN) && (usSRegHoldBuf[regaddr] <= DIMMERMAX)) {
   			percent[channel] = (uint8_t) usSRegHoldBuf[regaddr];
   		}
		dimmer_run(channel, percent[channel]);
    } else {
    	dimmer_stop(channel);
    }
	writeInput(regaddr, percent[channel]);
	setCoilBit(regaddr, dimmer_state(channel));

	writeOk();
}

// set current dimmer percent
static void setDimmerPercentCmd(channel_t channel, uint8_t regaddr) {
	if ( (usSRegHoldBuf[regaddr] < DIMMERMIN) || (usSRegHoldBuf[regaddr] > DIMMERMAX)) {
		writeError(CMD_PARAM);
		return;
	}

	percent[channel] = (uint8_t) usSRegHoldBuf[regaddr];
   	if (config.chcfg[channel].enable && dimmer_state(channel)) {
   	   	dimmer_stop(channel);
   		dimmer_run(channel, percent[channel]);
   	}
	writeInput(regaddr, percent[channel]);

	writeOk();
}

// set & save to eNVM dimmer percent
static void setDimmerConfigCmd(channel_t channel, uint8_t regaddr) {
	uint8_t ret, state;
   	if (! ( ucSCoilBuf[0] & (1 << regaddr) ) ) return;

	setCoilBit(regaddr, 0);

	if (config.chcfg[channel].enable == usSRegHoldBuf[regaddr - 1] && config.chcfg[channel].percent == usSRegHoldBuf[regaddr]) return;

	if ( (usSRegHoldBuf[regaddr - 1] < 0) || (usSRegHoldBuf[regaddr - 1] > 1)) goto error;
	if ( (usSRegHoldBuf[regaddr] < DIMMERMIN) || (usSRegHoldBuf[regaddr] > DIMMERMAX)) goto error;

	config.chcfg[channel].enable = (uint8_t) usSRegHoldBuf[regaddr - 1];
	config.chcfg[channel].percent = (uint8_t) usSRegHoldBuf[regaddr];
	percent[channel] = config.chcfg[channel].percent;

#if EN_WDG
	watchdog_set_wdsv_count(watchdog_calc_timeout_from_sec(WDGTIMEOUT));
#endif

	// save current state
	state = dimmer_state(channel);
	dimmer_stop(channel);

	ret = write_config(ENVM_START_ADDRESS);

	if (config.chcfg[channel].enable && state)
		dimmer_run(channel, percent[channel]);

	if (! ret) {
		writeError(CFG_WRITE);
		return;
	}

	writeOk();
	return;

error:
	writeError(CMD_PARAM);
}


// main
void main(void) {

#if EN_SW0 || EN_SW1
	// variable definition
	uint8_t ex_sw0, ex_sw1;
	uint8_t cmd;
#endif

	// program code
#if EN_LED
	gpio_pin_configure(LEDPIN,
		GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT
		| GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_CLEAR
		| GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH
		);
#endif

 	ucSDiscInBuf[0] = 0;
 	ucSCoilBuf[0] = 0;
 	memset(usSRegInBuf, 0, 2 * S_REG_INPUT_NREGS);
 	memset(usSRegHoldBuf, 0, 2 * S_REG_HOLDING_NREGS);

	if (! read_config(ENVM_START_ADDRESS) ) {
	 	setDiscreteBit(ADDR_STATUS, 1);
	 	writeInput(ADDR_STATUS, CFG_READ);
		defconfig();
		if (! write_config(ENVM_START_ADDRESS) ) {
			// config write error stop work
			halt();
		}
	}

	interrupt_control_global_enable();
 	initModbus (config.addr, config.bitrate, config.parity);

 	writeHolding(ADDR_STATUS, FIRMWARE);
 	setCoilBit(ADDR_STATUS, 1);

 	dimmer_init();
#if EN_CH0
 	percent[CH0] = config.chcfg[CH0].percent;
#endif
#if EN_CH1
 	percent[CH1] = config.chcfg[CH1].percent;
#endif

#if EN_SW0
	// SW0PIN pin configure
	gpio_pin_configure(SW0PIN,
			GPIO_PIN_CONFIG_OPTION_DIR_INPUT
			| GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS
	);
	ex_sw0 = gpio_pin_val_read(SW0PIN);
	if (ex_sw0) {
#if EN_CH0
		if (config.chcfg[CH0].enable && percent[CH0] >= DIMMERMIN && percent[CH0] <= DIMMERMAX) {
			dimmer_run(CH0, percent[CH0]);
		}
#endif
	}
#endif

#if EN_SW1
	// SW1PIN pin configure
	gpio_pin_configure(SW1PIN,
			GPIO_PIN_CONFIG_OPTION_DIR_INPUT
			| GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS
	);
	ex_sw1 = gpio_pin_val_read(SW1PIN);
	if (ex_sw1) {
#if EN_CH1
		if (config.chcfg[CH1].enable && percent[CH1] >= DIMMERMIN && percent[CH1] <= DIMMERMAX) {
			dimmer_run(CH1, percent[CH1]);
		}
#endif
	}
#endif


#if EN_WDG
	watchdog_setup();
	watchdog_set_wdsv_count(watchdog_calc_timeout_from_sec(WDGTIMEOUT));

	// if reset by watchdog restore dimmer state
	if ( pwr_clk_mgmt_was_prev_reset_watchdog(pwr_clk_mgmt_get_reset_reason()) ) {
#if EN_CH0
		if (config.chcfg[CH0].enable && percent[CH0] >= DIMMERMIN && percent[CH0] <= DIMMERMAX) {
			dimmer_run(CH0, percent[CH0]);
		} else {
			dimmer_stop(CH0);
		}
#endif
#if EN_CH1
		if (config.chcfg[CH1].enable && percent[CH1] >= DIMMERMIN && percent[CH1] <= DIMMERMAX) {
			dimmer_run(CH1, percent[CH1]);
		} else {
			dimmer_stop(CH1);
		}
#endif
	}
#endif


	while(1) {

#if EN_LED
		if ( usSRegInBuf[1]++ == 0 )
			gpio_pin_val_complement(LEDPIN);
#endif

#if EN_CH0
		writeInput(ADDR_DIMMER0_STATE, percent[CH0]);
		setCoilBit(ADDR_DIMMER0_STATE, dimmer_state(CH0));
#endif
#if EN_CH1
		writeInput(ADDR_DIMMER1_STATE, percent[CH1]);
		setCoilBit(ADDR_DIMMER1_STATE, dimmer_state(CH1));
#endif

#if EN_SW0
		setDiscreteBit(ADDR_DIMMER0_STATE, ex_sw0);
		cmd = gpio_pin_val_read(SW0PIN);
		if (cmd != ex_sw0) {
#if EN_CH0
			if (cmd && !dimmer_state(CH0) && percent[CH0] >= DIMMERMIN && percent[CH0] <= DIMMERMAX) {
				if (config.chcfg[CH0].enable)
					dimmer_run(CH0, percent[CH0]);
			};
			if (!cmd && dimmer_state(CH0)) {
				dimmer_stop(CH0);
			}
#endif
		}
		ex_sw0 = cmd;
#endif
#if EN_SW1
		setDiscreteBit(ADDR_DIMMER1_STATE, ex_sw1);
		cmd = gpio_pin_val_read(SW1PIN);
		if (cmd != ex_sw1) {
#if EN_CH1
			if (cmd && !dimmer_state(CH1) && percent[CH1] >= DIMMERMIN && percent[CH1] <= DIMMERMAX) {
				if (config.chcfg[CH1].enable)
					dimmer_run(CH1, percent[CH1]);
			};
			if (!cmd && dimmer_state(CH1)) {
				dimmer_stop(CH1);
			}
#endif
		}
		ex_sw1 = cmd;
#endif

		eMBPoll();

        // check coils changed
       	if ( (ucChangeCoilBuf[0] & (1 << ADDR_CONFIG)) ) {
       		setModbusCmd();
        }
#if EN_CH0
       	if ( (ucChangeCoilBuf[0] & (1 << ADDR_DIMMER0_STATE)) ) {
       		setDimmerStateCmd(CH0, ADDR_DIMMER0_STATE);
        }
       	if ( (ucChangeCoilBuf[0] & (1 << ADDR_DIMMER0_CONFIG)) ) {
       		setDimmerConfigCmd(CH0, ADDR_DIMMER0_CONFIG);
        }
       	// check holding changed
       	if ( (ucChangeCoilBuf[0] & (1 << ADDR_DIMMER0_CONFIG)) == 0
       			&& percent[CH0] != usSRegHoldBuf[ADDR_DIMMER0_STATE]
				&& usSRegHoldBuf[ADDR_DIMMER0_STATE] >= DIMMERMIN && usSRegHoldBuf[ADDR_DIMMER0_STATE] <= DIMMERMAX	 ) {
       		setDimmerPercentCmd(CH0, ADDR_DIMMER0_STATE);
       	}
#endif
#if EN_CH1
       	if ( (ucChangeCoilBuf[0] & (1 << ADDR_DIMMER1_STATE)) ) {
       		setDimmerStateCmd(CH1, ADDR_DIMMER1_STATE);
        }
       	if ( (ucChangeCoilBuf[0] & (1 << ADDR_DIMMER1_CONFIG)) ) {
       		setDimmerConfigCmd(CH1, ADDR_DIMMER1_CONFIG);
        }
       	// check holding changed
       	if ( (ucChangeCoilBuf[0] & (1 << ADDR_DIMMER1_CONFIG)) == 0
       			&& percent[CH1] != usSRegHoldBuf[ADDR_DIMMER1_STATE]
				&& usSRegHoldBuf[ADDR_DIMMER1_STATE] >= DIMMERMIN && usSRegHoldBuf[ADDR_DIMMER1_STATE] <= DIMMERMAX	 ) {
       		setDimmerPercentCmd(CH1, ADDR_DIMMER1_STATE);
       	}
#endif

	} // main loop
}
