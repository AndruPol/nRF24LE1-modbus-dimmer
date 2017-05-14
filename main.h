/*
 * main.h
 *
 *  Created on: 5.05.2017
 *      Author: andru
 */

#ifndef MAIN_H_
#define MAIN_H_

#define EN_LED		1	// LED enable
#define EN_WDG		1	// watchdog enable
#define EN_SW0		1	// external switch  0 enable
#define EN_SW1		1	// external switch  1 enable

#include <stdint.h>
#include "modbus_slave.h"
#include "dimmer.h"

typedef struct chconfig chconfig_t;
struct chconfig {
	uint8_t enable;		// dimmer state
	uint8_t percent;	// dimmer percent
};

// NVM configuration data for nRF24LE1
typedef struct CONFIG CONFIG_T;
struct CONFIG {
	uint8_t magic;				// configuration magic word
	uint8_t addr;				// modbus address
	mb_bitrate_t bitrate;		// bitrate
	mb_parity_t parity;			// parity
	chconfig_t chcfg[DIMMERS];	// dimmers config
};

typedef enum {
	CMD_OK,
	CFG_READ,
	CFG_WRITE,
	CMD_PARAM,
	CMD_DIMMER,
} error_t;

extern CONFIG_T config;
void halt(void);
void setwatchdog(void);

#endif /* MAIN_H_ */
