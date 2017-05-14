/*
 * dimmer.h
 *
 *  Created on: 15.04.2017
 *      Author: andru
 */

#ifndef DIMMER_H_
#define DIMMER_H_

#define DIMMERS			2
#define DIMMERMIN		20
#define DIMMERMAX		100

#define EN_CH0			1
#define EN_CH1			1

typedef enum {
	CH0,
	CH1,
} channel_t;

void dimmer_init(void);
uint8_t dimmer_run(channel_t channel, uint8_t percent);
void dimmer_stop(channel_t channel);
uint8_t dimmer_state(channel_t channel);

#endif /* DIMMER_H_ */
