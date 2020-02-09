/*
 * setup.h
 *
 *  Created on: 9 feb 2020
 *      Author: massimo
 */

#ifndef SETUP_H_
#define SETUP_H_


#define  SEC	1000

/// buffer per leggere il convertitore AD
bool ADupdate = false;
uint32_t ADCbuffer[8];

void toggleLed(void);

#endif /* SETUP_H_ */
