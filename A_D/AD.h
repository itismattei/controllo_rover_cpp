/*
 * AD.h
 *
 *  Created on: Feb 9, 2020
 *      Author: massimo
 */

#ifndef AD_H_
#define AD_H_

#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern uint32_t ADCbuffer[];

class A_D {
public:
	A_D();
	virtual ~A_D();

	void sample(void);
	bool updated;
};

#endif /* AD_H_ */
