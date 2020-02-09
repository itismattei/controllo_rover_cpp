/*
 * AD.h
 *
 *  Created on: Feb 9, 2020
 *      Author: massimo
 */

#ifndef AD_H_
#define AD_H_

/*
 *   * ASSEGNAZIONE PIN E CANALI ADC:
  * IN0		-	PA0	(A0)
  * IN1		- 	PA1	(A1)
  * IN4 	-	PA4	(A2)
  * IN8		-	PB0	(A3)
  * IN11	-	PC1	(A4)
  * ----------------------
  * GAS
  * IN10	-	PC0	(A5)
  * ----------------------
  * NOTA
  * IN6		-	PA6	(D12)
  * ----------------------
  * LIVELLO BATTERIA
  * IN7		-	PA7 (D11)
 *
 * */

#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern uint32_t ADCbuffer[];

class A_D {
public:
	A_D();
	virtual ~A_D();

	void sample(void);
	void regCpy(void);

	bool updated;
	uint32_t distanza[5];
	uint32_t Sgas;
	uint32_t Nota;
	uint32_t BATT;
};

#endif /* AD_H_ */
