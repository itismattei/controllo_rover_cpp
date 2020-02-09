/*
 * I2C.h
 *
 *  Created on: 29 dic 2019
 *      Author: massimo
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"


#define		IMPOSTATA			1
#define		NON_IMPOSTATA		3
#define		ERRORE_COM			2
#define		I2C_OK				0

void Error_Handler(void);

class I2C {
public:
	I2C();
	I2C(I2C_HandleTypeDef *);
	virtual ~I2C();

	void InitI2C(uint32_t);
	void I2CSetSlave_Add(uint8_t sa){SLAVE_ADD = sa;}
	uint32_t I2CGetN(uint8_t reg, uint8_t numElem, uint8_t buff[]);
	uint32_t I2CGetN(uint8_t numElem, uint8_t buff[]);
	uint32_t I2CGet(uint8_t reg);
	void I2CSend(uint8_t num_of_args, uint8_t, uint8_t *);
	void I2CPut_buff(char array[]);


public:
	/// proprieta'
	uint32_t 			BASE_ADDR;
	uint8_t				SLAVE_ADD;
	uint16_t			stato;
	I2C_HandleTypeDef 	*hi2c;
};


#endif /* I2C_H_ */
