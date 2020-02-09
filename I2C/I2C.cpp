/*
 * I2C.cpp
 *
 *  Created on: 29 dic 2019
 *      Author: massimo
 */

#include "I2C.h"
#include <stdio.h>

I2C::I2C() {
	// TODO Auto-generated constructor stub
	stato = NON_IMPOSTATA;
	hi2c = NULL;
}

I2C::I2C(I2C_HandleTypeDef *p){
	if (p != NULL)
		hi2c = p;
}

uint32_t I2C::I2CGetN(uint8_t reg, uint8_t numElem, uint8_t buff[]){

	HAL_StatusTypeDef a = HAL_ERROR;
	if (hi2c != NULL){
		uint8_t ADDRESS = (SLAVE_ADD << 1);
		 a = HAL_I2C_Mem_Read(hi2c, ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buff, numElem, 300);
	}
	return (uint32_t)a;
}

I2C::~I2C() {
	// TODO Auto-generated destructor stub
}


uint32_t I2C::I2CGetN(uint8_t numElem, uint8_t buff[]){
	uint8_t ADDRESS = (SLAVE_ADD << 1);
	if(HAL_I2C_Master_Receive(hi2c, ADDRESS, buff, numElem, 300) != HAL_OK){
		  Error_Handler();
	}
	return I2C_OK;
}


//sends an I2C command to the specified slave
void I2C::I2CSend(uint8_t num_of_args, uint8_t REG, uint8_t buff[]){

	uint8_t ADDRESS = (SLAVE_ADD << 1);
    if(num_of_args == 0){
        // trasmette un solo dato
    	uint8_t dummy = 0;
    	/// il dato trasmesso e' uno 0 poiché il suo valore e'
    	/// irrilevante per lo slave
    	if(HAL_I2C_Master_Transmit(hi2c, ADDRESS, &dummy, 1, 300) != HAL_OK){
			  Error_Handler();
		}
    }
    else{
    	if(HAL_I2C_Mem_Write(hi2c, ADDRESS, REG, I2C_MEMADD_SIZE_8BIT, buff, num_of_args, 300) != HAL_OK){
		  Error_Handler();
		  printf("scrittura campionamento errata\n\n");
    	}
    }
}


