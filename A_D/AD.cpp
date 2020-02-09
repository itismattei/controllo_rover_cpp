/*
 * AD.cpp
 *
 *  Created on: Feb 9, 2020
 *      Author: massimo
 */

#include "AD.h"

A_D::A_D() {
	// TODO Auto-generated constructor stub
	updated = false;

}

A_D::~A_D() {
	// TODO Auto-generated destructor stub
}


void A_D::sample(){
	/// hadc1 e' il gestore del convertitore definito in setup.cpp
	HAL_ADC_Start_DMA(&hadc1, ADCbuffer, 8);
	updated = true;
}
