/*
 * AD.cpp
 *
 *  Created on: Feb 9, 2020
 *      Author: massimo
 */

#include "AD.h"
#include <stdio.h>

A_D::A_D() {
	// TODO Auto-generated constructor stub
	updated = false;

}


A_D::~A_D() {
	// TODO Auto-generated destructor stub
}



void A_D::sample(void){
	/// hadc1 e' il gestore del convertitore definito in setup.cpp
	HAL_ADC_Start_DMA(&hadc1, ADCbuffer, 8);

}


void A_D::regCpy(void){
	for (int i = 0; i < 5; i++){
		distanza[i] = (uint16_t)ADCbuffer[i];
		Sgas	=	(uint16_t)ADCbuffer[5];
		Nota	=	(uint16_t)ADCbuffer[6];
		BATT	= 	(uint16_t)ADCbuffer[7];
	}
}


void A_D::stampa(void){
	for (int i = 0; i < 5; i++)
		printf("distanza %d = %d\n", i, (int)distanza[i]);
	printf("gas =\t %d\n", (int)Sgas);
	printf("BATT =\t %d\n", (int) BATT);
	printf("nota =\t %d\n", (int) Nota);
}
