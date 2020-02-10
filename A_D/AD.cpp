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
		distanza[i] = ADCbuffer[i];
		Sgas	=	ADCbuffer[5];
		Nota	=	ADCbuffer[6];
		BATT	= 	ADCbuffer[7];
	}
}


void A_D::stampa(void){
	for (int i = 0; i < 5; i++)
		printf("distanza %d = %d\n", i, distanza[i]);
	printf("gas =\t %d\n",Sgas);
	printf("BATT =\t %d\n", BATT);
	printf("nota =\t %d\n", Nota);
}
