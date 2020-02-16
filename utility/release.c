/*
 * release.c
 *
 *  Created on: 16 feb 2020
 *      Author: massimo
 */


#include <stdio.h>

void PRINTFL(float f, int numDec);
const char release[] = "1.0.0 - stm32f411re";


void welcome(void){
	printf("\n\nROVER OBII\n\n");
	printf("****************\n");
	printf("numero reale \n\n");
	PRINTFL(1234500000, 7);
	printf("\n\n");
	printf("scritto in C++\n\n");
	printf("\nversione: %s\n", release);
	printf("\n\n\t########\n");
}


