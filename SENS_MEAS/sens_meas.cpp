/*
 * sens_meas.cpp
 *
 *  Created on: 14 feb 2020
 *      Author: massimo
 */

#include "sens_meas.h"




/// carica gli ultimi 10 valori letti dal convertitore AD in un array di oggetti A_D
int aggMemMisureDist(A_D *D, array<A_D, 10>& VA, int *i){

	int stato = -1;
	int ind = *i;
	if (ind <= 9){
		VA[ind] = *D;
		stato = 0;
		ind++;
		ind = ind % 10;
		*i = ind;
	}
	return stato;
}

/// stampa i 10 valori memorizzati
void stampaMisureDist(array<A_D, 10>& VA){
	int i , j;
	for (i = 0; i < 10; i++){
		for (j = 0; j < 5; j++)
			printf("dist: %d\n", (int)VA[i].distanza[j]);
		printf("gas: %d\n", (int)VA[i].Sgas);
		printf("nota: %d\n", (int)VA[i].Nota);
		printf("BATT: %d\n", (int)VA[i].BATT);
	}
	printf("***                §§               ***\n");
	HAL_Delay(50);
}
