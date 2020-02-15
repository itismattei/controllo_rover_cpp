/*
 * sens_meas.cpp
 *
 *  Created on: 14 feb 2020
 *      Author: massimo
 */

#include "sens_meas.h"





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
