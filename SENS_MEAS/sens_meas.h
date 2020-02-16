/*
 * sens_meas.h
 *
 *  Created on: 14 feb 2020
 *      Author: massimo
 */

#ifndef SENS_MEAS_H_
#define SENS_MEAS_H_

#include "AD.h"
#include <array>
using namespace std;

int aggMemMisureDist(A_D *, array<A_D, 10>&, int *);
void stampaMisureDist(array<A_D, 10>& VA);


#endif /* SENS_MEAS_H_ */
