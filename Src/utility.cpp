/*
 * utility.cpp
 *
 *  Created on: 9 feb 2020
 *      Author: massimo
 */


#include "main.h"
#include <stdbool.h>

bool ADup = false;

extern bool ADupdate;

void toggleLed(void){
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}
