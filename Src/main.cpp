/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Abilitati gli encoder su TIM1_1 e TIM1_2 rispettivamente PA8 e PA9 e gestiti
  * ad interruzione
  * Inseriti i convertitori AD
  * Aggiunto il TIMER2 per generare una base dei tempi.
  * Aggiunto il bus I2C
  * Aggiunti i due segnali PWM per la potenza sui motori: PB4 (D5) e PC7(D9)
  * Gestione della UART2 ad interruzione sia in ricezione che in trasmissione.
  *
  *
  ******************************************************************************
  ******************************************************************************
  ******************************************************************************
  * SVILUPPO DEL RAMO DI TEST DELLA SCHEDA
  */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include <stdio.h>

class APPLICATION{
public:
	APPLICATION(){}
	void RUN();

private:


};

void loop1(void);
void setup(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){

	/* MCU Configuration--------------------------------------------------------*/
	APPLICATION ROVER;
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/// avvio dell'applicazione
	ROVER.RUN();
//	setup();
//
//	while (1){
//		loop1();
//	}
	return 0;
}


void APPLICATION::RUN(){
	setup();
	loop1();

}
///****************************************************** *****END OF FILE****/
