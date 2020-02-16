/*
 * imu.h
 *
 *  Created on: 15 feb 2020
 *      Author: massimo
 */

#ifndef IMU_H_
#define IMU_H_


///////////////////////////////////////////////
///// ADDRESS
///////////////////////////////////////////////
#define		GYRO_ADDR		0x6B  /// oppure 0x6A dipendentemente da SDO_A/G


///////////////////////////////////////////////
///
///  		GYROSCOPE REGISTERS ADDRESSES
///
///////////////////////////////////////////////

#define		WHO_AM_I		0x0F
#define		CTRL_REG1		0x20
#define		CTRL_REG2		0x21
#define		CTRL_REG3		0x22
#define		CTRL_REG4		0x23
#define		CTRL_REG5		0x24

#define		OUT_TEMP		0x26
#define		STATUS_REG		0x27
/////
/////  DATA REGISTER
#define		OUT_X_L 		0x28
#define		OUT_X_H 		0x29
#define		OUT_Y_L 		0x2A
#define		OUT_Y_H 		0x2B
#define		OUT_Z_L 		0x2C
#define		OUT_Z_H 		0x2D

////
//// fifo  register
#define		FIFO_CTRL_REG 	0x2E
#define		FIFO_SRC_REG 	0x2F

////
//// USEFULL BITS

#define		ZYXOR 			0x80
#define		ZOR 			0x40
#define		YOR 			0x20
#define		XOR 			0x10
#define		ZYXDA 			0x08
#define		ZDA 			0x04
#define		YDA 			0x02
#define		XDA				0x01

/*0 0 0 Bypass mode
0 0 1 FIFO mode
0 1 0 Stream mode
0 1 1 Stream-to-FIFO mode
1 0 0 Bypass-to-Stream mode*/
#define 	BYPASS_MODE			0x00
#define		FIFO_MODE			32
#define		STREAM_MODE			64
#define		STREAM_TO_FIFO		96
#define		BYPASS_TO_STREAM	128
#define		WTM0				1
#define		WTM1				2
#define		WTM2				4
#define		WTM3				8
#define		WTM4				16

//I1_Int1 I1_Boot H_Lactive PP_OD I2_DRDY I2_WTM I2_ORun I2_Empty

#define 	I1_INT1				128
#define 	I1_BOOT				64
#define 	H_LACTIVE			32
#define 	PP_OD				16
#define		I2_DRDY				8
#define		I2_WTM				4
#define		I2_ORUN				2
#define		I2_EMPTY			1

/// data rate ODR
#define		ODR_95				0
#define		ODR_190				64

#define		FS0					16
#define		FS1					32
#define		FS_250				0
#define		FS_500				16
#define		FS_2000				32

#define		FIFO_EN				64
#define 	MUL_READ			0x80

#define		ALL_AXIS			0x7
#define		Z_AXIS				0x4

/// REGISTER VALUE
#define		WOH_AM_I_value		0x68	///LSM9DS1


#include "genDef.h"
#include <stdint.h>
#include "I2C.h"

class Jitter {
public:
	Jitter();
	virtual ~Jitter();
	inline uint32_t setActualGyro() {return 0;}

	uint32_t jitter_timerGyro, prevValueGyro;
	uint32_t jitter_timerPID, prevValuePID;
};




#define		maxDimBuff		256
#define		numSampleBias	128

class Giroscopio {
public:
	Giroscopio();
	virtual ~Giroscopio();

	void initGyro(char);
	void setupAssi(char stato);
	void misuraAngoli(Jitter *J);
	void azzeraAssi(void);
	void printAsseZ(int);
	int getTemp();
	void primoAzzeramento(void);
	void attachI2C(I2C *, uint8_t sa);

	void setYaw(int16_t valore);

	int16_t buffValori[maxDimBuff];
	int16_t buffX[maxDimBuff];
	int16_t tempReg;
	float  media;
	float m, q;


//private:
	char IsPresent;
	char IsOn;
	/// tiene il setup dell'asse
	int16_t x0, y0, z0;
	/// indica quali assi sono attivi
	char asseOn;
	/// valore degli angoli misurati da integrazione della rotazione
	/// i valori sono in gradi in modo da non dover memorizzare dei float
	int16_t yaw; 	/// azze z
	int16_t roll; 	/// asse y
	int16_t pitch;	/// asse x

	/// valori attuali degli assi
	float yawF, yawF0;
	float rollF;
	float pitchF;
	/// valori precedenti degli assi
	float yawP, rollP, pitchP;
	/// valori di gradi al secondo dedotti dal manuale del giroscopio
	int16_t gradiSec;
	/// fondo scala (°/s)
	float FS;
	/// intervallo di integrazione in ms
	float tick;
	/// fattori di scala per ciscun asse
	float kz, ky, kx;
	/// temepratura
	int temp;
	/// viene attiato dal pid durante la rotazione in modo da non resettare il giroscopio.
	char IsRotating;
	char offsetRequest, offsetDelayed;
	char rev[10];
	/// fattore di correzione del jitter nell'intgrazione dell'angolo
	float corr;
private:

	uint16_t posizione;
	I2C *i2cPtr;

};

uint8_t printFloat(double number, uint8_t digits);

void minQuad(int16_t *, int16_t *, int, float *, float *);


#endif /* IMU_H_ */
