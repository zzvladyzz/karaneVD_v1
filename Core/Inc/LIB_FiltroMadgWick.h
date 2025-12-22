/*
 * FiltroMadgWick.h
 *
 *  Created on: Dec 12, 2025
 *      Author: Vlady-Chuwi
 */

#ifndef INC_LIB_FILTROMADGWICK_H_
#define INC_LIB_FILTROMADGWICK_H_


#include	"LIB_MPU6500_SPI.h"
#include <stdint.h>

#ifndef DPS_TO_RADS
#define DPS_TO_RADS	0.0174533f
#endif

#define SampleFrequency		100		//Frecuencia en HZ
#define betaDef				0.3f	//Ganancia proporcional 2

typedef struct{
	float beta;
	float q0;
	float q1;
	float q2;
	float q3;
	float roll;
	float pitch;
	float yaw;
	float invSampleFrequency;
	uint8_t anglesComputed;
	float gx;
	float gy;
	float gz;
	float ax;
	float ay;
	float az;

}MadgWick_t;

void madgwickInit(MadgWick_t* init);
//valor en DPS dentro de funcion lo convierte a rad/s
void  madgwickUpdateIMU(MadgWick_t* datos,MPU6500_float_t* mpu6500_float_DPS_G);
void  madgwickGetAngles(MadgWick_t* angles);

float madgwickGetRollRadians(MadgWick_t* angles);
float madgwickGetPitchRadians(MadgWick_t* angles);
float madgwickGetYawRadians(MadgWick_t* angles);
float invSqrt(float x);

#endif /* INC_LIB_FILTROMADGWICK_H_ */
