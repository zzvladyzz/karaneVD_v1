/*
 * LIB_MPU6500.c
 *
 *  Created on: Dec 10, 2025
 *      Author: Vlady-Chuwi
 */
#include "LIB_MPU6500_SPI.h"

#include "gpio.h"
#include "spi.h"




//Funciones

void	MPU6500_Read(MPU6500_Read_t* valoresMPU){
	uint8_t Reg=MPU_READ|ACCEL_XOUT_H;
	uint8_t	Val[6];
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &Reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2,&Val[0],6, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, 1);
	HAL_Delay(1);
	valoresMPU->MPU6500_ACCELX.MPU6500_uint8[1]=Val[0];
	valoresMPU->MPU6500_ACCELX.MPU6500_uint8[0]=Val[1];
	valoresMPU->MPU6500_ACCELY.MPU6500_uint8[1]=Val[2];
	valoresMPU->MPU6500_ACCELY.MPU6500_uint8[0]=Val[3];
	valoresMPU->MPU6500_ACCELZ.MPU6500_uint8[1]=Val[4];
	valoresMPU->MPU6500_ACCELZ.MPU6500_uint8[0]=Val[5];

	Reg=MPU_READ|GYRO_XOUT_H;
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &Reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2,&Val[0],6, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, 1);
	HAL_Delay(1);
	valoresMPU->MPU6500_GYROX.MPU6500_uint8[1]=Val[0];	/////MSB
	valoresMPU->MPU6500_GYROX.MPU6500_uint8[0]=Val[1];	/////LSB
	valoresMPU->MPU6500_GYROY.MPU6500_uint8[1]=Val[2];
	valoresMPU->MPU6500_GYROY.MPU6500_uint8[0]=Val[3];
	valoresMPU->MPU6500_GYROZ.MPU6500_uint8[1]=Val[4];
	valoresMPU->MPU6500_GYROZ.MPU6500_uint8[0]=Val[5];

}
uint8_t 	MPU6500_Read_Reg(uint8_t Reg){
	uint8_t Value=0;
	Reg=Reg|MPU_READ;
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &Reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2,&Value, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, 1);
	return Value;
}
void 	MPU6500_Write_Reg(uint8_t Reg,uint8_t value){
	Reg=Reg|MPU_WRITE;
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &Reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi2,&value, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, 1);
	HAL_Delay(1);
}
void	MPU6500_Write(uint8_t Reg,uint8_t* value, uint8_t  len){
	Reg=Reg|MPU_WRITE;
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &Reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi2, value, len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, 1);
	HAL_Delay(1);
}

MPU6500_status_e	MPU6500_Init(MPU6500_Read_t * offset,uint8_t N,uint8_t dps,uint8_t g){

	double PromX[N],PromY[N],PromZ[N];
	double SumaX,SumaY,SumaZ=0;

	uint8_t status_mpu=MPU6500_Read_Reg(WHO_AM_I);
	if (status_mpu!=0x70) {
		return MPU6500_fail;
	}

	MPU6500_Write_Reg(PWR_MGMT_1, 0b10000000);
	MPU6500_Write_Reg(SIGNAL_PATH_RESET, 0b00000111);
	//colocamos a 1000dps y 16 g para offset
	MPU6500_Write_Reg(CONFIG_ACCEL, G16);
	MPU6500_Write_Reg(CONFIG_GYRO, DPS1000);

	/////////////////////Mejorar codigo muestreo//////////////////////////
	for (uint8_t n = 0; n < N; ++n) {
		MPU6500_Read(offset);
		PromX[n]=(double)offset->MPU6500_ACCELX.MPU6500_int16;
		PromY[n]=(double)offset->MPU6500_ACCELY.MPU6500_int16;
		PromZ[n]=(double)offset->MPU6500_ACCELZ.MPU6500_int16;
		HAL_Delay(500);
	}
	for (uint8_t n = 0; n < N; ++n) {
		SumaX=SumaX+PromX[n];
		SumaY=SumaY+PromY[n];
		SumaZ=SumaZ+PromZ[n];
	}
	offset->MPU6500_ACCELX.MPU6500_int16=(int16_t)SumaX/N;
	offset->MPU6500_ACCELY.MPU6500_int16=(int16_t)SumaY/N;
	offset->MPU6500_ACCELZ.MPU6500_int16=(int16_t)SumaZ/N;
	/////////////////////////////////////////////////////////////////////////////
	///////////////Leemos offset de ACCEL y luego restamos offset
	uint8_t hr=0;
	uint8_t lr=0;
	uint16_t valor=0;
	int16_t  resta=0;


	hr=MPU6500_Read_Reg(OFFSET_H_AX);
	lr=MPU6500_Read_Reg(OFFSET_L_AX);
	valor=(((hr<<8)|lr))>>1;
	resta=(int16_t)valor;
	resta=resta-((offset->MPU6500_ACCELX.MPU6500_int16)/2);// segun se debe hacerlo con el offset pero si no divido entre 2 el valor se duplica
	hr=resta>>7;
	lr=resta<<1;
	MPU6500_Write_Reg(OFFSET_H_AX, hr);
	MPU6500_Write_Reg(OFFSET_L_AX, lr);

	hr=MPU6500_Read_Reg(OFFSET_H_AY);
	lr=MPU6500_Read_Reg(OFFSET_L_AY);
	valor=(((hr<<8)|lr))>>1;
	resta=(int16_t)valor;
	resta=resta-((offset->MPU6500_ACCELY.MPU6500_int16)/2);// segun se debe hacerlo con el offset pero si no divido entre 2 el valor se duplica
	hr=resta>>7;
	lr=resta<<1;
	MPU6500_Write_Reg(OFFSET_H_AY, hr);
	MPU6500_Write_Reg(OFFSET_L_AY, lr);

	hr=MPU6500_Read_Reg(OFFSET_H_AZ);
	lr=MPU6500_Read_Reg(OFFSET_L_AZ);
	valor=(((hr<<8)|lr))>>1;
	offset->MPU6500_ACCELZ.MPU6500_int16=(offset->MPU6500_ACCELZ.MPU6500_int16)-2048;
	resta=(int16_t)valor;
	resta=resta-((offset->MPU6500_ACCELZ.MPU6500_int16)/2);// segun se debe hacerlo con el offset pero si no divido entre 2 el valor se duplica
	hr=resta>>7;
	lr=resta<<1;
	MPU6500_Write_Reg(OFFSET_H_AZ, hr);
	MPU6500_Write_Reg(OFFSET_L_AZ, lr);
	///////////////////////////////
	offset->MPU6500_GYROX.MPU6500_int16=-(offset->MPU6500_GYROX.MPU6500_int16);
	MPU6500_Write_Reg(OFFSET_H_GX, (offset->MPU6500_GYROX.MPU6500_uint8[1]));
	MPU6500_Write_Reg(OFFSET_L_GX, (offset->MPU6500_GYROX.MPU6500_uint8[0]));

	offset->MPU6500_GYROY.MPU6500_int16=-(offset->MPU6500_GYROY.MPU6500_int16);
	MPU6500_Write_Reg(OFFSET_H_GY, (offset->MPU6500_GYROY.MPU6500_uint8[1]));
	MPU6500_Write_Reg(OFFSET_L_GY, (offset->MPU6500_GYROY.MPU6500_uint8[0]));

	offset->MPU6500_GYROZ.MPU6500_int16=-(offset->MPU6500_GYROZ.MPU6500_int16);
	MPU6500_Write_Reg(OFFSET_H_GZ, (offset->MPU6500_GYROZ.MPU6500_uint8[1]));
	MPU6500_Write_Reg(OFFSET_L_GZ, (offset->MPU6500_GYROZ.MPU6500_uint8[0]));



////colocamos valores a usar
	MPU6500_Write_Reg(CONFIG_ACCEL, g);
	MPU6500_Write_Reg(CONFIG_GYRO, dps);


	return MPU6500_ok;

}
MPU6500_float_t		MPU6500_Scale(MPU6500_Read_t* raw,float dpsConv,float gConv)
{
	MPU6500_float_t convDatos;
	convDatos.MPU6500_floatAX=(raw->MPU6500_ACCELX.MPU6500_int16)/gConv;
	convDatos.MPU6500_floatAY=(raw->MPU6500_ACCELY.MPU6500_int16)/gConv;
	convDatos.MPU6500_floatAZ=(raw->MPU6500_ACCELZ.MPU6500_int16)/gConv;

	convDatos.MPU6500_floatGX=(raw->MPU6500_GYROX.MPU6500_int16)/dpsConv;
	convDatos.MPU6500_floatGY=(raw->MPU6500_GYROY.MPU6500_int16)/dpsConv;
	convDatos.MPU6500_floatGZ=(raw->MPU6500_GYROZ.MPU6500_int16)/dpsConv;

	return convDatos;
}
float		MPU6500_Pitch(MPU6500_float_t* convPitch){

	  float accelX=convPitch->MPU6500_floatAX;
	  float accelY=convPitch->MPU6500_floatAY;
	  float accelZ=convPitch->MPU6500_floatAZ;
	  float accelY_sq=accelY*accelY;
	  float accelZ_sq=accelZ*accelZ;
	  float angulo_pitch=atan2(accelX,sqrt(accelY_sq+accelZ_sq));
	  angulo_pitch=angulo_pitch*(180.0/PI);
	  return angulo_pitch;
}
float		MPU6500_Roll(MPU6500_float_t* convRoll){
	//float accelX=convRoll->MPU6500_floatAX;
	float accelY=convRoll->MPU6500_floatAY;
	float accelZ=convRoll->MPU6500_floatAZ;
	float angulo_roll=atan2(accelY,accelZ);
	angulo_roll=angulo_roll*(180.0/PI);
	return angulo_roll;
}

MPU6500_float_t		MPU6500_ConvRad(MPU6500_float_t* conv);



