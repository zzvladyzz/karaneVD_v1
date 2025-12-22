/*
 * MOD_spi.h
 *
 *  Created on: Dec 4, 2025
 *      Author: vlady-HP
 */

#ifndef INC_LIB_MPU6500_SPI_H_
#define INC_LIB_MPU6500_SPI_H_

#ifndef PI
#define PI	3.14159265358979323846
#endif

#ifndef ALPHA
#define AlPHA	0.98
#endif

#include <stdint.h>
#include <math.h>


// definiciones registros MPU6500
#define	MPU_READ		0X80
#define	MPU_WRITE		0X00


#define WHO_AM_I		0X75

#define ACCEL_XOUT_H	0X3B
#define ACCEL_XOUT_L	0X3C
#define ACCEL_YOUT_H	0X3D
#define ACCEL_YOUT_L	0X3E
#define ACCEL_ZOUT_H	0X3F
#define ACCEL_ZOUT_L	0X40

#define TEMP_OUT_H		0X41
#define TEMP_OUT_L		0X42

#define GYRO_XOUT_H		0X43
#define GYRO_XOUT_L		0X44
#define GYRO_YOUT_H		0X45
#define GYRO_YOUT_L		0X46
#define GYRO_ZOUT_H		0X47
#define GYRO_ZOUT_L		0X48

#define SELF_TEST_GX	0X00
#define SELF_TEST_GY	0X01
#define SELF_TEST_GZ	0X02
#define SELF_TEST_AX	0X0D
#define SELF_TEST_AY	0X0E
#define SELF_TEST_AZ	0X0F

#define OFFSET_H_GX		0X13
#define OFFSET_L_GX		0X14
#define OFFSET_H_GY		0X15
#define OFFSET_L_GY		0X16
#define OFFSET_H_GZ		0X17
#define OFFSET_L_GZ		0X18

#define OFFSET_H_AX		0X77
#define OFFSET_L_AX		0X78
#define OFFSET_H_AY		0X7A
#define OFFSET_L_AY		0X7B
#define OFFSET_H_AZ		0X7D
#define OFFSET_L_AZ		0X1E



#define CONFIG			0X1A
#define CONFIG_GYRO		0X1B
#define CONFIG_ACCEL	0X1C
#define CONFIG_ACCEL_2	0X1D

#define SIGNAL_PATH_RESET	0X68
#define USER_CTRL			0X6A
#define PWR_MGMT_1			0X6B
#define PWR_MGMT_2			0X6C


#define	DPS2000			0B00011000
#define	DPS1000			0b00010000	//usado para calibrar
#define	DPS500			0b00001000
#define	DPS250			0b00000000

#define G16				0b00011000
#define G8				0b00010000
#define G4				0b00001000
#define G2				0b00000000

#define DPS2000_CONV	4000
#define DPS1000_CONV	2000
#define DPS500_CONV		1000
#define DPS250_CONV		500

#define G16_CONV	32
#define G8_CONV		16
#define G4_CONV		8
#define G2_CONV		4

//creamos una union para los valores ya que registro nos entrega en dos registros
typedef union{
	uint16_t MPU6500_uint16;
	int16_t  MPU6500_int16;
	uint8_t  MPU6500_uint8[2];
	int8_t   MPU6500_int8[2];
}MPU6500_int_u;

//Creamos una struct donde leeremos tanto offset como datos
typedef struct{
	MPU6500_int_u MPU6500_GYROX;
	MPU6500_int_u MPU6500_GYROY;
	MPU6500_int_u MPU6500_GYROZ;
	MPU6500_int_u MPU6500_ACCELX;
	MPU6500_int_u MPU6500_ACCELY;
	MPU6500_int_u MPU6500_ACCELZ;
}MPU6500_Read_t;

typedef struct{
	float MPU6500_floatGX;
	float MPU6500_floatGY;
	float MPU6500_floatGZ;
	float MPU6500_floatAX;
	float MPU6500_floatAY;
	float MPU6500_floatAZ;
}MPU6500_float_t;

typedef enum{MPU6500_ok,MPU6500_fail}MPU6500_status_e;


//Para ver que no sucede un error al inicializar mpu y
// Reset registros y validar ID y realizar primeras lecturas offset

MPU6500_status_e 		MPU6500_Init(MPU6500_Read_t* offset,uint8_t N,uint8_t dps,uint8_t g);// Mejorar codigo muestreo
void		MPU6500_Read(MPU6500_Read_t* valoresMPU);
uint8_t 	MPU6500_Read_Reg(uint8_t Reg);
void 		MPU6500_Write_Reg(uint8_t Reg,uint8_t value);
void		MPU6500_Write(uint8_t Reg,uint8_t *value, uint8_t len);
MPU6500_float_t		MPU6500_Scale(MPU6500_Read_t* raw,float dpsConv,float gConv);
MPU6500_float_t		MPU6500_ConvRad(MPU6500_float_t* conv);
float		MPU6500_Pitch(MPU6500_float_t* convPitch);
float		MPU6500_Roll(MPU6500_float_t* convRoll);





#endif /* INC_LIB_MPU6500_SPI_H_ */
