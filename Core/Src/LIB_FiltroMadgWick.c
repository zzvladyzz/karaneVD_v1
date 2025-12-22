/*
 * LIB_FiltroMadgWick.c
 *
 *  Created on: Dec 12, 2025
 *      Author: Vlady-Chuwi
 */
#include	"LIB_FiltroMadgWick.h"
void madgwickInit(MadgWick_t* init){
	init->beta=betaDef;
	init->q0=1.0f;
	init->q1=0;
	init->q2=0;
	init->q3=0;
	init->invSampleFrequency=1.0f/SampleFrequency;
	init->roll=0;
	init->pitch=0;
	init->yaw=0;
	init->anglesComputed=0;
	init->ax=0;
	init->ay=0;
	init->az=0;

	init->gx=0;
	init->gy=0;
	init->gz=0;

}

void madgwickUpdateIMU(MadgWick_t* qD,MPU6500_float_t* mpu6500_float_DPS_G)
{

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	// Convert gyroscope degrees/sec to radians/sec
	qD->gx=mpu6500_float_DPS_G->MPU6500_floatGX;
	qD->gy=mpu6500_float_DPS_G->MPU6500_floatGY;
	qD->gz=mpu6500_float_DPS_G->MPU6500_floatGZ;
	qD->ax=mpu6500_float_DPS_G->MPU6500_floatAX;
	qD->ay=mpu6500_float_DPS_G->MPU6500_floatAY;
	qD->az=mpu6500_float_DPS_G->MPU6500_floatAZ;


	(qD->gx) *= DPS_TO_RADS;
	(qD->gy) *= DPS_TO_RADS;
	(qD->gz) *= DPS_TO_RADS;

		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * ((-qD->q1) * qD->gx - qD->q2 * qD->gy - qD->q3 * qD->gz);
		qDot2 = 0.5f * (qD->q0 * (qD->gx) + (qD->q2) * (qD->gz) - (qD->q3) * (qD->gy));
		qDot3 = 0.5f * (qD->q0 * (qD->gy) - (qD->q1) * (qD->gz) + (qD->q3) * (qD->gx));
		qDot4 = 0.5f * (qD->q0 * (qD->gz) + (qD->q1) * (qD->gy) - (qD->q2) * (qD->gx));

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((qD->ax == 0.0f) && (qD->ay == 0.0f) && (qD->az == 0.0f))) {

			// Normalise accelerometer measurement
			recipNorm = invSqrt((qD->ax) * (qD->ax) + (qD->ay) * (qD->ay) + (qD->az) * (qD->az));
			(qD->ax) *= recipNorm;
			(qD->ay) *= recipNorm;
			(qD->az) *= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			_2q0 = 2.0f * qD->q0;
			_2q1 = 2.0f * qD->q1;
			_2q2 = 2.0f * qD->q2;
			_2q3 = 2.0f * qD->q3;
			_4q0 = 4.0f * qD->q0;
			_4q1 = 4.0f * qD->q1;
			_4q2 = 4.0f * qD->q2;
			_8q1 = 8.0f * qD->q1;
			_8q2 = 8.0f * qD->q2;
			q0q0 = qD->q0 * qD->q0;
			q1q1 = qD->q1 * qD->q1;
			q2q2 = qD->q2 * qD->q2;
			q3q3 = qD->q3 * qD->q3;

			// Gradient decent algorithm corrective step
			s0 = _4q0 * q2q2 + _2q2 * qD->ax + _4q0 * q1q1 - _2q1 * qD->ay;
			s1 = _4q1 * q3q3 - _2q3 * qD->ax + 4.0f * q0q0 * qD->q1 - _2q0 * qD->ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * qD->az;
			s2 = 4.0f * q0q0 * qD->q2 + _2q0 * qD->ax + _4q2 * q3q3 - _2q3 * qD->ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * qD->az;
			s3 = 4.0f * q1q1 * qD->q3 - _2q1 * qD->ax + 4.0f * q2q2 * qD->q3 - _2q2 * qD->ay;
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= qD->beta * s0;
			qDot2 -= qD->beta * s1;
			qDot3 -= qD->beta * s2;
			qDot4 -= qD->beta * s3;
		}
		// Integrate rate of change of quaternion to yield quaternion
		qD->q0 += qDot1 * qD->invSampleFrequency;
		qD->q1 += qDot2 * qD->invSampleFrequency;
		qD->q2 += qDot3 * qD->invSampleFrequency;
		qD->q3 += qDot4 * qD->invSampleFrequency;

		// Normalise quaternion
		recipNorm = invSqrt(qD->q0 * qD->q0 + qD->q1 * qD->q1 + qD->q2 * qD->q2 + qD->q3 * qD->q3);
		qD->q0 *= recipNorm;
		qD->q1 *= recipNorm;
		qD->q2 *= recipNorm;
		qD->q3 *= recipNorm;

		qD->roll = atan2f(qD->q0*qD->q1 + qD->q2*qD->q3, 0.5f - qD->q1*qD->q1 - qD->q2*qD->q2);
		qD->pitch = asinf(-2.0f * (qD->q1*qD->q3 - qD->q0*qD->q2));
		qD->yaw = atan2f(qD->q1*qD->q2 + qD->q0*qD->q3, 0.5f - qD->q2*qD->q2 - qD->q3*qD->q3);

		qD->roll=qD->roll*57.29578f;
		qD->pitch=qD->pitch*57.29578f;
		qD->yaw=qD->yaw*57.29578f+180.0f;

}

//void  madgwickGetAngles(MadgWick_t* angles);

//float madgwickGetRollRadians(MadgWick_t* angles);
//float madgwickGetPitchRadians(MadgWick_t* angles);
//float madgwickGetYawRadians(MadgWick_t* angles);


float invSqrt(float x)
{
	float halfx = 0.5f * x;
		float y = x;
		long i = *(long*)&y;
		i = 0x5f3759df - (i>>1);
		y = *(float*)&i;
		y = y * (1.5f - (halfx * y * y));
		y = y * (1.5f - (halfx * y * y));
		return y;
}



