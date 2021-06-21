/*
 * MPU6050.c
 *
 *  Created on: May 14, 2021
 *      Authors: Quatela Alessandro, Roberto Giuseppe
 */

#include "main.h"
#include "MPU6050.h"
#include <math.h>


/* MPU Register Definition*/
#define GY521_ADDR 0xD0
#define WHO_ADDR 0x75
#define PWR_MNG1 0x6B
#define SMPRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

typedef struct {
	float Ax, Ay, Az, Gx, Gy, Gz;
} Measures_t;

Control_Data_t Control_Data;

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t AxOFF = 0;
int16_t AyOFF = 0;
int16_t AzOFF = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

int16_t GxOFF = 0;
int16_t GyOFF = 0;
int16_t GzOFF = 0;


Measures_t MPU6050_Read(I2C_HandleTypeDef hi2c) {
	uint8_t Rec_data[6];

	/* Accel Reading */
	while(HAL_I2C_Mem_Read(&hi2c, GY521_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, Rec_data, 6, 100) != HAL_OK);

	/* Accel Registers Merging */
	Accel_X_RAW = (int16_t)(Rec_data[0] << 8 | Rec_data[1]);
	Accel_Y_RAW = (int16_t)(Rec_data[2] << 8 | Rec_data[3]);
	Accel_Z_RAW = (int16_t)(Rec_data[4] << 8 | Rec_data[5]);

	/* Gyro Reading */
	while(HAL_I2C_Mem_Read(&hi2c, GY521_ADDR, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, Rec_data, 6, 100) != HAL_OK);

	/* Gyro Registers Merging */
	//Gyro_X_RAW = (int16_t)(Rec_data[0] << 8 | Rec_data[1]);
	Gyro_Y_RAW = (int16_t)(Rec_data[2] << 8 | Rec_data[3]);
	//Gyro_Z_RAW = (int16_t)(Rec_data[4] << 8 | Rec_data[5]);

	Measures_t Measures;

	/* Raw Data Scaling */
	Measures.Ax = (Accel_X_RAW - AxOFF) / 16384.0 * 9.81;
	Measures.Ay = (Accel_Y_RAW - AyOFF) / 16384.0 * 9.81;
	Measures.Az = (Accel_Z_RAW - AzOFF) / 16384.0 * 9.81;

	//Measures.Gx = (Gyro_X_RAW - GxOFF) / 131;
	Measures.Gy = (Gyro_Y_RAW - GyOFF) / 131;
	//Measures.Gz = (Gyro_Z_RAW - GzOFF) / 131;

	return Measures;

}

int MPU6050_Init(I2C_HandleTypeDef hi2c, int iter, int timeout) {
	uint8_t check, data;

	/* WHO_AM_I Register check */
	uint16_t t1 = HAL_GetTick();
	while(HAL_I2C_Mem_Read(&hi2c, GY521_ADDR, WHO_ADDR, I2C_MEMADD_SIZE_8BIT, &check, 1, 100) == HAL_BUSY && HAL_GetTick() - t1 <= timeout) {
	}

	if (check == 0x68) {

		/* GY-581 Register Configuration */
	  	data = 8;
	  	HAL_I2C_Mem_Write(&hi2c, GY521_ADDR, PWR_MNG1, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);		// PWR_MGMT_1
	  	HAL_Delay(50);

	  	data = 7;
	  	HAL_I2C_Mem_Write(&hi2c, GY521_ADDR, SMPRT_DIV, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);	 	// SMPRT_DIV
	  	HAL_Delay(50);

	  	data = 0;
	  	HAL_I2C_Mem_Write(&hi2c, GY521_ADDR, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50); 	// GYRO_CONFIG
	  	HAL_Delay(50);

	  	data = 0;
	  	HAL_I2C_Mem_Write(&hi2c, GY521_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);	// ACCEL_CONFIG
	  	HAL_Delay(50);

	  	/* GY-581 Gyroscope and Accelerometer Calibration */
	  	int16_t _GxOFF = 0, _GyOFF = 0, _GzOFF = 0;
	  	int16_t _AxOFF = 0, _AyOFF = 0, _AzOFF = 0;

	  	for (int i=0; i<iter; i++) {
	  		MPU6050_Read(hi2c);
	  		//_GxOFF += Gyro_X_RAW;
	  	  	_GyOFF += Gyro_Y_RAW;
	  	  	//_GzOFF += Gyro_Z_RAW;
	  	    _AxOFF += Accel_X_RAW;
	  	  	_AyOFF += Accel_Y_RAW;
	  	  	_AzOFF += Accel_Z_RAW - 16384.0;
	  	  	HAL_Delay(2);	/* Used to simulate control sampling time */
	  	}

	  	//GxOFF = _GxOFF / iter;
	  	GyOFF = _GyOFF / iter;
	  	//GzOFF = _GzOFF / iter;

	  	AxOFF = _AxOFF / iter;
	  	AyOFF = _AyOFF / iter;
	  	AzOFF = _AzOFF / iter;

	  	Control_Data.angleY = 0;
	  	Control_Data.Gy = 0;

	  	return 0;
	}
	return 1;

}

Control_Data_t MPU6050_Data(I2C_HandleTypeDef hi2c, float dt) {
	/* Measurements processing and angles computation */

	Measures_t Measures = MPU6050_Read(hi2c);

	Control_Data.angleY += dt * (Measures.Gy + Control_Data.Gy) / 2;
	Control_Data.Gy = Measures.Gy;
	float accel_angleY = - atan2(Measures.Ax,sqrt(Measures.Az * Measures.Az)) * 180 / M_PI;

	/* Complementary filter */
	Control_Data.angleY = Control_Data.angleY * 0.999 + accel_angleY * 0.001;

    return Control_Data;

}
